/*
 * Copyright (c) 2024, Carnegie Mellon University
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <enso/helpers.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>

#include "example_helpers.h"
#include "schedPerf.h"

#define TX_BUFFER_MAX_SIZE 131072
#define INTEL_FPGA_PCIE_BDF "65:00.0"
#define MIN_PACKET_SIZE 64

Client::Client(const ClientConfig& config) { startClient(config); }

void Client::fillPipeWithPackets(uint8_t* pipeBuf, uint32_t& alignedBytes,
                                 uint32_t& rawBytes, uint32_t& pkts) {
  uint32_t initBufLength = alignedBytes;
  uint32_t initGoodBytes = rawBytes;
  uint32_t initNumPkts = pkts;
  while ((alignedBytes + initBufLength) <= TX_BUFFER_MAX_SIZE) {
    memcpy(pipeBuf + alignedBytes, pipeBuf, initBufLength);
    alignedBytes += initBufLength;
    rawBytes += initGoodBytes;
    pkts += initNumPkts;
  }
}

void Client::pcapPktHandler(u_char* user, const struct pcap_pkthdr* pktHeader,
                            const u_char* pktBytes) {
  (void)pktHeader;
  struct PcapHandler* context = (struct PcapHandler*)user;

  const struct ether_header* l2_hdr = (struct ether_header*)pktBytes;
  if (l2_hdr->ether_type != htons(ETHERTYPE_IP)) {
    std::cerr << "Non-IPv4 packets are not supported" << std::endl;
    exit(1);
  }

  uint32_t len = enso::get_pkt_len(pktBytes);
  uint32_t nb_flits = (len - 1) / MIN_PACKET_SIZE + 1;
  TxPipe* pipe = context->dev->AllocateTxPipe();
  if (!pipe) {
    std::cerr << "Problem creating TX pipe" << std::endl;
    exit(2);
  }
  uint8_t* buf = (uint8_t*)malloc(TX_BUFFER_MAX_SIZE * sizeof(uint8_t));
  struct EnsoTxPipe etp(pipe, buf);
  memcpy(buf, pktBytes, len);
  etp.nb_aligned_bytes = nb_flits * MIN_PACKET_SIZE;
  etp.nb_raw_bytes = len;
  etp.nb_pkts = 1;
  context->client->fillPipeWithPackets(buf, etp.nb_aligned_bytes,
                                       etp.nb_raw_bytes, etp.nb_pkts);
  context->txPipes.push_back(etp);
}

void Client::runTx(std::vector<enso::tx_stats_t>& stats,
                   struct EnsoTxPipe& pipe) {
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "Running on core " << sched_getcpu() << std::endl;
  while (ProgramConfig::keepRunning) {
    // send the packets
    uint8_t* pipeBuf = (uint8_t*)pipe.tx_pipe->AllocateBuf(TX_BUFFER_MAX_SIZE);
    if (pipeBuf == NULL) {
      continue;
    }
    memcpy(pipeBuf, pipe.buf, pipe.nb_aligned_bytes);
    pipe.tx_pipe->SendAndFree(pipe.nb_aligned_bytes);
    // update the stats
    stats[pipe.tx_pipe->id()].nb_bytes += pipe.nb_raw_bytes;
    stats[pipe.tx_pipe->id()].nb_pkts += pipe.nb_pkts;
  }
}

int Client::startClient(const ClientConfig& config) {
  std::cout << "Running in client mode with:\n"
            << "  Connections: " << config.numFlowsPerCore * config.numCores
            << "\n"
            << "  Cores: " << config.numCores << "\n"
            << "  PCAP path: " << config.pcapPath << "\n";
  if (config.count) {
    std::cout << "  Count: " << *config.count << "\n";
  }
  std::unique_ptr<Device> dev = Device::Create(INTEL_FPGA_PCIE_BDF);
  if (!dev) {
    std::cerr << "Problem creating device" << std::endl;
    exit(2);
  }

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t* pcap = pcap_open_offline(config.pcapPath.c_str(), errbuf);
  if (pcap == NULL) {
    std::cerr << "Error loading pcap file (" << errbuf << ")" << std::endl;
    return 2;
  }

  struct PcapHandler context(dev, pcap, this);
  std::vector<struct EnsoTxPipe>& txPipes = context.txPipes;

  if (pcap_loop(context.pcap, 0, Client::pcapPktHandler, (u_char*)&context) <
      0) {
    std::cerr << "Error while reading pcap (" << pcap_geterr(context.pcap)
              << ")" << std::endl;
    return -2;
  }

  if (txPipes.size() != (config.numCores * config.numFlowsPerCore)) {
    std::cerr << "PCAP file does not have the same number of flows"
              << std::endl;
    std::cerr << config.numFlowsPerCore * config.numCores << " expected. "
              << txPipes.size() << " found." << std::endl;
    return -2;
  }

  // stats to record the metrics
  std::vector<std::thread> threads;
  std::vector<enso::tx_stats_t> thread_stats(config.numCores *
                                             config.numFlowsPerCore);

  for (uint16_t flowId = 0; flowId < config.numFlowsPerCore; flowId++) {
    threads.emplace_back(&Client::runTx, this, std::ref(thread_stats),
                         std::ref(txPipes[flowId]));
    if (enso::set_core_id(threads.back(), flowId)) {
      std::cerr << "Error setting CPU affinity" << std::endl;
      return 6;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  show_tx_flow_stats(thread_stats, config.numCores * config.numFlowsPerCore,
                     &ProgramConfig::keepRunning);

  for (auto& thread : threads) {
    thread.join();
  }

  return 0;
}
