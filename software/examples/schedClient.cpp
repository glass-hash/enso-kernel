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
#include <fstream>
#include <iostream>
#include <memory>

#include "example_helpers.h"
#include "schedPerf.h"

#define INTEL_FPGA_PCIE_BDF "65:00.0"
#define MIN_PACKET_SIZE 64

Client::Client(const ClientConfig& config) { startClient(config); }

void Client::fillPipeWithPackets(uint8_t* pipeBuf, uint32_t& alignedBytes,
                                 uint32_t& rawBytes, uint32_t& pkts,
                                 uint32_t batchSize) {
  uint32_t initBufLength = alignedBytes;
  uint32_t initGoodBytes = rawBytes;
  uint32_t initNumPkts = pkts;
  while ((alignedBytes + initBufLength) <= batchSize) {
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

  const struct ether_header* l2Header = (struct ether_header*)pktBytes;
  if (l2Header->ether_type != htons(ETHERTYPE_IP)) {
    std::cerr << "Non-IPv4 packets are not supported" << std::endl;
    exit(1);
  }

  uint16_t devId = context->txPipes.size() / context->numFlowsPerCore;
  uint32_t len = enso::get_pkt_len(pktBytes);
  uint32_t numFlits = (len - 1) / MIN_PACKET_SIZE + 1;
  TxPipe* pipe = context->devs[devId]->AllocateTxPipe();
  if (!pipe) {
    std::cerr << "Problem creating TX pipe" << std::endl;
    exit(2);
  }
  uint8_t* buf;
  if (posix_memalign((void**)&buf, 64, context->batchSize * sizeof(uint8_t)) !=
      0) {
    std::cerr << "Posix memalign failed" << std::endl;
    exit(2);
  }
  struct EnsoTxPipe etp(pipe, buf);
  memcpy(buf, pktBytes, len);
  etp.numAlignedBytes = numFlits * MIN_PACKET_SIZE;
  etp.numRawBytes = len;
  etp.numPkts = 1;
  context->client->fillPipeWithPackets(buf, etp.numAlignedBytes,
                                       etp.numRawBytes, etp.numPkts,
                                       context->batchSize);
  context->txPipes.push_back(etp);
}

inline uint64_t get_ns_chrono(void) {
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             now.time_since_epoch())
      .count();
}

void Client::runTx(std::vector<enso::tx_stats_t>& stats,
                   std::vector<struct EnsoTxPipe>& pipes, uint16_t coreId,
                   uint16_t flowsPerCore) {
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "Running on core " << sched_getcpu() << std::endl;
  uint16_t startInd = coreId * flowsPerCore;
  uint16_t endInd = startInd + flowsPerCore;
  /*int64_t now = 0;
  int64_t tokens_lc = 0;
  int64_t last_checkpoint = get_ns_chrono();

  uint64_t rate = 12500000000;
  uint64_t max_size = (int64_t)((int64_t)2*1024*1024*1024) / 8;
  int64_t buffer = (int64_t) (max_size * 1000000000) / rate;
  int64_t tokens = buffer;
  int64_t batch_size = 86016;*/

  while (ProgramConfig::keepRunning) {
    for (uint16_t i = startInd; i < endInd; i++) {
      // send the packets
      uint8_t* pipeBuf = pipes[i].txPipe->AllocateBuf(pipes[i].numAlignedBytes);
      memcpy(pipeBuf, pipes[i].buf, pipes[i].numAlignedBytes);
      // now = get_ns_chrono();
      // tokens_lc = std::min(now - last_checkpoint, buffer);
      // tokens_lc += tokens;
      // if (tokens_lc > buffer)
      //     tokens_lc = buffer;
      // tokens_lc -= (int64_t)(batch_size * 1000000000) / rate;
      // if(tokens_lc >= 0) {
      pipes[i].txPipe->SendAndFree(pipes[i].numAlignedBytes, pipes[i].numPkts);
      // update the stats
      stats[pipes[i].txPipe->id()].nb_bytes += pipes[i].numRawBytes;
      stats[pipes[i].txPipe->id()].nb_pkts += pipes[i].numPkts;
      // last_checkpoint = now;
      // tokens = tokens_lc;
      // }
    }
  }
}

int Client::startClient(const ClientConfig& config) {
  std::cout << "Running in client mode with:\n"
            << "  Connections: " << config.numFlowsPerCore * config.numCores
            << "\n"
            << "  Cores: " << config.numCores << "\n"
            << "  PCAP path: " << config.pcapPath << "\n"
            << "  Timeout: " << config.timeout << "\n";
  std::vector<std::unique_ptr<Device>> devs(config.numCores);
  for (uint16_t i = 0; i < config.numCores; i++) {
    devs[i] = Device::Create(INTEL_FPGA_PCIE_BDF);
    if (!devs[i]) {
      std::cerr << "Problem creating device" << std::endl;
      exit(2);
    }
  }

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t* pcap = pcap_open_offline(config.pcapPath.c_str(), errbuf);
  if (pcap == NULL) {
    std::cerr << "Error loading pcap file (" << errbuf << ")" << std::endl;
    return 2;
  }

  struct PcapHandler context(devs, pcap, this, config.numFlowsPerCore,
                             config.batchSize);
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

  std::vector<std::thread> threads;
  uint16_t totalFlows = config.numCores * config.numFlowsPerCore;
  std::vector<enso::tx_stats_t> flowStats(totalFlows);

  for (uint16_t coreId = 0; coreId < config.numCores; coreId++) {
    threads.emplace_back(&Client::runTx, this, std::ref(flowStats),
                         std::ref(txPipes), coreId, config.numFlowsPerCore);
    if (enso::set_core_id(threads.back(), coreId)) {
      std::cerr << "Error setting CPU affinity" << std::endl;
      return 6;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  show_tx_flow_stats(flowStats, config.numCores * config.numFlowsPerCore,
                     &ProgramConfig::keepRunning, config.timeout);

  for (auto& thread : threads) {
    thread.join();
  }

  // calculate final stats and put in a file
  uint64_t totalBytes = 0;
  uint64_t totalPkts = 0;
  for (uint32_t i = 0; i < totalFlows; i++) {
    totalBytes += flowStats[i].nb_bytes;
    totalPkts += flowStats[i].nb_pkts;
  }
  std::ofstream statsFile("schedTxStats.csv");
  statsFile << totalBytes << "," << totalPkts << std::endl;
  statsFile.close();

  // free all buffers
  for (auto pipe : txPipes) {
    free(pipe.buf);
  }
  return 0;
}
