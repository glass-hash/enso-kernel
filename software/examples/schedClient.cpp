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
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>

#include "example_helpers.h"
#include "schedPerf.h"

#define INTEL_FPGA_PCIE_BDF "65:00.0"
#define MIN_PACKET_SIZE 64
#define NSEC_PER_SEC 1000000000ULL

Client::Client(const ClientConfig& config) { startClient(config); }

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
  // Set the timestamp to zero to calculate inter-arrival packet rates on the
  // receiver
  enso::set_pkt_rtt(pktBytes, 0);
  uint32_t numFlits = (len - 1) / MIN_PACKET_SIZE + 1;
  TxPipe* pipe = context->devs[devId]->AllocateTxPipe();
  if (!pipe) {
    std::cerr << "Problem creating TX pipe" << std::endl;
    exit(2);
  }
  uint8_t* buf;
  // Instead of allocating batch size worth of data and copying it on to the
  // TxPipe we keep the source buffer small and copy it over and over again for
  // better cache performance
  uint32_t pktAlignedSize = numFlits * MIN_PACKET_SIZE;
  if (posix_memalign((void**)&buf, 64, pktAlignedSize * sizeof(uint8_t)) != 0) {
    std::cerr << "Posix memalign failed" << std::endl;
    exit(2);
  }
  uint32_t numPktsInBatch = context->batchSize / pktAlignedSize;
  struct EnsoTxPipe etp(pipe, buf);
  memcpy(buf, pktBytes, len);
  etp.bufSize = pktAlignedSize;
  etp.numAlignedBytes = pktAlignedSize * numPktsInBatch;
  etp.numRawBytes = len * numPktsInBatch;
  etp.numPkts = numPktsInBatch;
  context->txPipes.push_back(etp);
}

static inline uint64_t get_ns(void) {
  struct timespec ts;
  // Get current time using CLOCK_MONOTONIC
  clock_gettime(CLOCK_MONOTONIC, &ts);
  // Convert to nanoseconds
  uint64_t ns = (uint64_t)ts.tv_sec * NSEC_PER_SEC + (uint64_t)ts.tv_nsec;
  return ns;
}

void Client::runTx(std::vector<enso::tx_stats_t>& stats,
                   std::vector<struct EnsoTxPipe>& pipes, uint16_t coreId,
                   uint16_t flowsPerCore, uint16_t rate) {
  std::cout << "Running on core " << coreId << " with pid = " << getpid()
            << std::endl;
  uint16_t startInd = coreId * flowsPerCore;
  uint16_t endInd = startInd + flowsPerCore;
  uint64_t rate_bytes = ((uint64_t)rate * 1000000000) / 8;
  int64_t cur_rate_bytes = rate_bytes;
  uint64_t time_last = get_ns();
  // Assuming that all pipes send the same batch size
  uint64_t batch_size_on_wire = pipes[0].numRawBytes + pipes[0].numPkts * 24;
  while (ProgramConfig::keepRunning) {
    for (uint16_t i = startInd; i < endInd; i++) {
      uint64_t time_now = get_ns();
      if (time_now > (time_last + NSEC_PER_SEC)) {
        cur_rate_bytes = rate_bytes;
        time_last = time_now;
      }
      cur_rate_bytes -= batch_size_on_wire;
      if (cur_rate_bytes > 0) {
        // send the packets
        uint8_t* pipeBuf =
            pipes[i].txPipe->AllocateBuf(pipes[i].numAlignedBytes);
        enso::memcpy_wrap_around(pipeBuf, pipes[i].buf,
                                 pipes[i].numAlignedBytes, pipes[i].bufSize);
        pipes[i].txPipe->SendAndFree(pipes[i].numAlignedBytes);
        // update the stats
        stats[pipes[i].txPipe->id()].nb_bytes += pipes[i].numRawBytes;
        stats[pipes[i].txPipe->id()].nb_pkts += pipes[i].numPkts;
      }
    }
  }
}

int Client::startClient(const ClientConfig& config) {
  std::cout << "Running in client mode with:\n"
            << "  Connections: " << config.numFlowsPerCore * config.numCores
            << "\n"
            << "  Cores: " << config.numCores << "\n"
            << "  PCAP path: " << config.pcapPath << "\n"
            << "  Timeout: " << config.timeout << "\n"
            << "  Rate: " << config.rate << "\n";
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
                         std::ref(txPipes), coreId, config.numFlowsPerCore,
                         config.rate);
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
