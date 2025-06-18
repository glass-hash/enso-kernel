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

#include <arpa/inet.h>
#include <enso/helpers.h>
#include <net/ethernet.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
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

constexpr char kDstMac[] = "aa:aa:aa:aa:aa:aa";
constexpr char kSrcMac[] = "bb:bb:bb:bb:bb:bb";

Client::Client(const ClientConfig& config) { startClient(config); }

void Client::initializeTxPipes(std::vector<struct EnsoTxPipe>& txPipes,
                               uint16_t numFlows, uint16_t pktSize,
                               uint32_t batchSize,
                               const std::unique_ptr<Device>& dev,
                               uint16_t coreID) {
  // We want to have one flow per tx pipe. During testing, the program will
  // be used with the same number of flows but different core IDs.
  uint16_t coreFlowIndexStart = coreID * numFlows;
  for (uint16_t flowInd = 0; flowInd < numFlows; flowInd++) {
    uint8_t* pktBuf;
    if (posix_memalign((void**)&pktBuf, 64, pktSize * sizeof(uint8_t)) != 0) {
      std::cerr << "Posix memalign failed" << std::endl;
      exit(2);
    }
    struct ether_addr dstMac = *ether_aton(kDstMac);
    struct ether_addr srcMac = *ether_aton(kSrcMac);
    uint16_t pktSizeNoCrc = pktSize - 4;
    struct ether_header* ethHeader = (struct ether_header*)pktBuf;
    struct iphdr* ipHeader = (struct iphdr*)(ethHeader + 1);
    struct udphdr* udpHeader = (struct udphdr*)(ipHeader + 1);
    uint32_t baseIP = 0xC0A80000;  // 192.168.0.0
    // Ethernet header
    memcpy(&ethHeader->ether_shost, &srcMac, ETHER_ADDR_LEN);
    memcpy(&ethHeader->ether_dhost, &dstMac, ETHER_ADDR_LEN);
    ethHeader->ether_type = htons(ETHERTYPE_IP);
    // IP header
    ipHeader->version = 4;
    ipHeader->ihl = 5;  // 20 bytes (5 * 4)
    ipHeader->tos = 0;
    ipHeader->tot_len = htons(pktSizeNoCrc - sizeof(struct ether_header));
    ipHeader->id = 0;
    ipHeader->frag_off = 0;
    ipHeader->ttl = 64;
    ipHeader->protocol = IPPROTO_UDP;
    ipHeader->saddr = htonl(baseIP);  // srcIP always remains the same;
    // why create new flows based on dst ip though, why not dst port?
    ipHeader->daddr = htonl(baseIP + coreFlowIndexStart + flowInd);
    ipHeader->check = 0;
    // UDP header
    udpHeader->source = htons(8080);
    udpHeader->dest = htons(80);
    udpHeader->len = htons(
        pktSizeNoCrc - (sizeof(struct ether_header) + sizeof(struct iphdr)));
    udpHeader->check = 0;
    // Fill payload
    uint8_t* payload = (uint8_t*)(((char*)udpHeader) + sizeof(struct udphdr));
    uint16_t payloadSize = pktSizeNoCrc - sizeof(struct ether_header) -
                           sizeof(struct iphdr) - sizeof(struct udphdr);
    for (uint16_t i = 0; i < payloadSize; i++) {
      payload[i] = 0xff;
    }
    uint32_t numFlits = (pktSizeNoCrc - 1) / MIN_PACKET_SIZE + 1;
    uint32_t pktAlignedSize = numFlits * MIN_PACKET_SIZE;
    uint32_t numPktsInBatch = batchSize / pktAlignedSize;

    TxPipe* pipe = dev->AllocateTxPipe();
    if (!pipe) {
      std::cerr << "Problem creating TX pipe" << std::endl;
      exit(2);
    }
    struct EnsoTxPipe etp(pipe, pktBuf);
    etp.bufSize = pktAlignedSize;
    etp.numAlignedBytes = pktAlignedSize * numPktsInBatch;
    etp.numRawBytes = pktSizeNoCrc * numPktsInBatch;
    etp.numPkts = numPktsInBatch;
    txPipes.push_back(etp);
  }
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
  uint16_t startInd = 0;
  uint16_t endInd = flowsPerCore;
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
        stats[i].nb_bytes += pipes[i].numRawBytes;
        stats[i].nb_pkts += pipes[i].numPkts;
      }
    }
  }
}

int Client::startClient(const ClientConfig& config) {
  std::cout << "Running in client mode with:\n"
            << "  Connections: " << config.numFlows << "\n"
            << "  Core ID: " << config.coreID << "\n"
            << "  Timeout: " << config.timeout << "\n"
            << "  Rate: " << config.rate << "\n"
            << "  Packet size: " << config.pktSize << "\n";

  std::unique_ptr<Device> dev = Device::Create(INTEL_FPGA_PCIE_BDF);
  if (!dev) {
    std::cerr << "Problem creating device" << std::endl;
    exit(2);
  }

  std::vector<struct EnsoTxPipe> txPipes;
  initializeTxPipes(txPipes, config.numFlows, config.pktSize, config.batchSize,
                    dev, config.coreID);
  std::vector<std::thread> threads;
  std::vector<enso::tx_stats_t> flowStats(config.numFlows);

  threads.emplace_back(&Client::runTx, this, std::ref(flowStats),
                       std::ref(txPipes), config.coreID, config.numFlows,
                       config.rate);
  if (enso::set_core_id(threads.back(), config.coreID)) {
    std::cerr << "Error setting CPU affinity" << std::endl;
    return 6;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  show_tx_flow_stats(flowStats, config.numFlows, &ProgramConfig::keepRunning,
                     config.timeout);

  for (auto& thread : threads) {
    thread.join();
  }

  // calculate final stats and put in a file
  uint64_t totalBytes = 0;
  uint64_t totalPkts = 0;
  for (uint32_t i = 0; i < config.numFlows; i++) {
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
