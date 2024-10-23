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
#include <pcap/pcap.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include "example_helpers.h"
#include "schedPerf.h"

#define INTEL_FPGA_PCIE_BDF "65:00.0"
#define DEFAULT_STATS_DELAY 1000
#define ONE_MILLION 1e6
#define ONE_THOUSAND 1e3
#define FPGA_PACKET_OVERHEAD 20
#define MIN_PACKET_SIZE 64
#define DEFAULT_NB_QUEUES 4
#define MAX_FLOWS 4096

Server::Server(const ServerConfig& serverConfig) {
  numFlows = serverConfig.numFlows;
}

void Server::runRx(enso::stats_t* stats, std::vector<uint64_t>& pktsPerFlow) {
  // create the device and initialize the RxPipe
  std::unique_ptr<Device> dev = Device::Create(INTEL_FPGA_PCIE_BDF);
  if (!dev) {
    std::cerr << "Problem creating device" << std::endl;
    exit(2);
  }

  // Enable round robin to ensure Rx does not drop packets
  // TODO(kshitij): Remove this once support is added in HW
  // for one Tx pipe per flow
  dev->EnableRoundRobin();

  std::vector<RxPipe*> rxPipes;
  for (uint32_t i = 0; i < DEFAULT_NB_QUEUES; ++i) {
    RxPipe* rxPipe = dev->AllocateRxPipe(true);
    if (!rxPipe) {
      std::cerr << "Problem creating RX pipe" << std::endl;
      exit(3);
    }
    rxPipes.push_back(rxPipe);
  }

  while (ProgramConfig::keepRunning) {
    uint64_t nb_pkts = 0;

    RxPipe* rxPipe = dev->NextRxPipeToRecv();
    if (unlikely(rxPipe == nullptr)) {
      continue;
    }

    auto batch = rxPipe->PeekPktsFromTail();
    for (auto pkt : batch) {
      (void)pkt;
      uint16_t pktDst = enso::get_pkt_dst_lsb(pkt);
      pktsPerFlow[pktDst]++;
      nb_pkts++;
    }
    uint32_t batch_length = batch.processed_bytes();
    rxPipe->ConfirmBytes(batch_length);

    stats->recv_bytes += batch_length;
    stats->nb_batches++;
    stats->nb_pkts += nb_pkts;

    rxPipe->Clear();
  }
}

void Server::startServer() {
  std::cout << "Running in server mode with " << numFlows << " connections"
            << std::endl;

  std::vector<enso::stats_t> threadStats(1);
  std::vector<uint64_t> pktsPerFlow(MAX_FLOWS);

  std::thread rxThread(&Server::runRx, this, &threadStats[0],
                       std::ref(pktsPerFlow));
  enso::set_core_id(rxThread, 0);
  enso::show_rx_flow_stats(pktsPerFlow, &threadStats[0], numFlows,
                           &ProgramConfig::keepRunning);

  rxThread.join();

  uint64_t totalPkts = 0;
  for (int i = 0; i < numFlows; i++) {
    std::cout << "Flow " << i << ": " << pktsPerFlow[i] << std::endl;
    totalPkts += pktsPerFlow[i];
  }
  std::cout << "Total packets received: " << totalPkts << std::endl;
}
