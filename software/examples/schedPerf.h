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
#ifndef SOFTWARE_EXAMPLES_SCHEDPERF_H_
#define SOFTWARE_EXAMPLES_SCHEDPERF_H_
#include <enso/pipe.h>
#include <pcap/pcap.h>

#include <thread>
#include <vector>

using enso::Device;
using enso::RxPipe;
using enso::TxPipe;

class Client;

struct ClientConfig {
  uint16_t numFlowsPerCore;
  uint16_t numCores;
  std::string pcapPath;
  uint16_t timeout;
  uint32_t batchSize;
};

struct ServerConfig {
  uint16_t numFlows;
};

/**
 * @brief Structure to store an Enso TxPipe object and attributes related
 * to it.
 */
struct EnsoTxPipe {
  explicit EnsoTxPipe(TxPipe* pipe, uint8_t* _buf)
      : txPipe(pipe),
        numAlignedBytes(0),
        numRawBytes(0),
        numPkts(0),
        buf(_buf) {}
  // Enso TxPipe
  TxPipe* txPipe;
  // Number of cache aligned bytes in the pipe
  uint32_t numAlignedBytes;
  // Number of raw bytes in the pipe
  uint32_t numRawBytes;
  // Number of packets in the pipe
  uint32_t numPkts;
  uint8_t* buf;
};

// structure for libpcap
struct PcapHandler {
  PcapHandler(std::vector<std::unique_ptr<Device>>& devs_, pcap_t* pcap_,
              Client* c_, uint16_t flowsPerCore, uint32_t _batchSize)
      : devs(devs_),
        pcap(pcap_),
        client(c_),
        numFlowsPerCore(flowsPerCore),
        batchSize(_batchSize) {}
  // Pointer to Enso device
  std::vector<std::unique_ptr<Device>>& devs;
  // Pipes to store the packets from the PCAP file
  std::vector<struct EnsoTxPipe> txPipes;
  // libpcap object associated with the opened PCAP file
  pcap_t* pcap;
  Client* client;
  uint16_t numFlowsPerCore;
  uint32_t batchSize;
};

class ProgramConfig {
 public:
  enum class Mode { Client, Server, Unknown };

  static bool parseArgs(int argc, char* argv[], ProgramConfig& config);

  Mode getMode() const { return mode; }
  const ClientConfig& getClientConfig() const { return clientConfig; }
  const ServerConfig& getServerConfig() const { return serverConfig; }

  static volatile bool keepRunning;
  static void sigintHandler(int signal);

 private:
  Mode mode = Mode::Unknown;
  ClientConfig clientConfig;
  ServerConfig serverConfig;
};

class Server {
 public:
  void startServer(const ServerConfig& serverConfig);
  explicit Server(const ServerConfig& serverConfig);

 private:
  void runRx(enso::stats_t* stats, std::vector<uint64_t>& pktsPerFlow);
};

class Client {
 public:
  int startClient(const ClientConfig& config);
  explicit Client(const ClientConfig& clientConfig);

 private:
  void fillPipeWithPackets(uint8_t* pipeBuf, uint32_t& alignedBytes,
                           uint32_t& rawBytes, uint32_t& pkts,
                           uint32_t batchSize);
  static void pcapPktHandler(u_char* user, const struct pcap_pkthdr* pktHeader,
                             const u_char* pktBytes);
  void runTx(std::vector<enso::tx_stats_t>& stats,
             std::vector<struct EnsoTxPipe>& pipes, uint16_t coreId,
             uint16_t flowsPerCore);
};
#endif  // SOFTWARE_EXAMPLES_SCHEDPERF_H_
