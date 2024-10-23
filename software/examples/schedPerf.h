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

#include <thread>
#include <vector>

using enso::Device;
using enso::RxPipe;
using enso::TxPipe;

struct ClientConfig {
  int numFlows;
  int numCores;
  std::string pcapPath;
  std::optional<int> count;
};

struct ServerConfig {
  int numFlows;
};

class ProgramConfig {
 public:
  enum class Mode { Client, Server, Unknown };

  static bool parseArgs(int argc, char* argv[], ProgramConfig& config);

  Mode getMode() const { return mode; }
  const ClientConfig& getClientConfig() const { return clientConfig; }
  const ServerConfig& getServerConfig() const { return serverConfig; }

 private:
  Mode mode = Mode::Unknown;
  ClientConfig clientConfig;
  ServerConfig serverConfig;
};

class Server {
 public:
  void startServer();
  explicit Server(const ServerConfig& serverConfig);

 private:
  int numFlows;
  void runRx(enso::stats_t* stats, std::vector<uint64_t>& pkts_per_flow);
};
#endif  // SOFTWARE_EXAMPLES_SCHEDPERF_H_
