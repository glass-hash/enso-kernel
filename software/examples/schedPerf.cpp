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
#include "schedPerf.h"

#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

volatile bool ProgramConfig::keepRunning = true;

void ProgramConfig::sigintHandler(int signal __attribute__((unused))) {
  keepRunning = false;
}

// TODO(kshitij): Fix argument parsing with proper required and optional
// arguments
bool ProgramConfig::parseArgs(int argc, char* argv[], ProgramConfig& config) {
  int opt;
  // Reset getopt
  optind = 1;
  config.mode = Mode::Unknown;
  uint16_t timeoutVal = 0;
  uint32_t batchSize = 0;
  uint16_t rate = 0;

  while ((opt = getopt(argc, argv, "s:c:t:b:r:")) != -1) {
    switch (opt) {
      case 's':
        if (config.mode != Mode::Unknown) {
          std::cerr << "Error: Cannot specify both server and client mode"
                    << std::endl;
          return false;
        }
        config.mode = Mode::Server;
        config.serverConfig.numFlows = atoi(optarg);
        if (config.serverConfig.numFlows <= 0) {
          std::cerr << "Error: Number of connections must be positive"
                    << std::endl;
          return false;
        }
        break;

      case 'c':
        if (config.mode != Mode::Unknown) {
          std::cerr << "Error: Cannot specify both server and client mode"
                    << std::endl;
          return false;
        }
        config.mode = Mode::Client;
        config.clientConfig.numFlowsPerCore = atoi(optarg);
        if (config.clientConfig.numFlowsPerCore <= 0) {
          std::cerr << "Error: Number of connections must be positive"
                    << std::endl;
          return false;
        }
        break;

      case 't':
        timeoutVal = atoi(optarg);
        if (timeoutVal <= 0) {
          std::cerr << "Invalid timeout value" << std::endl;
          return false;
        }
        break;

      case 'b':
        batchSize = atoi(optarg);
        if ((batchSize == 0) || (batchSize > 131072)) {
          std::cerr << "Invalid batch size value" << std::endl;
          return false;
        }
        break;

      case 'r':
        rate = atoi(optarg);
        if ((rate == 0) || (rate > 100)) {
          std::cerr << "Invalid rate value" << std::endl;
          return false;
        }
        break;

      case '?':
        std::cerr << "Error: Invalid option" << std::endl;
        return false;
    }
  }

  std::cout << "Opt index = " << optind << std::endl;
  // Process remaining arguments based on mode
  if (config.mode == Mode::Client) {
    // Need at least 2 more arguments (cores and pcap path)
    config.clientConfig.timeout = timeoutVal;
    if (optind + 1 >= argc) {
      std::cerr << "Error: Client mode requires <num-cores> and <pcap-path>"
                << std::endl;
      return false;
    }

    config.clientConfig.numCores = atoi(argv[optind]);
    if (config.clientConfig.numCores <= 0) {
      std::cerr << "Error: Number of cores must be positive" << std::endl;
      return false;
    }

    config.clientConfig.pcapPath = argv[optind + 1];
    config.clientConfig.batchSize = (batchSize == 0) ? 131072 : batchSize;
    config.clientConfig.rate = (rate == 0) ? 100 : rate;
    optind += 2;

  } else if (config.mode == Mode::Server) {
    // Server mode shouldn't have any additional arguments
    if (batchSize != 0) {
      std::cerr << "Error: Server mode cannot have batch size" << std::endl;
      return false;
    }
    if (optind < argc) {
      std::cerr << "Error: Unexpected additional arguments for server mode"
                << std::endl;
      return false;
    } else if (argc < 3) {
      std::cerr << "Error: Not enough arguments for server mode" << std::endl;
      return false;
    }
  } else {
    std::cerr << "Error: Must specify either server (-s) or client (-c) mode"
              << std::endl;
    return false;
  }

  return true;
}

int main(int argc, char* argv[]) {
  ProgramConfig config;
  if (!ProgramConfig::parseArgs(argc, argv, config)) {
    std::cerr << "Usage:\n"
              << "  Server mode: " << argv[0] << " -s <num-flows>\n"
              << "  Client mode: " << argv[0]
              << " -c <num-flows-per-core> <num-cores> <pcap-path> -t "
                 "<timeout> -b <batch-size>"
              << std::endl;
    return 1;
  }
  // init signal handler
  signal(SIGINT, ProgramConfig::sigintHandler);

  if (config.getMode() == ProgramConfig::Mode::Server) {
    std::unique_ptr<Server> s =
        std::make_unique<Server>(config.getServerConfig());
  } else {
    std::unique_ptr<Client> c =
        std::make_unique<Client>(config.getClientConfig());
  }
  return 0;
}
