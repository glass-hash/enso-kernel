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

#ifndef SOFTWARE_SRC_BACKENDS_ENSO_ENSO_API_HPP_
#define SOFTWARE_SRC_BACKENDS_ENSO_ENSO_API_HPP_

#include <sys/types.h>

#include <cstddef>
#include <cstdint>

#ifdef _WIN32
#endif

#ifdef __linux__
#include "linux/enso_api_linux.hpp"
#endif  // __linux__

namespace enso_api {

/**
 * Enso device handle class.
 *
 * This class provides a handle to access an Enso device from
 * the user-space without worrying about kernel interface.
 *
 * @see enso_backend.h for function documentation.
 *
 */
class EnsoDev {
 public:
  static EnsoDev *Create() noexcept;

  ~EnsoDev(void);

  void test();
  int get_nb_fallback_queues();
  int set_rr_status(bool enable_rr);
  int get_rr_status();
  int alloc_notif_buffer_id();
  int free_notif_buffer_id(int id);
  int alloc_rx_pipe_id(bool fallback = false);
  int free_rx_pipe_id(int id);
  int alloc_notif_buffer(int id);
  int send_tx_pipe(uint64_t phys_addr, uint32_t len, uint32_t buf_id);
  int get_unreported_completions();
  int send_config(struct TxNotification *txNotification);
  int alloc_rx_pipe(int pipe_id, uint64_t buf_phys_addr);
  int free_rx_pipe(int pipe_id);
  int consume_rx_pipe(int &pipe_id, uint32_t &krx_tail);
  int full_adv_pipe(int pipe_id);
  int get_next_batch(int notif_id, int &pipe_id, uint32_t &krx_tail);
  int advance_pipe(int pipe_id, size_t len);
  int next_rx_pipe_to_recv();
  int prefetch_pipe(int pipe_id);
  int alloc_tx_pipe_id();
  int free_tx_pipe_id(int pipe_id);

 private:
  /**
   * Class should be instantiated via the Create() factory method.
   */
  EnsoDev() noexcept = default;

  int Init() noexcept;

  ssize_t m_dev_handle;
};

}  // namespace enso_api

#endif  // SOFTWARE_SRC_BACKENDS_ENSO_ENSO_API_HPP_
