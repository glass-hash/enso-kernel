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
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

#include "../enso_api.hpp"

namespace enso_api {

EnsoDev *EnsoDev::Create() noexcept {
  EnsoDev *dev = new (std::nothrow) EnsoDev();
  if (!dev) {
    return NULL;
  }

  if (dev->Init()) {
    delete dev;
    return NULL;
  }

  return dev;
}

/******************************************************************************
 * Enso device class methods
 *****************************************************************************/
int EnsoDev::Init() noexcept {
  ssize_t fd;

  fd = open("/dev/enso", O_RDWR | O_CLOEXEC);

  if (fd == -1) {
    std::cerr << "could not open character device; ensure that Enso "
                 "kernel driver has been loaded"
              << std::endl;
    return -1;
  }
  m_dev_handle = fd;
  return 0;
}

EnsoDev::~EnsoDev(void) { close(m_dev_handle); }

void EnsoDev::test(void) { ioctl(m_dev_handle, ENSO_IOCTL_TEST); }

int EnsoDev::get_nb_fallback_queues() {
  int result;
  unsigned int nb_fallback_queues;
  result = ioctl(m_dev_handle, ENSO_IOCTL_GET_NB_FALLBACK_QUEUES,
                 &nb_fallback_queues);

  if (result != 0) {
    return -1;
  }

  return nb_fallback_queues;
}

int EnsoDev::set_rr_status(bool enable_rr) {
  return ioctl(m_dev_handle, ENSO_IOCTL_SET_RR_STATUS, enable_rr);
}

int EnsoDev::get_rr_status() {
  int result;
  bool rr_status;
  result = ioctl(m_dev_handle, ENSO_IOCTL_GET_RR_STATUS, &rr_status);

  if (result != 0) {
    return -1;
  }

  return rr_status;
}

int EnsoDev::alloc_notif_buffer_id() {
  int result;
  unsigned int buf_id;
  result = ioctl(m_dev_handle, ENSO_IOCTL_ALLOC_NOTIF_BUFFER_ID, &buf_id);

  if (result != 0) {
    return -1;
  }

  return buf_id;
}

int EnsoDev::free_notif_buffer_id(int id) {
  return ioctl(m_dev_handle, ENSO_IOCTL_FREE_NOTIF_BUFFER_ID, id);
}

int EnsoDev::alloc_rx_pipe_id(bool fallback) {
  int result;
  unsigned int uarg = fallback;
  result = ioctl(m_dev_handle, ENSO_IOCTL_ALLOC_RX_PIPE_ID, &uarg);

  if (result != 0) {
    return -1;
  }

  return uarg;
}

int EnsoDev::free_rx_pipe_id(int id) {
  return ioctl(m_dev_handle, ENSO_IOCTL_FREE_RX_PIPE_ID, id);
}

int EnsoDev::alloc_notif_buffer(int id) {
  int result;
  result = ioctl(m_dev_handle, ENSO_IOCTL_ALLOC_NOTIF_BUFFER, id);

  if (result != 0) {
    return -1;
  }

  return result;
}

int EnsoDev::send_tx_pipe(uint64_t phys_addr, uint32_t len, uint32_t buf_id) {
  int result;
  struct enso_send_tx_pipe_params stpp;
  stpp.phys_addr = phys_addr;
  stpp.len = len;
  stpp.id = buf_id;
  result = ioctl(m_dev_handle, ENSO_IOCTL_SEND_TX_PIPE, &stpp);

  if (result != 0) {
    std::cout << "failure at send_tx ioctl " << result << std::endl;
    return -1;
  }

  return result;
}

int EnsoDev::get_unreported_completions() {
  int result;
  unsigned int completions;
  result =
      ioctl(m_dev_handle, ENSO_IOCTL_GET_UNREPORTED_COMPLETIONS, &completions);

  if (result != 0) {
    std::cout << "unreported completions failed" << std::endl;
    return -1;
  }

  return completions;
}

int EnsoDev::send_config(struct TxNotification *txNotification) {
  int result;
  result = ioctl(m_dev_handle, ENSO_IOCTL_SEND_CONFIG, txNotification);

  if (result != 0) {
    std::cout << "failure at send_confif ioctl " << result << std::endl;
    return -1;
  }

  return result;
}

int EnsoDev::alloc_rx_pipe(int pipe_id, uint64_t buf_phys_addr) {
  int result;
  struct enso_pipe_init_params param;
  param.phys_addr = buf_phys_addr;
  param.id = pipe_id;
  result = ioctl(m_dev_handle, ENSO_IOCTL_ALLOC_RX_PIPE, &param);

  if (result != 0) {
    return -1;
  }

  return result;
}

int EnsoDev::free_rx_pipe(int pipe_id) {
  int result;
  result = ioctl(m_dev_handle, ENSO_IOCTL_FREE_RX_PIPE, pipe_id);

  if (result != 0) {
    return -1;
  }

  return 0;
}

int EnsoDev::consume_rx_pipe(int &pipe_id, uint32_t &krx_tail) {
  int result;
  struct enso_consume_rx_params param;
  param.id = pipe_id;
  param.new_rx_tail = 0;
  result = ioctl(m_dev_handle, ENSO_IOCTL_CONSUME_RX, &param);
  krx_tail = param.new_rx_tail;
  pipe_id = param.id;
  return result;
}

int EnsoDev::full_adv_pipe(int pipe_id) {
  int result;
  result = ioctl(m_dev_handle, ENSO_IOCTL_FULL_ADV_PIPE, &pipe_id);
  return result;
}

int EnsoDev::get_next_batch(int notif_id, int &pipe_id, uint32_t &krx_tail) {
  int result;
  struct enso_get_next_batch_params param;
  param.notif_id = notif_id;
  param.new_rx_tail = 0;
  param.pipe_id = -1;
  result = ioctl(m_dev_handle, ENSO_IOCTL_GET_NEXT_BATCH, &param);
  if (result < 0) {
    return -1;
  }
  krx_tail = param.new_rx_tail;
  pipe_id = param.pipe_id;
  return result;
}

int EnsoDev::advance_pipe(int pipe_id, size_t len) {
  int result;
  struct enso_advance_pipe_params param;
  param.id = pipe_id;
  param.len = len;
  result = ioctl(m_dev_handle, ENSO_IOCTL_ADVANCE_PIPE, &param);
  return result;
}

int EnsoDev::next_rx_pipe_to_recv() {
  int result;
  result = ioctl(m_dev_handle, ENSO_IOCTL_NEXT_RX_PIPE_RCV, 0);
  return result;
}

int EnsoDev::prefetch_pipe(int pipe_id) {
  int result;
  result = ioctl(m_dev_handle, ENSO_IOCTL_PREFETCH_PIPE, &pipe_id);
  return result;
}

}  // namespace enso_api
