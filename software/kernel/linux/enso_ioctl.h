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

#ifndef SOFTWARE_KERNEL_LINUX_ENSO_IOCTL_H_
#define SOFTWARE_KERNEL_LINUX_ENSO_IOCTL_H_

#include "enso_setup.h"

/*
 * struct enso_send_tx_pipe_params - Structure used to send a TxPipe.
 * */
struct enso_send_tx_pipe_params {
  uint64_t phys_addr;
  uint32_t len;
  uint32_t id;
} __attribute__((packed));

struct enso_pipe_init_params {
  uint64_t phys_addr;
  uint32_t id;
} __attribute__((packed));

struct enso_consume_rx_params {
  uint32_t id;
  uint32_t new_rx_tail;
} __attribute__((packed));

struct enso_get_next_batch_params {
  uint32_t notif_id;
  uint32_t pipe_id;
  uint32_t new_rx_tail;
};

struct enso_advance_pipe_params {
  uint32_t id;
  size_t len;
};

#define ENSO_IOCTL_MAGIC 0x07
#define ENSO_IOCTL_TEST _IOR(ENSO_IOCTL_MAGIC, 0, unsigned int)
#define ENSO_IOCTL_GET_NB_FALLBACK_QUEUES \
  _IOR(ENSO_IOCTL_MAGIC, 1, unsigned int *)
#define ENSO_IOCTL_SET_RR_STATUS _IOR(ENSO_IOCTL_MAGIC, 2, bool)
#define ENSO_IOCTL_GET_RR_STATUS _IOR(ENSO_IOCTL_MAGIC, 3, bool *)
#define ENSO_IOCTL_ALLOC_NOTIF_BUFFER _IOR(ENSO_IOCTL_MAGIC, 4, unsigned int *)
#define ENSO_IOCTL_FREE_NOTIF_BUFFER _IOR(ENSO_IOCTL_MAGIC, 5, unsigned int)
#define ENSO_IOCTL_ALLOC_PIPE _IOR(ENSO_IOCTL_MAGIC, 6, unsigned int *)
#define ENSO_IOCTL_FREE_PIPE _IOR(ENSO_IOCTL_MAGIC, 7, unsigned int)
#define ENSO_IOCTL_ALLOC_NOTIF_BUF_PAIR _IOR(ENSO_IOCTL_MAGIC, 8, unsigned int)
#define ENSO_IOCTL_SEND_TX_PIPE \
  _IOR(ENSO_IOCTL_MAGIC, 9, struct enso_send_tx_pipe_params *)
#define ENSO_IOCTL_GET_UNREPORTED_COMPLETIONS \
  _IOR(ENSO_IOCTL_MAGIC, 10, unsigned int *)
#define ENSO_IOCTL_SEND_CONFIG \
  _IOR(ENSO_IOCTL_MAGIC, 11, struct tx_notification *)
#define ENSO_IOCTL_ALLOC_RX_ENSO_PIPE \
  _IOR(ENSO_IOCTL_MAGIC, 12, struct enso_pipe_init_params *)
#define ENSO_IOCTL_FREE_RX_ENSO_PIPE _IOR(ENSO_IOCTL_MAGIC, 13, unsigned int)
#define ENSO_IOCTL_CONSUME_RX \
  _IOR(ENSO_IOCTL_MAGIC, 14, struct enso_consume_rx_params *)
#define ENSO_IOCTL_FULL_ADV_PIPE _IOR(ENSO_IOCTL_MAGIC, 15, unsigned int *)
#define ENSO_IOCTL_GET_NEXT_BATCH \
  _IOR(ENSO_IOCTL_MAGIC, 16, struct enso_get_next_batch_params *)
#define ENSO_IOCTL_ADVANCE_PIPE \
  _IOR(ENSO_IOCTL_MAGIC, 17, struct enso_advance_pipe_params *)
#define ENSO_IOCTL_NEXT_RX_PIPE_RCV _IOR(ENSO_IOCTL_MAGIC, 18, unsigned int)
#define ENSO_IOCTL_PREFETCH_PIPE _IOR(ENSO_IOCTL_MAGIC, 19, unsigned int *)
#define ENSO_IOCTL_MAXNR 19

long enso_unlocked_ioctl(struct file *filp, unsigned int cmd,
                         unsigned long arg);
int free_rx_pipe(struct rx_pipe_internal *pipe);

#endif  // SOFTWARE_KERNEL_LINUX_ENSO_IOCTL_H_
