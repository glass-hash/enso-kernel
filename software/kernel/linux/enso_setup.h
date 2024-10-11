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

#ifndef SOFTWARE_KERNEL_LINUX_ENSO_SETUP_H_
#define SOFTWARE_KERNEL_LINUX_ENSO_SETUP_H_

#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#define ENSO_DRIVER_NAME "enso"
#define NOTIFICATION_BUF_SIZE 16384
#define ENSO_PIPE_SIZE 32768
#define MAX_TRANSFER_LEN 131072
#define HUGE_PAGE_SIZE (0x1ULL << 21)
#define MEM_PER_QUEUE (0x1ULL << 12)
#define BATCH_SIZE 64

// These determine the maximum number of notification buffers and enso pipes.
// These macros also exist in hardware and **must be kept in sync**. Update the
// variables with the same name on `hardware/src/constants.sv`,
// `software/include/enso/consts.h`, and `scripts/hwtest/my_stats.tcl`.
#define MAX_NB_APPS 1024
#define MAX_NB_FLOWS 8192

/**
 * @struct enso_intel_pcie
 *
 * @brief Stores the PCIe Base Address Resgiter (BAR) information from
 *         Intel's drivers. Note: we only use BAR2's information, rest are
 *         unused.
 * */
struct enso_intel_pcie {
  void *__iomem base_addr;
};

/**
 * @struct enso_global_bookkeep
 *
 * @brief Global bookkeeping information relevant to the Enso driver.
 *
 * @var cdev Used to create a character device.
 * @var chr_class Used to creating the enso device under /dev/.
 * @var intel_enso The start of the BAR region to be mapped into the kernel
 * space.
 * @var chr_major Major number of the device.
 * @var chr_minor Minor number of the device.
 * */
struct enso_global_bookkeep {
  struct cdev cdev;
  struct class *chr_class;
  struct enso_intel_pcie *intel_enso;
  int chr_major;
  int chr_minor;
  struct dev_bookkeep *dev_bk;
};

/**
 * @struct dev_bookkeep
 *
 * @brief Bookkeeping structure per device.
 *
 * @var chr_open_cnt   Number of character device handles opened with
 *                  this device selected.
 * @var sem            Synchronizes accesses to this structure. Can be used in
 *                  interrupt context. Currently, only @chr_open_cnt requires
 *                  synchronization as other fields are written once.
 * @var nb_fb_queues   Number of fallback queues.
 * @var enable_rr     Enable round-robin scheduling among fallback queues.
 * @var notif_q_status Bit vector to keep track of which notification queue has
 *                  been allocated.
 * @var rx_pipe_id_status   Bit vector to keep track of which Rx Pipe ID has
 * been allocated.
 * @var tx_pipe_id_status   Bit vector to keep track of which Tx Pipe ID has
 * been allocated.
 */
struct dev_bookkeep {
  struct semaphore sem;
  uint8_t *notif_q_status;
  uint8_t *rx_pipe_id_status;
  uint8_t *tx_pipe_id_status;
  uint32_t chr_open_cnt;
  uint32_t nb_fb_queues;
  uint32_t nb_tx_pipes;
  bool enable_rr;
};

/**
 * @struct chr_dev_bookkeep
 *
 * @brief Bookkeeping structure per character device handle.
 *
 * @var dev_bk         Pointer to the device bookkeeping structure for the
 *                  currently selected device.
 * @var nb_fb_queues   Number of fallback queues.
 * @var notif_q_status Bit vector to keep track of which notification queue has
 *                  been allocated for this particular character device.
 * @var pipe_status    Bit vector to keep track of which pipe has been allocated
 *                  for this particular character device.
 * @var notif_buf_pair Notification buffer pair specific to this device handle.
 * @var rx_pipes Enso RX pipes specific to this device handle.
 */
struct chr_dev_bookkeep {
  struct dev_bookkeep *dev_bk;
  uint32_t nb_fb_queues;
  uint8_t *notif_q_status;
  uint8_t *rx_pipe_id_status;
  uint8_t *tx_pipe_id_status;
  struct notification_buf_pair *notif_buf_pair;
  struct rx_pipe_internal **rx_pipes;
};

/**
 * @struct rx_notification
 *
 * @brief Represents the structure of an Rx notification.
 *
 * @var signal 1 for a valid Rx notification
 * @var queue_id pipe ID of the enso pipe this notification belongs to
 * @var tail TODO: figure this out
 */
struct __attribute__((__packed__)) rx_notification {
  uint64_t signal;
  uint64_t queue_id;
  uint64_t tail;
  uint64_t pad[5];
};

/**
 * @struct tx_notification
 *
 * @brief Represents the structure of a Tx notification. Also used
 * to send configuration to the NIC.
 *
 * @var signal 1 for data, 2 for configuration.
 * @var phys_addr starting address of the batch (physical address).
 * @var length size of the batch in bytes (up to 1 MB).
 */
struct __attribute__((__packed__)) tx_notification {
  uint64_t signal;
  uint64_t phys_addr;
  uint64_t length;
  uint64_t pad[5];
};

/**
 * @struct queue_regs
 *
 * @brief Core structure representing a queue metadata of Rx and Tx
 * notifications in a notification buffer pair structure. This structure is
 * mapped into IO memory of the Enso NIC and configures the start Rx and Tx
 * notification buffers (memory information, head, tail). Part of the
 * notification_buf_pair structure.
 *
 * @var rx_tail tail of the Rx notification buffer.
 * @var rx_head head of the Rx notification buffer.
 * @var rx_mem_low contains the least significant 32 bits of the Rx buffer's
 * physical address.
 * @var rx_mem_high holds the most significant 32 bits of the Rx buffer's
 * physical address.
 * @var tx_tail tail of the Tx notification buffer.
 * @var tx_head tail of the Rx notification buffer.
 * @var tx_mem_low contains the least significant 32 bits of the Tx notification
 * buffer's physical address.
 * @var tx_mem_high contains the most significant 32 bits of the Tx notification
 * buffer's physical address.
 */
struct queue_regs {
  uint32_t rx_tail;
  uint32_t rx_head;
  uint32_t rx_mem_low;
  uint32_t rx_mem_high;
  uint32_t tx_tail;
  uint32_t tx_head;
  uint32_t tx_mem_low;
  uint32_t tx_mem_high;
  uint32_t padding[8];
};

/**
 * @struct notification_buf_pair
 *
 * @brief Represents a notification buffer pair (Rx and Tx).
 *
 * @var rx_buf Rx notification buffer (DMA allocated).
 * @var tx_buf Tx notification buffer (DMA allocated).
 * @var rx_head_ptr Points to the head of the Rx queue (tail is modified by the
 * NIC).
 * @var tx_tail_ptr Points to the tail of the Tx queue (head is modified by the
 * NIC).
 * @var regs queue_regs Storing Rx and Tx notification buffer metadata (memory
 * mapped IO).
 * @var next_rx_pipe_ids Next rx pipe id to consume, used in prefetching
 * batches.
 * @var wrap_tracker Keeps track of notifications where the packet wraps around
 * the buffer's boundary.
 * @var pending_rx_pipe_tails Stores the tail of the pipe (indexed by pipe ID).
 * @var tx_full_cnt Number of times the Tx notification buffer was full.
 * @var id Notification buffer ID.
 * @var rx_head Used to store the value pointed by `rx_head_ptr`.
 * @var tx_head Used to store the value of Tx notification buffer's head.
 * @var tx_tail Used to store the value pointed by `tx_tail_ptr`.
 * @var nb_unreported_completions Stores the number of unreported Tx
 * notification completions to the application.
 * @var next_rx_ids_head Stores the head of the next pipe ID to be fetched
 * during prefetching.
 * @var next_rx_ids_tail Stores the tail of the next pipe ID to be fetched
 * during prefetching.
 * */
struct notification_buf_pair {
  struct rx_notification *rx_buf;
  struct tx_notification *tx_buf;
  uint32_t *rx_head_ptr;
  uint32_t *tx_tail_ptr;
  struct queue_regs *regs;
  uint32_t *next_rx_pipe_ids;
  uint8_t *wrap_tracker;
  uint32_t *pending_rx_pipe_tails;

  uint64_t tx_full_cnt;

  uint32_t id;
  uint32_t rx_head;
  uint32_t tx_head;
  uint32_t tx_tail;
  uint32_t nb_unreported_completions;

  uint16_t next_rx_ids_head;
  uint16_t next_rx_ids_tail;
};

/**
 * struct rx_pipe_internal
 *
 * @brief Represents an Enso Rx pipe.
 *
 * @var regs queue_regs Stores Rx and Tx notification buffer metadata (memory
 * mapped IO).
 * @var buf_head_ptr pointer to the head of the Rx queue.
 * @var rx_head stores the head of the Rx queue.
 * @var rx_tail stores the tail of the Rx queue.
 * @var id ID of the Rx pipe.
 * @var allocated whether the Rx pipe is allocated or not.
 * */
struct rx_pipe_internal {
  struct queue_regs *regs;
  uint32_t *buf_head_ptr;

  uint32_t rx_head;
  uint32_t rx_tail;
  uint32_t id;

  bool allocated;
};

// extern as it is used by enso_chr.c to fill out the character device info.
extern struct enso_global_bookkeep global_bk;

#endif  // SOFTWARE_KERNEL_LINUX_ENSO_SETUP_H_
