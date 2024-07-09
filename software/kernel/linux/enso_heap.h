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
#ifndef SOFTWARE_KERNEL_LINUX_ENSO_HEAP_H_
#define SOFTWARE_KERNEL_LINUX_ENSO_HEAP_H_

#include "enso_setup.h"

#define MAX_HEAP_SIZE 100000

struct tx_queue_node {
  struct enso_send_tx_pipe_params batch;
  unsigned long ftime;
};

struct heap_node {
  struct tx_queue_node *queue_node;
};

struct min_heap {
  struct heap_node *harr;
  uint32_t capacity;
  uint32_t size;
};

void init_heap(struct min_heap *heap);
void insert_heap(struct min_heap *heap, struct tx_queue_node *node);
void show_heap(struct min_heap *heap);
struct tx_queue_node *top(struct min_heap *heap);
void pop(struct min_heap *heap);
void heapify(struct min_heap *heap, int ind);
void free_heap(struct min_heap *heap);
uint32_t get_heap_size(struct min_heap *heap);

#endif  // SOFTWARE_KERNEL_LINUX_ENSO_HEAP_H_
