// Copyright 2021 ETH Zurich and University of Bologna.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: Domenic Wüthrich, ETH Zurich

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "kernel/matmul.c"
#include "printf.h"
#ifdef MEMPOOL
#include "runtime.h"
#include "synchronization.h"
#endif

// Define Matrix dimensions:
// C = AB with A=[MxN], B=[NxP], C=[MxP]
#ifndef MATRIX_DIM
#define MATRIX_DIM 256
#endif
#ifndef KERNEL_M
#define KERNEL_M 2
#endif
#ifndef KERNEL_P
#define KERNEL_P 2
#endif
#define M MATRIX_DIM
#define N MATRIX_DIM
#define P MATRIX_DIM
// Specify how the matrices A and B should be initialized
// The entries will follow this format:
// a(i,j) = A_a*i + A_b*j + A_c
// b(i,j) = B_a*i + B_b*j + B_c
// The result will be the following matrix
// c(i,j) = (A_a*B_b*i*j + A_a*B_c*i + A_c*B_b*j + A_c*B_c) * N
//        + (A_a*B_a*i + A_b*B_b*j + A_b*B_c + B_a*A_c) * (N*(N-1))/2
//        + (A_b*B_a) * (N*(N-1)*(2*N-1))/6
// Note: To keep the code simpler, we use indices that go from 0 to N-1 instead
// of 1 to N as the mathematicians do. Hence, for A, i=[0,M-1] j=[0,M-1]
#define A_a 1
#define A_b 1
#define A_c -32
#define B_a 2
#define B_b 1
#define B_c 16

int32_t a[M * N] __attribute__((aligned(4), section(".l1")));
int32_t b[N * P] __attribute__((aligned(4), section(".l1")));
int32_t c[M * P] __attribute__((aligned(4), section(".l1")));

// Initialize the matrices
void init_matrix(int32_t *matrix, uint32_t rows_start, uint32_t rows_end,
                 uint32_t num_columns, int32_t a, int32_t b, int32_t c) {
  for (uint32_t i = rows_start; i < rows_end; ++i) {
    for (uint32_t j = 0; j < num_columns; ++j) {
      matrix[i * num_columns + j] = a * (int32_t)i + b * (int32_t)j + c;
    }
  }
}

// Verify the matrices
int verify_matrix(int32_t *matrix, uint32_t row_start, uint32_t row_end,
                  uint32_t n, uint32_t p, int32_t aa, int32_t ab, int32_t ac,
                  int32_t ba, int32_t bb, int32_t bc) {
  for (int32_t i = (int32_t)row_start; i < (int32_t)row_end; i++) {
    for (int32_t j = 0; j < (int32_t)p; j++) {
      int32_t lin =
          (aa * bb * i * j + aa * bc * i + ac * bb * j + ac * bc) * (int32_t)n;
      int32_t qua = ((aa * ba * i + ab * bb * j + ab * bc + ba * ac) *
                     (int32_t)(n * (n - 1))) /
                    2;
      int32_t cub = ((ab * ba) * (int32_t)(n * (n - 1) * (2 * n - 1))) / 6;
      int32_t golden = lin + qua + cub;
      if (matrix[i * (int32_t)p + j] != golden) {
        return (i + j) == 0 ? -1 : i * (int32_t)p + j;
      }
      matrix[i * (int32_t)p + j] = 0;
    }
  }
  return 0;
}

void print_matrix(int32_t const *matrix, uint32_t num_rows,
                  uint32_t num_columns) {
  printf("0x%8X\n", (uint32_t)matrix);
  for (uint32_t i = 0; i < num_rows; ++i) {
    for (uint32_t j = 0; j < num_columns; ++j) {
      printf("%5d ", matrix[i * num_columns + j]);
    }
    printf("\n");
  }
}

int main() {
  uint32_t num_cores = mempool_get_core_count();
  uint32_t cores_per_group = num_cores / NUM_GROUPS;
  uint32_t core_id = mempool_get_core_id();
  uint32_t core_id_group = core_id % cores_per_group;
  uint32_t group_id = core_id / cores_per_group;

  uint32_t timer_start, timer_end, timer;
  uint32_t row_start, row_end;

  uint32_t m_start, m_end;
  uint32_t p_start, p_end;
  uint32_t vl;
  uint32_t dim;
  uint32_t kernel_size;
  uint32_t measure_iterations = 1;

  // Initialize multicore barrier
  mempool_barrier_init(core_id);

  // Reset timer
  timer = (uint32_t)-1;

  // Set matrix dimension
  dim = MATRIX_DIM;
  kernel_size = KERNEL_M;
  vl = KERNEL_P;

  // Can every core execute its desired kernel?
  if ((dim * dim) / (kernel_size * vl) < num_cores)
    return -1;
  // Does the vl fit inside the dim
  if (vl > dim)
    return -2;

  // Block dimension of group
  uint32_t dim_group = dim / NUM_GROUPS;
  // Number of parallel cores in m direction
  uint32_t split_m_count = dim_group / kernel_size;

  if (split_m_count < cores_per_group) {
    // Split P dimension up
    uint32_t split_p_count = cores_per_group / split_m_count;
    p_start = dim / split_p_count * (core_id_group % split_p_count);
    p_end = dim / split_p_count * ((core_id_group % split_p_count) + 1);
    m_start =
        dim_group * group_id + kernel_size * (core_id_group / split_p_count);
    m_end = dim_group * group_id +
            kernel_size * (core_id_group / split_p_count + 1);
  } else {
    // Work over complete P dimension
    p_start = 0;
    p_end = dim;
    m_start =
        dim_group * group_id + (dim_group / cores_per_group) * core_id_group;
    m_end = dim_group * group_id +
            (dim_group / cores_per_group) * (core_id_group + 1);
  }

  // Initialize matrices
  uint32_t cores_per_row = num_cores / dim;
  uint32_t do_init_verify = 1;
  if (dim < num_cores) {
    row_start = core_id / cores_per_row;
    row_end = core_id / cores_per_row + 1;
    // Core will skip matrix init and verify parts
    do_init_verify = (core_id % cores_per_row) == 0;
  } else {
    row_start = dim / num_cores * (core_id);
    row_end = dim / num_cores * (core_id + 1);
  }

  if (do_init_verify) {
    init_matrix(a, row_start, row_end, dim, A_a, A_b, A_c);
    init_matrix(b, row_start, row_end, dim, B_a, B_b, B_c);
  }

  // Execute matmul a few times and measure runtime
  for (uint32_t i = 0; i < measure_iterations; i++) {
    // Wait for all cores to finish
    mempool_barrier(num_cores);

    // Start timer
    timer_start = mempool_get_timer();

    // Calculate matmul
    if (kernel_size == 2) {
      matmul_2xVL(c, a, b, m_start, m_end, dim, dim, p_start, p_end, vl);
    } else if (kernel_size == 4) {
      matmul_4xVL(c, a, b, m_start, m_end, dim, dim, p_start, p_end, vl);
    } else if (kernel_size == 8) {
      matmul_8xVL(c, a, b, m_start, m_end, dim, dim, p_start, p_end, vl);
    } else {
      return -2;
    }
    // Wait for all cores to finish matmul
    mempool_barrier(num_cores);

    // End timer and check if new best runtime
    timer_end = mempool_get_timer();
    uint32_t timer_temp = timer_end - timer_start;
    if (core_id == 0) {
      if (timer_temp < timer) {
        timer = timer_temp;
      }
    }
  }

  // Check and display results
  if (core_id == 0) {
    uint32_t performance = 1000 * 2 * dim * dim * dim / timer;
    uint32_t utilization = performance / (2 * num_cores * N_IPU);

    printf("\n----- (%dx%d) matmul -----\n", dim, dim);
    printf("The execution took %u cycles.\n", timer);
    printf("The performance is %u OP/1000cycle (%u%%o utilization).\n",
           performance, utilization);
  }

  if (do_init_verify) {
    int32_t error = verify_matrix(c, row_start, row_end, dim, dim, A_a, A_b,
                                  A_c, B_a, B_b, B_c);

    if (error != 0) {
      printf("Error core %d: c[%d]=%d\n", core_id, error, c[error]);
      return error;
    }
  }

  // Wait for core 0 to finish displaying results
  mempool_barrier(num_cores);

  return 0;
}
