// Data compression testbed

#include <stdint.h>
#include <stdio.h>

#define idx(_nrows, _ncols, _row, _col) ((_row)*(_ncols) + (_col))

static inline void mat_print(
  int m, int n,
  const float *a
) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++)
      printf(" %7.1f", a[idx(m, n, i, j)]);
    putchar('\n');
  }
}

static inline void mat_transpose(
  int m, int n,
  float *restrict t,        // n * m
  const float *restrict a   // m * n
) {
  for (int j = 0; j < m; j++)
    for (int i = 0; i < n; i++)
      t[idx(m, n, j, i)] = a[idx(m, n, i, j)];
}

static inline void mat_mul(
  int m, int n, int p,
  float *restrict c,        // m * p
  const float *restrict a,  // m * n
  const float *restrict b   // n * p
) {
  for (int i = 0; i < m; i++)
    for (int k = 0; k < p; k++) {
      c[idx(m, p, i, k)] = 0;
      for (int j = 0; j < n; j++)
        c[idx(m, p, i, k)] += a[idx(m, n, i, j)] * b[idx(n, p, j, k)];
    }
}

static inline void mat_svd(
  int m, int n,
  float *restrict u,        // m * m
  float *restrict s,        // min(m, n) * 1
  float *restrict v,        // n * n
  const float *restrict a   // m * n
) {
}

// Data format:
// All samples' LSB discarded (become 23 bits)
// 23b: the most current sample, sample 0
// Each next sample takes variable length
// - Define `diff` to be the difference between the real sample value
//   and the quad-fit based on all previous samples
//   - Note: 'quad-fit' refers to the least-squares-fit extrapolation
//     of the sample, which for sample 1 equals sample 0 and for sample 2
//     equals a straightforward two-point linear extrapolation
//   - The extrapolated value is truncated to the 23-bit unsigned range
// - The value of `diff` is encoded as follows:
//     0svvvvvv vvvvvvvv -- -16384~16383
//     1svvvvvv vvvvvvvv vvvvvvvv -- -4194304~4194303 (full range)
//   (Alternatively:)
//     0svvvvvv -- -64~63 (?)
//     1svvvvvv 0vvvvvvv -- -8192~8191
//     1svvvvvv 1vvvvvvv vvvvvvvv -- -2097152~2097151 (?)

#define MAX_N 12
void compress_24b_values(uint32_t *values, size_t count, uint8_t *buffer, size_t length)
{
  buffer[0] = (values[0] >>  0) & 0xff;
  buffer[1] = (values[0] >>  8) & 0xff;
  buffer[2] = (values[0] >> 16) & 0xff;
  size_t n = 1; // Values pointer
  size_t p = 3; // Buffer pointer
  while (n < count && p < length) {
    // LLS estimate (Moore–Penrose pseudoinverse)
    // A[n*3] = [1 i i^2]
    // A[n*3] x[3*1] = b[n*1]
    // Solution: x = (A* A)^-1 A* b
    // With SVD: A = U S V*
    // Solution: x = V S^-1 U* b
    float A[MAX_N * 3];
    for (int i = 0; i < n; i++) {
      A[idx(n, 3, i, 0)] = 1;
      A[idx(n, 3, i, 1)] = i;
      A[idx(n, 3, i, 2)] = i * i;
    }
    printf("n = %zu\n", n);
    mat_print(n, 3, A);
    float U[MAX_N * MAX_N], V[3 * 3], S[3];
    mat_svd(n, 3, A, U, S, V);
    n++;
  }
}

int main()
{
  uint32_t values[10];
  for (int i = 0; i < 10; i++) values[i] = (1 << 16) + i * i * 5 + 997 % (i + 2);
  uint8_t buffer[11];
  compress_24b_values(values, 10, buffer, 11);
  return 0;
}
