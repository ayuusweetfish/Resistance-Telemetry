// Data compression testbed

#include <stdint.h>
#include <stdio.h>

#define idx(_nrows, _ncols, _row, _col) ((_row)*(_ncols) + (_col))

static inline void mat_print(
  int n, int m,
  const float *a
) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < m; j++)
      printf(" %7.1f", a[idx(n, m, i, j)]);
    putchar('\n');
  }
}

static inline void mat_transpose(
  int n, int m,
  float *restrict t,
  const float *restrict a
) {
  for (int j = 0; j < m; j++)
    for (int i = 0; i < n; i++)
      t[idx(m, n, j, i)] = a[idx(n, m, i, j)];
}

static inline void mat_mul(
  int n, int m, int p,
  float *restrict c,
  const float *restrict a,
  const float *restrict b
) {
  for (int i = 0; i < n; i++)
    for (int k = 0; k < p; k++) {
      c[idx(n, p, i, k)] = 0;
      for (int j = 0; j < m; j++)
        c[idx(n, p, i, k)] += a[idx(n, m, i, j)] * b[idx(m, p, j, k)];
    }
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
    // LLS estimate
    // X[n*3] = [1 i i^2]
    // Beta = inv(X' X) X' y
    float x[MAX_N * 3];
    for (int i = 0; i < n; i++) {
      x[idx(n, 3, i, 0)] = 1;
      x[idx(n, 3, i, 1)] = i;
      x[idx(n, 3, i, 2)] = i * i;
    }
    float xT[3 * MAX_N];
    mat_transpose(n, 3, xT, x);
    printf("n = %zu\n", n);
    mat_print(n, 3, x);
    mat_print(3, n, xT);
    float a1[3 * 3];
    mat_mul(3, n, 3, a1, xT, x);
    mat_print(3, 3, a1);
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
