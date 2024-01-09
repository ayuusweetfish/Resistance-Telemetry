#include <float.h>  // FLT_EPSILON
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define SQR(_x) ((_x) * (_x))
#define SIGN(_a, _b) ((_b) >= 0 ? \
  ((_a) >= 0 ? (_a) : -(_a)) : \
  ((_a) >= 0 ? -(_a) : (_a)))
#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#define MAX(_a, _b) ((_a) > (_b) ? (_a) : (_b))
static inline float pythag(float a, float b)
{
  float absa = fabsf(a), absb = fabsf(b);
  return (absa > absb
    ? absa * sqrtf(1.0 + SQR(absb / absa))
    : (absb == 0.0 ? 0.0 : absb * sqrtf(1.0 + SQR(absa / absb))));
}
static inline void mat_svd(
  int m, int n,
  float *restrict u,
  float *restrict w,
  float *restrict v
) {
  // Numerical Recipes
  // http://numerical.recipes/webnotes/nr3web2.pdf
  // 'a,'bs/\([uv]\)\[\([^\]()]\+\)\]\[\([^\]()]\+\)\]/\1[\2 * n + \3]/g

  // SVD::decompose()
  bool flag;
  int i, its, j, jj, k, l, nm;
  float anorm, c, f, g, h, s, scale, x, y, z;
  float rv1[n];
  g = scale = anorm = 0.0; // Householder reduction to bidiagonal form.
  for (i = 0; i < n; i++) {
    l = i + 2;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m) {
      for (k = i; k < m; k++) scale += fabsf(u[k * n + i]);
      if (scale != 0.0) {
        for (k = i; k < m; k++) {
          u[k * n + i] /= scale;
          s += u[k * n + i] * u[k * n + i];
        }
        f = u[i * n + i];
        g = -SIGN(sqrtf(s), f);
        h = f * g - s;
        u[i * n + i] = f - g;
        for (j = l - 1; j < n; j++) {
          for (s = 0.0, k = i; k < m; k++) s += u[k * n + i] * u[k * n + j];
          f = s / h;
          for (k = i; k < m; k++) u[k * n + j] += f * u[k * n + i];
        }
        for (k = i; k < m; k++) u[k * n + i] *= scale;
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if (i + 1 <= m && i + 1 != n) {
      for (k = l - 1; k < n; k++) scale += fabsf(u[i * n + k]);
      if (scale != 0.0) {
        for (k = l - 1; k < n; k++) {
          u[i * n + k] /= scale;
          s += u[i * n + k] * u[i * n + k];
        }
        f = u[i * n + l - 1];
        g = -SIGN(sqrtf(s), f);
        h = f * g - s;
        u[i * n + l - 1] = f - g;
        for (k = l - 1; k < n; k++) rv1[k] = u[i * n + k] / h;
        for (j = l - 1; j < m; j++) {
          for (s = 0.0, k = l - 1; k < n; k++) s += u[j * n + k] * u[i * n + k];
          for (k = l - 1; k < n; k++) u[j * n + k] += s * rv1[k];
        }
        for (k = l - 1; k < n; k++) u[i * n + k] *= scale;
      }
    }
    anorm = MAX(anorm, (fabsf(w[i]) + fabsf(rv1[i])));
  }
  for (i = n - 1; i >= 0; i--) { // Accumulation of right-hand transformations.
    if (i < n - 1) {
      if (g != 0.0) {
        for (j = l; j < n; j++) // Double division to avoid possible underflow.
          v[j * n + i] = (u[i * n + j] / u[i * n + l]) / g;
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++) s += u[i * n + k] * v[k * n + j];
          for (k = l; k < n; k++) v[k * n + j] += s * v[k * n + i];
        }
      }
      for (j = l; j < n; j++) v[i * n + j] = v[j * n + i] = 0.0;
    }
    v[i * n + i] = 1.0;
    g = rv1[i];
    l = i;
  }
  for (i = MIN(m, n) - 1; i >= 0; i--) {
    // Accumulation of left-hand transformations.
    l = i + 1;
    g = w[i];
    for (j = l; j < n; j++)
      u[i * n + j] = 0.0;
    if (g != 0.0) {
      g = 1.0 / g;
      for (j = l; j < n; j++) {
        for (s = 0.0, k = l; k < m; k++) s += u[k * n + i] * u[k * n + j];
        f = (s / u[i * n + i]) * g;
        for (k = i; k < m; k++) u[k * n + j] += f * u[k * n + i];
      }
      for (j = i; j < m; j++)
        u[j * n + i] *= g;
    } else for (j = i; j < m; j++) u[j * n + i] = 0.0;
    ++u[i * n + i];
  }
  for (k = n - 1; k >= 0; k--) {
    // Diagonalization of the bidiagonal form: Loop over
    // singular values, and over allowed iterations.
    for (its = 0; its < 30; its++) {
      flag = true;
      for (l = k; l >= 0; l--) { // Test for splitting.
        nm = l - 1;
        if (l == 0 || fabsf(rv1[l]) <= FLT_EPSILON * anorm) {
          flag = false;
          break;
        }
        if (fabsf(w[nm]) <= FLT_EPSILON * anorm) break;
      }
      if (flag) {
        c = 0.0; // Cancellation of rv1[l], if l > 0.
        s = 1.0;
        for (i = l; i < k + 1; i++) {
          f = s * rv1[i];
          rv1[i] = c * rv1[i];
          if (fabsf(f) <= FLT_EPSILON * anorm) break;
          g = w[i];
          h = pythag(f, g);
          w[i] = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          for (j = 0; j < m; j++) {
            y = u[j * n + nm];
            z = u[j * n + i];
            u[j * n + nm] = y * c + z * s;
            u[j * n + i] = z * c - y * s;
          }
        }
      }
      z = w[k];
      if (l == k) { // Convergence.
        if (z < 0.0) { // Singular value is made nonnegative.
          w[k] = -z;
          for (j = 0; j < n; j++) v[j * n + k] = -v[j * n + k];
        }
        break;
      }
      if (its == 29)
        puts("no convergence in 30 svdcmp iterations");
      x = w[l]; // Shift from bottom 2-by-2 minor.
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
      c = s = 1.0; // Next QR transformation:
      for (j = l; j <= nm; j++) {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        for (jj = 0; jj < n; jj++) {
          x = v[jj * n + j];
          z = v[jj * n + i];
          v[jj * n + j] = x * c + z * s;
          v[jj * n + i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z; // Rotation can be arbitrary if z = 0.
        if (z) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = c * g + s * y;
        x = c * y - s * g;
        for (jj = 0; jj < m; jj++) {
          y = u[jj * n + j];
          z = u[jj * n + i];
          u[jj * n + j] = y * c + z * s;
          u[jj * n + i] = z * c - y * s;
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }

  // Prior to reordering, replace with negative tags
  // the singular values corresponding to a zero column vector in U
  // so that they be ordered last
  for (int j = 0; j < n; j++) {
    for (i = 0; i < m; i++)
      if (fabsf(u[i * n + j]) >= FLT_EPSILON * anorm) break;
    if (i == m) w[j] = -1.0;
  }

  // SVD::reorder()
  int inc = 1;
  float sw;
  float su[m], sv[n];
  do {
    inc *= 3;
    inc++;
  } while (inc <= n);
  // Sort. The method is Shell’s sort.
  // (The work is negligible as compared to that already done in decompose.)
  do {
    inc /= 3;
    for (i = inc; i < n; i++) {
      sw = w[i];
      for (k = 0; k < m; k++) su[k] = u[k * n + i];
      for (k = 0; k < n; k++) sv[k] = v[k * n + i];
      j = i;
      while (w[j - inc] < sw) {
        w[j] = w[j - inc];
        for (k = 0; k < m; k++) u[k * n + j] = u[k * n + j - inc];
        for (k = 0; k < n; k++) v[k * n + j] = v[k * n + j - inc];
        j -= inc;
        if (j < inc) break;
      }
      w[j] = sw;
      for (k = 0; k < m; k++) u[k * n + j] = su[k];
      for (k = 0; k < n; k++) v[k * n + j] = sv[k];
    }
  } while (inc > 1);
  for (k = 0; k < n; k++) { // Flip signs.
    int s = 0;
    for (i = 0; i < m; i++) if (u[i * n + k] < 0.) s++;
    for (j = 0; j < n; j++) if (v[j * n + k] < 0.) s++;
    if (s > (m + n) / 2) {
      for (i = 0; i < m; i++) u[i * n + k] = -u[i * n + k];
      for (j = 0; j < n; j++) v[j * n + k] = -v[j * n + k];
    }
  }

  // Recover negative tags
  for (j = n - 1; j >= 0 && w[j] < 0.; j--) w[j] = 0.;
}
#undef SQR
#undef SIGN
#undef MIN
#undef MAX

#define idx(_nrows, _ncols, _row, _col) ((_row)*(_ncols) + (_col))

static inline void mat_print(
  int m, int n,
  const float *a
) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++)
      printf(" %7.3f", a[idx(m, n, i, j)]);
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

int main()
{
  const int M = 4;
  const int N = 5;
  float A[] = {
    1, 0, 0, 0, 2,
    0, 0, 3, 0, 0,
    0, 0, 0, 0, 0,
    0, 2, 0, 0, 0,
  };
  float w[10], V[100];
  mat_svd(M, N, A, w, V);
  mat_print(M, N, A); putchar('\n');
  mat_print(1, N, w); putchar('\n');
  mat_print(N, N, V); putchar('\n');

  float W[100] = { 0 };
  for (int i = 0; i < N; i++) W[i * N + i] = w[i];
  float Vt[100], WV[100], UWV[100];
  mat_transpose(N, N, Vt, V);
  mat_mul(N, N, N, WV, W, Vt);
  mat_print(N, N, W); putchar('\n');
  mat_print(N, N, WV); putchar('\n');
  mat_mul(M, N, N, UWV, A, WV);
  mat_print(M, N, UWV); putchar('\n');

  return 0;
}
