#include "fft.h"
#include "sim_types.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int is_power_of_2(size_t n)
{
    return n > 0 && (n & (n - 1)) == 0;
}

static void apply_hanning_window(const double *in, Complex *out, size_t n)
{
    double scale = 2.0 * M_PI / (double)(n - 1);
    for (size_t i = 0; i < n; i++) {
        double w = 0.5 * (1.0 - cos(scale * (double)i));
        out[i].re = in[i] * w;
        out[i].im = 0.0;
    }
}

static size_t bit_reverse(size_t val, int log2n)
{
    size_t result = 0;
    for (int i = 0; i < log2n; i++) {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    return result;
}

static void radix2_fft_inplace(Complex *x, size_t n)
{
    int log2n = 0;
    {
        size_t tmp = n;
        while (tmp > 1) { log2n++; tmp >>= 1; }
    }

    for (size_t i = 0; i < n; i++) {
        size_t j = bit_reverse(i, log2n);
        if (j > i) {
            Complex tmp = x[i];
            x[i] = x[j];
            x[j] = tmp;
        }
    }

    for (size_t s = 1; s <= (size_t)log2n; s++) {
        size_t m = (size_t)1 << s;
        double angle = -2.0 * M_PI / (double)m;
        Complex wm;
        wm.re = cos(angle);
        wm.im = sin(angle);

        for (size_t k = 0; k < n; k += m) {
            Complex w = {1.0, 0.0};
            for (size_t j = 0; j < m / 2; j++) {
                Complex t;
                t.re = w.re * x[k + j + m / 2].re - w.im * x[k + j + m / 2].im;
                t.im = w.re * x[k + j + m / 2].im + w.im * x[k + j + m / 2].re;

                Complex u = x[k + j];
                x[k + j].re = u.re + t.re;
                x[k + j].im = u.im + t.im;
                x[k + j + m / 2].re = u.re - t.re;
                x[k + j + m / 2].im = u.im - t.im;

                double wr = w.re * wm.re - w.im * wm.im;
                double wi = w.re * wm.im + w.im * wm.re;
                w.re = wr;
                w.im = wi;
            }
        }
    }
}

int fft_spectrum_dB(const double *signal, size_t n, double fs_hz,
                     double *freq_out, double *mag_dB_out, size_t max_bins)
{
    if (!signal || !freq_out || !mag_dB_out)
        return 0;
    if (n == 0 || !is_power_of_2(n))
        return 0;
    if (fs_hz <= 0.0)
        return 0;

    size_t half = n / 2;
    if (max_bins < half)
        return 0;

    Complex *buf = (Complex *)malloc(n * sizeof(Complex));
    if (!buf)
        return 0;

    apply_hanning_window(signal, buf, n);
    radix2_fft_inplace(buf, n);

    double df = fs_hz / (double)n;
    for (size_t i = 0; i < half; i++) {
        freq_out[i] = (double)i * df;
        double mag_sq = buf[i].re * buf[i].re + buf[i].im * buf[i].im;
        mag_dB_out[i] = 10.0 * log10(mag_sq);
    }

    free(buf);
    return (int)half;
}

int fft_complex_spectrum_dB(const double *re, const double *im,
                            size_t n, double fs_hz,
                            double *freq_out, double *mag_dB_out,
                            size_t max_bins) {
    if (!re || !im || !freq_out || !mag_dB_out) return 0;
    if (n == 0 || !is_power_of_2(n)) return 0;
    if (fs_hz <= 0.0) return 0;
    if (max_bins < n - 1u) return 0;

    Complex *buf = (Complex *)malloc(n * sizeof(Complex));
    if (!buf) return 0;

    double scale = 2.0 * M_PI / (double)(n - 1);
    for (size_t i = 0; i < n; i++) {
        double w = 0.5 * (1.0 - cos(scale * (double)i));
        buf[i].re = re[i] * w;
        buf[i].im = im[i] * w;
    }

    radix2_fft_inplace(buf, n);

    double df = fs_hz / (double)n;
    size_t half = n / 2u, wi = 0u;
    /* Negative frequencies: -fs/2 + df .. -df */
    for (size_t i = half + 1u; i < n; i++, wi++) {
        freq_out[wi] = -((double)(n - i)) * df;
        double mag_sq = buf[i].re * buf[i].re + buf[i].im * buf[i].im;
        mag_dB_out[wi] = 10.0 * log10(mag_sq);
    }
    /* DC */
    freq_out[wi] = 0.0;
    mag_dB_out[wi] = 10.0 * log10(buf[0].re * buf[0].re + buf[0].im * buf[0].im);
    wi++;
    /* Positive frequencies: df .. fs/2 - df */
    for (size_t i = 1u; i < half; i++, wi++) {
        freq_out[wi] = (double)i * df;
        double mag_sq = buf[i].re * buf[i].re + buf[i].im * buf[i].im;
        mag_dB_out[wi] = 10.0 * log10(mag_sq);
    }

    free(buf);
    return (int)wi;
}

static double *twiddles_re = NULL;
static double *twiddles_im = NULL;
static size_t twiddles_max_size = 0;

void fft_init(size_t n) {
    if (n <= twiddles_max_size) return;

    size_t new_size = 1;
    while (new_size < n) new_size <<= 1;

    double *new_re = (double *)realloc(twiddles_re, (new_size / 2) * sizeof(double));
    double *new_im = (double *)realloc(twiddles_im, (new_size / 2) * sizeof(double));
    if (!new_re || !new_im) {
        if (new_re) twiddles_re = new_re;
        if (new_im) twiddles_im = new_im;
        return;
    }
    twiddles_re = new_re;
    twiddles_im = new_im;

    for (size_t i = 0; i < new_size / 2; i++) {
        double angle = -2.0 * M_PI * (double)i / (double)new_size;
        twiddles_re[i] = cos(angle);
        twiddles_im[i] = sin(angle);
    }
    twiddles_max_size = new_size;
}

void fft_free(void) {
    free(twiddles_re);
    free(twiddles_im);
    twiddles_re = NULL;
    twiddles_im = NULL;
    twiddles_max_size = 0;
}

size_t next_pow2(size_t n) {
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

size_t bitrev(size_t x, size_t bits) {
    size_t rev = 0;
    for (size_t i = 0; i < bits; ++i) {
        rev = (rev << 1) | (x & 1);
        x >>= 1;
    }
    return rev;
}

void simple_fft(Complex *x, size_t N) {
    size_t bits = 0;
    size_t tmp = N;
    while (tmp > 1) { bits++; tmp >>= 1; }

    for (size_t i = 0; i < N; ++i) {
        size_t j = bitrev(i, bits);
        if (i < j) {
            double re = x[i].re;
            double im = x[i].im;
            x[i].re = x[j].re;
            x[i].im = x[j].im;
            x[j].re = re;
            x[j].im = im;
        }
    }

    fft_init(N);

    for (size_t size = 2; size <= N; size <<= 1) {
        size_t step = twiddles_max_size / size;
        for (size_t i = 0; i < N; i += size) {
            for (size_t j = 0; j < size / 2; ++j) {
                size_t idx1 = i + j;
                size_t idx2 = idx1 + size / 2;

                double w_re = twiddles_re[j * step];
                double w_im = twiddles_im[j * step];

                double t_re = x[idx2].re * w_re - x[idx2].im * w_im;
                double t_im = x[idx2].re * w_im + x[idx2].im * w_re;

                x[idx2].re = x[idx1].re - t_re;
                x[idx2].im = x[idx1].im - t_im;
                x[idx1].re += t_re;
                x[idx1].im += t_im;
            }
        }
    }
}

void simple_ifft(Complex *X, size_t N) {
    for (size_t i = 0; i < N; ++i) {
        X[i].im = -X[i].im;
    }
    simple_fft(X, N);
    for (size_t i = 0; i < N; ++i) {
        X[i].re /= (double)N;
        X[i].im /= (double)N;
    }
}

void fft_convolve_complex(const Complex *restrict a, size_t n_a,
                          const double *b_real, size_t n_b,
                          Complex *restrict out, size_t out_len) {
    size_t result_len = n_a + n_b - 1;
    size_t fft_size = next_pow2(result_len);

    Complex *A = (Complex *)calloc(fft_size, sizeof(Complex));
    Complex *B = (Complex *)calloc(fft_size, sizeof(Complex));
    if (!A || !B) {
        if (A) free(A);
        if (B) free(B);
        return;
    }

    for (size_t i = 0; i < n_a; ++i) A[i] = a[i];
    for (size_t i = 0; i < n_b; ++i) { B[i].re = b_real[i]; B[i].im = 0.0; }

    simple_fft(A, fft_size);
    simple_fft(B, fft_size);

    for (size_t i = 0; i < fft_size; ++i) {
        double re = A[i].re * B[i].re - A[i].im * B[i].im;
        double im = A[i].re * B[i].im + A[i].im * B[i].re;
        A[i].re = re;
        A[i].im = im;
    }

    simple_ifft(A, fft_size);

    for (size_t i = 0; i < out_len; ++i) {
        out[i].re = A[i].re;
        out[i].im = A[i].im;
    }

    free(A);
    free(B);
}