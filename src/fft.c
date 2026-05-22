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