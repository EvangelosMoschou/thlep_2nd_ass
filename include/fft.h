#ifndef FFT_H
#define FFT_H

#include <stddef.h>
#include "sim_types.h"

/*
 * fft_spectrum_dB - Compute single-sided power spectrum in dB.
 *
 * @signal:     Input real-valued time-domain samples (length n, power of 2).
 * @n:          Number of samples (must be a power of 2).
 * @fs_hz:      Sample rate in Hz. Used to compute frequency axis.
 * @freq_out:   Output frequency axis [0 .. fs/2], length >= n/2.
 * @mag_dB_out: Output magnitude-squared spectrum in dB (10*log10), length >= n/2.
 * @max_bins:   Capacity of freq_out / mag_dB_out arrays (must be >= n/2).
 *
 * Returns:     Number of bins filled (n/2), or 0 on error.
 *
 * A Hanning window is applied to the input before the FFT.
 * The DC bin (bin 0) and Nyquist bin are included.
 * Magnitude is computed as 10*log10(re^2 + im^2) per bin.
 */
int fft_spectrum_dB(const double *signal, size_t n, double fs_hz,
                    double *freq_out, double *mag_dB_out, size_t max_bins);

/*
 * fft_complex_spectrum_dB - Double-sided power spectrum for complex I/Q signals.
 *
 * Takes a complex (I/Q) time-domain signal and produces a double-sided
 * spectrum from -fs/2 to +fs/2 (excluding the Nyquist bin).  Useful for
 * baseband I/Q signals where negative frequencies carry information
 * (e.g. downconverted mixer outputs, QPSK, 64-APSK).
 *
 * @re, @im:  Real and imaginary parts of the signal (length n, power of 2).
 * @n:        Number of samples (must be a power of 2).
 * @fs_hz:    Sample rate in Hz.
 * @freq_out:    Output frequency axis [-fs/2 ... fs/2], length >= n-1.
 * @mag_dB_out:  Output magnitude spectrum in dB, length >= n-1.
 * @max_bins:    Capacity of freq_out / mag_dB_out arrays (must be >= n-1).
 *
 * Returns:  Number of bins filled (n-1), or 0 on error.
 */
int fft_complex_spectrum_dB(const double *re, const double *im,
                            size_t n, double fs_hz,
                            double *freq_out, double *mag_dB_out,
                            size_t max_bins);

/*
 * next_pow2 - Find the next power of 2 >= n
 */
size_t next_pow2(size_t n);

/*
 * bitrev - Compute bit-reversed index
 */
size_t bitrev(size_t x, size_t bits);

/*
 * simple_fft - In-place Cooley-Tukey FFT using precomputed twiddles.
 * N must be a power of 2.
 */
void simple_fft(Complex *x, size_t N);

/*
 * simple_ifft - In-place inverse FFT.
 */
void simple_ifft(Complex *X, size_t N);

/*
 * fft_convolve_complex - FFT-based convolution for complex signals.
 */
void fft_convolve_complex(const Complex *restrict a, size_t n_a,
                          const double *b_real, size_t n_b,
                          Complex *restrict out, size_t out_len);

/*
 * fft_init - Precompute/initialize twiddle factors for FFTs up to size N.
 */
void fft_init(size_t n);

/*
 * fft_free - Free resources allocated for twiddle factors.
 */
void fft_free(void);

#endif /* FFT_H */