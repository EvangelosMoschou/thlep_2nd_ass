#include <math.h>
#include <stddef.h>

#include "metrics.h"
#include "sim_types.h"
#include "physics.h"
#include "math_utils.h"

/*
 * mean_power_complex — Calculate the average power of a complex signal
 *
 * Formula: P = (1/N) × Σ(I_i² + Q_i²)
 */
double mean_power_complex(const Complex *x, size_t n) {
  size_t i;
  double p = 0.0;

  if (!x || n == 0u) {
    return 0.0;
  }

#pragma omp parallel for reduction(+ : p)
  for (i = 0; i < n; ++i) {
    p += x[i].re * x[i].re + x[i].im * x[i].im;
  }
  return p / (double)n;
}

/*
 * mean_noise_power_complex — Calculate the noise power by comparing signal to
 * reference (MSE)
 *
 * Formula: P_noise = (1/N) × Σ|sig_i - ref_i|²
 */
double mean_noise_power_complex(const Complex *restrict sig,
                                const Complex *restrict ref, size_t n) {
  size_t i;
  double p = 0.0;

  if (!sig || !ref || n == 0u) {
    return 0.0;
  }

#pragma omp parallel for reduction(+ : p)
  for (i = 0; i < n; ++i) {
    const double dr = sig[i].re - ref[i].re;
    const double di = sig[i].im - ref[i].im;
    p += dr * dr + di * di;
  }
  return p / (double)n;
}

/*
 * mean_power_real — Calculate the average power of a real-valued signal
 *
 * Formula: P = (1/N) × Σ x_i²
 */
double mean_power_real(const double *x, size_t n) {
  size_t i;
  double p = 0.0;

  if (!x || n == 0u) {
    return 0.0;
  }

#pragma omp parallel for reduction(+ : p)
  for (i = 0; i < n; ++i) {
    p += x[i] * x[i];
  }
  return p / (double)n;
}

/*
 * mean_noise_power_real — Noise power for real signals (MSE between sig and
 * ref)
 *
 * Formula: P_noise = (1/N) × Σ(sig_i - ref_i)²
 */
double mean_noise_power_real(const double *restrict sig,
                             const double *restrict ref, size_t n) {
  size_t i;
  double p = 0.0;

  if (!sig || !ref || n == 0u) {
    return 0.0;
  }

#pragma omp parallel for reduction(+ : p)
  for (i = 0; i < n; ++i) {
    const double d = sig[i] - ref[i];
    p += d * d;
  }
  return p / (double)n;
}

/*
 * compute_snr_db — Compute SNR in dB from signal and noise power
 *
 * Formula: SNR_dB = 10 × log10(signal_power / noise_power)
 */
double compute_snr_db(double signal_power, double noise_power) {
  if (signal_power > 0.0 && noise_power > 0.0) {
    return lin_to_db(signal_power / noise_power);
  } else if (signal_power > 0.0 && noise_power == 0.0) {
    return INFINITY;
  } else {
    return -INFINITY;
  }
}

/*
 * compute_evm_pct — Compute Error Vector Magnitude as a percentage
 *
 * Formula: EVM(%) = sqrt(noise_power / signal_power) × 100
 */
double compute_evm_pct(double signal_power, double noise_power) {
  if (signal_power > 0.0 && noise_power > 0.0) {
    return sqrt(noise_power / signal_power) * 100.0;
  } else if (signal_power > 0.0 && noise_power == 0.0) {
    return 0.0;
  } else {
    return NAN;
  }
}
