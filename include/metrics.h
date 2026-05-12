#ifndef METRICS_H
#define METRICS_H

#include <stddef.h>
#include "sim_types.h"

/*
 * ============================================================================
 * MEASUREMENT / METRICS FUNCTIONS
 * ============================================================================
 *
 * These functions compute the key signal quality indicators by comparing
 * the "reference" (ideal, noise-free) signal to the "received" (degraded)
 * signal at each point in the receiver chain.
 */

/*
 * mean_power_complex — Calculate the average power of a complex signal
 *
 * Formula: P = (1/N) × Σ(I_i² + Q_i²)
 *
 * This is the mean squared magnitude of the complex signal.
 * For a unit-power constellation (Es = 1), this should return ≈ 1.0.
 *
 * Parameters:
 *   x — Array of complex samples
 *   n — Number of samples
 *
 * Returns:
 *   Mean power (always ≥ 0). Returns 0.0 if x is NULL or n is 0.
 */
double mean_power_complex(const Complex *x, size_t n);

/*
 * mean_noise_power_complex — Calculate the noise power by comparing signal to
 * reference
 *
 * Formula: P_noise = (1/N) × Σ|sig_i - ref_i|²
 *          = (1/N) × Σ((sig_I - ref_I)² + (sig_Q - ref_Q)²)
 *
 * This is the Mean Squared Error (MSE) between the received and reference
 * signals. It directly measures how much the signal has been corrupted by noise
 * and processing imperfections.
 *
 * Parameters:
 *   sig — Received (degraded) signal
 *   ref — Reference (ideal) signal
 *   n   — Number of samples
 *
 * Returns:
 *   Mean noise power (MSE). Returns 0.0 if signals are identical.
 */
double mean_noise_power_complex(const Complex *restrict sig,
                                const Complex *restrict ref, size_t n);

/*
 * mean_power_real — Calculate the average power of a real-valued signal
 *
 * Formula: P = (1/N) × Σ x_i²
 *
 * Used for RF signals, which are real-valued (not complex).
 */
double mean_power_real(const double *x, size_t n);

/*
 * mean_noise_power_real — Noise power for real signals (MSE between sig and
 * ref)
 *
 * Formula: P_noise = (1/N) × Σ(sig_i - ref_i)²
 */
double mean_noise_power_real(const double *restrict sig,
                             const double *restrict ref, size_t n);

/*
 * compute_snr_db — Compute SNR in dB from signal and noise power
 *
 * Formula: SNR_dB = 10 × log10(signal_power / noise_power)
 *
 * Parameters:
 *   signal_power — Measured signal power (linear, must be ≥ 0)
 *   noise_power  — Measured noise power (linear, must be ≥ 0)
 *
 * Returns:
 *   SNR in dB.
 *   +INFINITY if signal_power > 0 and noise_power == 0 (perfect signal).
 *   -INFINITY if signal_power == 0 (no signal — meaningless).
 *   -INFINITY if both are 0.
 */
double compute_snr_db(double signal_power, double noise_power);

/*
 * compute_evm_pct — Compute Error Vector Magnitude as a percentage
 *
 * Formula: EVM(%) = sqrt(noise_power / signal_power) × 100
 *
 * EVM measures the I/Q constellation error. It is only meaningful for
 * complex (I/Q) signals, not for real-valued RF signals.
 *
 * Parameters:
 *   signal_power — Measured signal power (linear, must be > 0)
 *   noise_power  — Measured noise power (linear, may be 0)
 *
 * Returns:
 *   EVM as a percentage (0-100+).
 *   0.0 if noise_power == 0 and signal_power > 0 (perfect signal).
 *   NAN if signal_power == 0 (EVM undefined).
 */
double compute_evm_pct(double signal_power, double noise_power);

/*
 * compute_stage_metric_complex — Fill a StageMetric from complex I/Q measurements
 *
 * Computes signal power, noise power, SNR, and EVM for a complex baseband
 * signal at a given receiver stage.
 *
 * Parameters:
 *   stage  — Stage name string (e.g., "LNA", "Mixer") — NOT copied, must persist
 *   domain — Signal domain string (e.g., "complex_baseband") — NOT copied
 *   ref    — Reference (ideal) complex signal
 *   sig    — Degraded received complex signal
 *   n      — Number of samples
 *
 * Returns:
 *   A fully populated StageMetric struct
 */
StageMetric compute_stage_metric_complex(const char *stage, const char *domain,
                                         const Complex *restrict ref,
                                         const Complex *restrict sig, size_t n);

/*
 * compute_stage_metric_real — Fill a StageMetric from real-valued measurements
 *
 * Same as compute_stage_metric_complex but for real-valued signals.
 * EVM is always set to NaN because it's only meaningful for I/Q constellation
 * signals.
 */
StageMetric compute_stage_metric_real(const char *stage, const char *domain,
                                      const double *restrict ref,
                                      const double *restrict sig, size_t n);

#endif /* METRICS_H */
