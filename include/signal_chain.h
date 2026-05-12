/**
 * @file signal_chain.h
 * @brief Receiver signal chain processor — per-stage signal processing functions.
 *
 * Each "apply_stage" function models what happens when a signal passes through
 * one physical component (stage) of the receiver chain. The processing follows
 * this sequence:
 *
 *   1. NONLINEARITY  — IP3 / P1dB compression (if configured)
 *   2. GAIN          — Amplify or attenuate by the stage's gain
 *   3. NOISE         — Add thermal noise based on Noise Figure (Friis formula)
 *   4. LIMITER       — Clip signal amplitude (if configured)
 *
 * The noise injection uses a T0-referenced power tracker to implement the
 * Friis noise cascade formula: each stage adds noise proportional to
 * (F - 1), where F is the linear noise figure, and later stages contribute
 * less relative noise because the signal has already been amplified.
 *
 * All PRNG-dependent functions accept a reentrant PrngState* parameter
 * (no global PRNG state is used).
 */

#ifndef SIGNAL_CHAIN_H
#define SIGNAL_CHAIN_H

#include <stddef.h>

#include "prng.h"
#include "sim_types.h"
#include "stage_models.h"

/**
 * @brief Process complex (I/Q) samples through one receiver stage.
 *
 * Operates on Complex arrays (interleaved re/im). Used for the complex
 * baseband analytical path and the post-downconversion baseband path.
 *
 * Processing steps:
 *   - Nonlinearity (IP3/P1dB compression on both ref and sig)
 *   - Gain (voltage gain = sqrt(power_gain_linear))
 *   - Noise injection (Friis formula: pn_add = N_t0_v2 × g_lin × (F - 1))
 *   - Limiter (if stg->is_limiter is set)
 *
 * @param stg          Stage model parameters (gain, NF, nonlinearity, etc.)
 * @param ref          Reference (clean) signal — modified in-place
 * @param sig          Received (noisy) signal — modified in-place, noise added
 * @param n            Number of complex samples
 * @param domain       Signal domain label (e.g., "complex_baseband")
 * @param N_t0_W       T0-referenced noise power per Hz [W]
 * @param N_current    [in/out] Accumulated noise power tracker [W]
 * @param Gain_total   [in/out] Accumulated linear gain product
 * @param P_sig_in_W   Input signal power [W] (for analytic SNR computation)
 * @param rng          Reentrant PRNG state for noise injection
 *
 * @return StageMetric with SNR (analytic Friis) and EVM (measured)
 */
StageMetric apply_stage_complex(const StageModel *stg,
                                Complex *restrict ref,
                                Complex *restrict sig, size_t n,
                                const char *domain, double N_t0_W,
                                double *N_current, double *Gain_total,
                                double P_sig_in_W, PrngState *rng);

/**
 * @brief Process real (RF) samples through one receiver stage.
 *
 * Operates on double arrays (real-valued RF waveform). Used for the RF
 * frontend path before downconversion. Same processing steps as
 * apply_stage_complex but with RF bandwidth correction on noise:
 *   pn_add = N_t0_v2 × g_lin × (F - 1) × (fs / (2 × B_NOISE_HZ))
 *
 * @param stg          Stage model parameters
 * @param ref          Reference RF signal — modified in-place
 * @param sig          Received RF signal — modified in-place, noise added
 * @param n            Number of real samples
 * @param domain       Signal domain label (e.g., "rf_real")
 * @param N_t0_W       T0-referenced noise power [W]
 * @param fs_hz        RF sample rate [Hz] (for bandwidth correction)
 * @param fc_hz        Carrier frequency [Hz] (reserved, unused currently)
 * @param N_current    [in/out] Accumulated noise power tracker [W]
 * @param Gain_total   [in/out] Accumulated linear gain product
 * @param P_sig_in_W   Input signal power [W]
 * @param rng          Reentrant PRNG state for noise injection
 *
 * @return StageMetric with SNR (analytic Friis), EVM = NaN (undefined for real)
 */
StageMetric apply_stage_realistic(const StageModel *stg,
                                  double *restrict ref,
                                  double *restrict sig,
                                  size_t n, const char *domain,
                                  double N_t0_W, double fs_hz,
                                  double fc_hz,
                                  double *N_current, double *Gain_total,
                                  double P_sig_in_W, PrngState *rng);

/**
 * @brief Compute SNR and EVM for complex (I/Q) signals.
 *
 * Compares the degraded signal to the clean reference:
 *   - SNR = 10·log10(P_signal / P_noise)
 *   - EVM = sqrt(P_noise / P_signal) × 100 [%]
 *
 * @param stage  Stage name label
 * @param domain Signal domain label
 * @param ref    Reference (clean) complex signal
 * @param sig    Received (degraded) complex signal
 * @param n      Number of samples
 *
 * @return StageMetric with signal_power, noise_power, snr_db, evm_pct
 */
StageMetric compute_metric_complex(const char *stage, const char *domain,
                                   const Complex *restrict ref,
                                   const Complex *restrict sig, size_t n);

/**
 * @brief Compute SNR for real-valued signals (EVM undefined).
 *
 * Same as compute_metric_complex but for real arrays.
 * EVM is always NaN (only meaningful for I/Q constellation signals).
 *
 * @param stage  Stage name label
 * @param domain Signal domain label
 * @param ref    Reference (clean) real signal
 * @param sig    Received (degraded) real signal
 * @param n      Number of samples
 *
 * @return StageMetric with signal_power, noise_power, snr_db; evm_pct = NaN
 */
StageMetric compute_metric_real(const char *stage, const char *domain,
                                const double *restrict ref,
                                const double *restrict sig, size_t n);

#endif /* SIGNAL_CHAIN_H */
