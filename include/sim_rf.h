/**
 * @file sim_rf.h
 * @brief RF simulation module — brute-force and realistic RF path simulation.
 *
 * This module implements the full RF upconversion/downconversion simulation
 * path, including:
 *   - Pulse shaping (RRC) at RF sample rate
 *   - IQ upconversion to 24 GHz carrier
 *   - RF frontend stage processing (real signal domain)
 *   - Mixer downconversion to complex baseband
 *   - Post-mixer baseband stage processing
 *
 * The "realistic" path adds impairment models:
 *   - TX/RX LO phase noise
 *   - I/Q imbalance (gain + phase error)
 *   - Flicker (1/f) noise
 *   - Biquad Butterworth filters
 *   - ADC quantization + jitter
 *   - LO leakage DC offset
 *
 * All PRNG-dependent functions use reentrant PrngState* (no global state).
 */

#ifndef SIM_RF_H
#define SIM_RF_H

#include <stddef.h>

#include "prng.h"
#include "sim_types.h"
#include "stage_models.h"

/**
 * @brief Maximum number of stage metric entries per RF simulation path.
 */
#define MAX_RF_METRICS 32

/**
 * @brief Run the brute-force RF simulation (no impairment models).
 *
 * Signal flow:
 *   1. Pulse-shape symbols at RF rate (RRC filter)
 *   2. Scale to physical input power level
 *   3. Add antenna thermal noise
 *   4. IQ-modulate onto 24 GHz carrier (upconversion)
 *   5. Record input RF metric
 *   6. Process through rf_frontend stages (real domain)
 *   7. Downconvert RF → complex baseband (mix + lowpass + decimate)
 *   8. Record downconversion metric
 *   9. Process through rf_postmix_bb stages (complex baseband)
 *  10. Record final VPP and metrics
 *
 * @param cfg               Simulation configuration
 * @param stage_cfg         Stage chain configuration (rf_frontend + rf_postmix_bb)
 * @param tx_symbols        Transmitted symbols
 * @param constellation_template  Reference constellation for EVM
 * @param constellation_count     Number of constellation points
 * @param nsym              Number of symbols
 * @param metrics           Output: stage metrics array (caller provides MAX_RF_METRICS)
 * @param metric_count      Output: number of metrics filled
 * @param final_vpp         Output: peak-to-peak voltage of reference
 * @param used_sps          Output: samples per symbol used
 * @param used_fs_hz        Output: actual RF sampling frequency
 * @param csv_dir           Directory for CSV output files
 * @param const_dir         Directory for constellation SVG files
 * @param trace_dir         Directory for trace SVG files
 *
 * @return 0 on success, -1 on memory error, -2 if chains missing,
 *         -3 if MAX_RF_METRICS exceeded
 */
int simulate_bruteforce_rf(
    const SimConfig *cfg, const StageModelsConfig *stage_cfg,
    const Complex *tx_symbols, const Complex *constellation_template,
    size_t constellation_count, size_t nsym, StageMetric *metrics,
    size_t *metric_count, double *final_vpp, int *used_sps, double *used_fs_hz,
    const char *csv_dir, const char *const_dir, const char *trace_dir,
    const char *spectrum_dir);

#include "sim_rf_realistic.h"


/**
 * @brief IQ-modulate a complex envelope onto an RF carrier (upconversion).
 *
 * Computes: rf[i] = env_re[i]*cos(theta) - env_im[i]*sin(theta)
 * where theta = 2*pi*fc_hz/fs_hz * i
 *
 * @param env_re  Envelope real part (input)
 * @param env_im  Envelope imaginary part (input)
 * @param n       Number of samples
 * @param fs_hz   Sampling frequency [Hz]
 * @param fc_hz   Carrier frequency [Hz]
 * @param rf_out  Output real RF signal
 */
void sim_rf_upconvert(const double *env_re, const double *env_im, size_t n,
                      double fs_hz, double fc_hz, double *rf_out);

size_t sim_rf_downconvert(const double *rf, size_t n, double fs_hz,
                          double fc_hz, double cutoff_hz, size_t dec_factor,
                          double *bb_re, double *bb_im,
                          double *i_raw, double *q_raw);

/* --- Helper functions for RF simulation --- */
void add_awgn_soa(double *restrict re, double *restrict im, size_t n,
                  double noise_power);

void scale_soa(double *restrict re, double *restrict im, size_t n,
                double amp);

double mean_power_soa(const double *restrict re,
                       const double *restrict im, size_t n);

double mean_noise_power_soa(const double *restrict sig_re,
                             const double *restrict sig_im,
                             const double *restrict ref_re,
                             const double *restrict ref_im, size_t n);

StageMetric apply_stage_soa(
    const StageModel *stg, double *restrict ref_re, double *restrict ref_im,
    double *restrict sig_re, double *restrict sig_im, size_t n,
    const char *domain, double N_t0_W, double *N_current, double *Gain_total,
    double P_sig_in_W);

StageMetric apply_stage_real_fused(const StageModel *stg,
                                   double *restrict ref, double *restrict sig,
                                   size_t n, const char *domain,
                                   double N_t0_W, double fs_hz,
                                   double fc_hz,
                                   double *N_current, double *Gain_total,
                                   double P_sig_in_W);

double complex_real_vpp(const Complex *x, size_t n);

size_t synchronize_and_downsample(const Complex *bb_stream, size_t n_in,
                                   size_t nsym, int sps, double rolloff,
                                   const Complex *tx_symbols, Complex *symbols_out);

#endif /* SIM_RF_H */
