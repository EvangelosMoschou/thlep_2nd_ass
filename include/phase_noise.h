/**
 * @file phase_noise.h
 * @brief Phase noise generation using time-domain IIR filtering of white noise.
 *
 * Models oscillator phase noise via a three-region PSD:
 *   L(f) = white_floor + K2/f² + K3/f³
 *
 * The implementation uses parallel IIR filter branches driven by white
 * Gaussian noise from the existing PRNG (prng_gauss), ensuring full
 * determinism given the same seed.
 *
 * Typical usage:
 * @code
 *   PhaseNoiseConfig cfg = {0};
 *   cfg.white_floor_dbc_hz = -155.0;
 *   cfg.f2_corner_hz = 100e3;
 *   cfg.f3_corner_hz = 10e3;
 *   cfg.f2_slope_dbc_hz = -100.0;
 *   cfg.f3_slope_dbc_hz = -80.0;
 *   cfg.sample_rate_hz = 96e6;
 *   if (phase_noise_init(&cfg) != 0) { ...handle error... }
 *
 *   double pn = phase_noise_generate();
 *
 *   phase_noise_free(&cfg);
 * @endcode
 */

#ifndef PHASE_NOISE_H
#define PHASE_NOISE_H

#include "prng.h"

/**
 * @brief Configuration for phase noise generation.
 *
 * All dBc/Hz values are in decibels relative to carrier per Hertz.
 * Corner frequencies are in Hz.
 */
typedef struct PhaseNoiseConfig {
    /** White noise floor level in dBc/Hz (e.g., -155 for a good VCO). */
    double white_floor_dbc_hz;

    /** Corner frequency where 1/f² region meets white floor (Hz). */
    double f2_corner_hz;

    /** Corner frequency where 1/f³ region meets 1/f² region (Hz). */
    double f3_corner_hz;

    /** PSD level at f2_corner in dBc/Hz (typically equals white_floor + 3 dB). */
    double f2_slope_dbc_hz;

    /** PSD level at f3_corner in dBc/Hz (typically equals f2_slope + 10 dB). */
    double f3_slope_dbc_hz;

    /** Sample rate in Hz (must match simulation sample rate). */
    double sample_rate_hz;

    /* -- Internal state (filled by phase_noise_init) -- */

    /** White noise gain (linear). */
    double white_gain;

    /** 1/f² section IIR coefficients. */
    double b2, a2;

    /** 1/f³ section IIR coefficients. */
    double b3, a3;

    /** Gains for shaped noise branches (linear, applied after filter). */
    double gain_1f2;
    double gain_1f3;

    /** Filter state variables. */
    double state_f2;
    double state_f3;
} PhaseNoiseConfig;

/**
 * @brief Initialize phase noise generator from PSD specification.
 *
 * Precomputes IIR filter coefficients using bilinear transform from
 * the three-region PSD model. Must be called before phase_noise_generate().
 *
 * @param cfg Configuration struct (must have sample_rate_hz set).
 * @return 0 on success, -1 on invalid parameters.
 */
int phase_noise_init(PhaseNoiseConfig* cfg);

/**
 * @brief Generate one sample of phase noise.
 *
 * Drives the precomputed IIR filter structure with white Gaussian
 * noise from prng_gauss() and returns the resulting phase deviation
 * in radians RMS.
 *
 * @param cfg Initialized configuration (state fields are updated).
 * @param prng PRNG state for Gaussian white noise generation.
 * @return Phase noise sample in radians.
 */
double phase_noise_generate(PhaseNoiseConfig* cfg, PrngState* prng);

/**
 * @brief Release resources held by the phase noise generator.
 *
 * Resets internal state to zero. Safe to call multiple times.
 *
 * @param cfg Configuration to free.
 */
void phase_noise_free(PhaseNoiseConfig* cfg);

#endif
