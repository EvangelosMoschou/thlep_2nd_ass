/**
 * @file flicker_noise.h
 * @brief Flicker (1/f) noise generation using Voss-McCartney algorithm.
 *
 * Models 1/f noise (pink noise) via a stochastic summation of random walks
 * at multiple octave-spaced time scales, producing a power spectral density
 * proportional to 1/f.
 *
 * Typical usage:
 * @code
 *   FlickerNoiseConfig cfg = {0};
 *   cfg.corner_freq_hz = 1000.0;    // 1 kHz corner frequency
 *   cfg.white_noise_power = 1.0;    // white noise power level
 *   cfg.sample_rate_hz = 96e6;      // simulation sample rate
 *   flicker_noise_init(&cfg);
 *
 *   double fn = flicker_noise_generate(&cfg);
 *
 *   flicker_noise_free(&cfg);
 * @endcode
 */

#ifndef FLICKER_NOISE_H
#define FLICKER_NOISE_H

#include <stdint.h>

#include "prng.h"

/** Maximum number of octaves for the Voss-McCartney algorithm. */
#define FLICKER_NOISE_MAX_OCTAVES 32

/**
 * @brief Configuration for flicker (1/f) noise generation.
 *
 * The Voss-McCartney algorithm maintains multiple accumulators, each updated
 * at a different octave rate. The sum produces a 1/f power spectrum.
 */
typedef struct FlickerNoiseConfig {
    /** Corner frequency where 1/f noise equals white noise power (Hz). */
    double corner_freq_hz;

    /** White noise power spectral density level (linear, not dB). */
    double white_noise_power;

    /** Sample rate in Hz (must match simulation sample rate). */
    double sample_rate_hz;

    /* -- Internal state (filled by flicker_noise_init) -- */

    /** Number of octaves computed from sample_rate and corner_freq. */
    int num_octaves;

    /** Per-octave accumulator state variables. */
    double accumulators[FLICKER_NOISE_MAX_OCTAVES];

    /** Per-octave update counters (how many samples since last update). */
    uint64_t update_counters[FLICKER_NOISE_MAX_OCTAVES];

    /** Per-octave update intervals (in samples). */
    uint64_t update_intervals[FLICKER_NOISE_MAX_OCTAVES];

    /** Scaling factor applied to the summed output. */
    double output_scale;

    /** Running sum of all accumulator values (for incremental update). */
    double running_sum;

    /** Global sample counter. */
    uint64_t sample_count;
} FlickerNoiseConfig;

/**
 * @brief Initialize flicker noise generator.
 *
 * Computes the number of octaves needed and sets up update intervals
 * based on the corner frequency and sample rate.
 *
 * @param cfg Configuration struct (must have corner_freq_hz, sample_rate_hz set).
 * @return 0 on success, -1 on invalid parameters.
 */
int flicker_noise_init(FlickerNoiseConfig* cfg);

/**
 * @brief Generate one sample of 1/f noise.
 *
 * Updates the appropriate octave accumulators based on the current
 * sample counter and returns the summed output scaled correctly.
 *
 * @param cfg Initialized configuration struct (state is mutated).
 * @param prng PRNG state for Gaussian random number generation.
 * @return One sample of 1/f noise (mean=0).
 */
double flicker_noise_generate(FlickerNoiseConfig* cfg, PrngState* prng);

/**
 * @brief Reset the internal state of the flicker noise generator.
 *
 * Clears all accumulators and counters so generation restarts
 * from a clean state. Useful for re-running simulations.
 *
 * @param cfg Configuration struct to reset.
 */
void flicker_noise_reset(FlickerNoiseConfig* cfg);

/**
 * @brief Free any dynamically allocated resources.
 *
 * Currently a no-op since all state is embedded in the struct,
 * but provided for API consistency with other noise modules.
 *
 * @param cfg Configuration struct to free.
 */
void flicker_noise_free(FlickerNoiseConfig* cfg);

#endif
