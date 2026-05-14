/**
 * @file flicker_noise.h
 * @brief Flicker (1/f) noise generation using Voss-McCartney algorithm.
 *
 * Models 1/f noise (pink noise) via a stochastic summation of random walks
 * at multiple octave-spaced time scales, producing a power spectral density
 * proportional to 1/f.
 */

#ifndef FLICKER_NOISE_H
#define FLICKER_NOISE_H

#include <stdint.h>
#include "prng.h"

#define FLICKER_NOISE_MAX_OCTAVES 32

typedef struct FlickerNoiseConfig {
    double corner_freq_hz;
    double white_noise_power;
    double sample_rate_hz;
    int num_octaves;
    double accumulators[FLICKER_NOISE_MAX_OCTAVES];
    uint64_t update_counters[FLICKER_NOISE_MAX_OCTAVES];
    uint64_t update_intervals[FLICKER_NOISE_MAX_OCTAVES];
    double output_scale;
    double running_sum;
    uint64_t sample_count;
} FlickerNoiseConfig;

int flicker_noise_init(FlickerNoiseConfig* cfg);
double flicker_noise_generate(FlickerNoiseConfig* cfg, PrngState* prng);
void flicker_noise_reset(FlickerNoiseConfig* cfg);
void flicker_noise_free(FlickerNoiseConfig* cfg);

#endif
