#ifndef FLICKER_NOISE_H
#define FLICKER_NOISE_H

#include "sim_types.h"

typedef struct {
    double corner_freq_hz;
    double white_noise_power;
    double sample_rate_hz;
    double* filter_coeffs;
    double* state;
    size_t order;
} FlickerNoiseConfig;

int flicker_noise_init(FlickerNoiseConfig* cfg);
double flicker_noise_generate(FlickerNoiseConfig* cfg, PrngState* rng);
void flicker_noise_free(FlickerNoiseConfig* cfg);

#endif
