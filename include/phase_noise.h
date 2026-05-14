/**
 * @file phase_noise.h
 * @brief Phase noise generation.
 */

#ifndef PHASE_NOISE_H
#define PHASE_NOISE_H

#include "prng.h"

typedef struct PhaseNoiseConfig {
    double white_floor_dbc_hz;
    double f2_corner_hz;
    double f3_corner_hz;
    double f2_slope_dbc_hz;
    double f3_slope_dbc_hz;
    double sample_rate_hz;
    double white_gain;
    double b2, a2;
    double b3, a3;
    double gain_1f2;
    double gain_1f3;
    double state_f2;
    double state_f3;
} PhaseNoiseConfig;

int phase_noise_init(PhaseNoiseConfig* cfg);
double phase_noise_generate(PhaseNoiseConfig* cfg, PrngState* prng);
void phase_noise_free(PhaseNoiseConfig* cfg);

#endif
