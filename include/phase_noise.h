#ifndef PHASE_NOISE_H
#define PHASE_NOISE_H

#include "sim_types.h"

typedef struct {
    double white_floor_dbc_hz;
    double f2_corner_hz;
    double f3_corner_hz;
    double f2_slope_dbc_hz;
    double f3_slope_dbc_hz;
    double sample_rate_hz;
    double* coeffs;
    double* state;
    size_t order;
} PhaseNoiseConfig;

int phase_noise_init(PhaseNoiseConfig* cfg);
double phase_noise_generate(PhaseNoiseConfig* cfg, PrngState* rng);
void phase_noise_free(PhaseNoiseConfig* cfg);

#endif
