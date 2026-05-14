#include "phase_noise.h"
#include <math.h>

int phase_noise_init(PhaseNoiseConfig* cfg) {
    if (!cfg || cfg->sample_rate_hz <= 0.0) return -1;
    cfg->white_gain = sqrt(pow(10.0, cfg->white_floor_dbc_hz / 10.0) * cfg->sample_rate_hz);
    
    /* Simplified IIR for 1/f^2 and 1/f^3 */
    double f2 = cfg->f2_corner_hz / cfg->sample_rate_hz;
    cfg->a2 = exp(-2.0 * M_PI * f2);
    cfg->b2 = 1.0 - cfg->a2;
    cfg->gain_1f2 = sqrt(pow(10.0, cfg->f2_slope_dbc_hz / 10.0) / (f2*f2));

    double f3 = cfg->f3_corner_hz / cfg->sample_rate_hz;
    cfg->a3 = exp(-2.0 * M_PI * f3);
    cfg->b3 = 1.0 - cfg->a3;
    cfg->gain_1f3 = sqrt(pow(10.0, cfg->f3_slope_dbc_hz / 10.0) / (f3*f3*f3));

    cfg->state_f2 = 0.0;
    cfg->state_f3 = 0.0;
    return 0;
}

double phase_noise_generate(PhaseNoiseConfig* cfg, PrngState* prng) {
    double w = prng_gauss(prng);
    cfg->state_f2 = cfg->a2 * cfg->state_f2 + cfg->b2 * w;
    cfg->state_f3 = cfg->a3 * cfg->state_f3 + cfg->b3 * cfg->state_f2;
    
    double out = cfg->white_gain * w + cfg->gain_1f2 * cfg->state_f2 + cfg->gain_1f3 * cfg->state_f3;
    return out; /* Output in radians RMS */
}

void phase_noise_free(PhaseNoiseConfig* cfg) {
    (void)cfg;
}
