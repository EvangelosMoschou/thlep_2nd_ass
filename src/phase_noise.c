/*
 * ============================================================================
 * phase_noise.c — Phase Noise Generation via Time-Domain IIR Filtering
 * ============================================================================
 *
 * PURPOSE:
 *   Generates oscillator phase noise by shaping white Gaussian noise through
 *   parallel IIR filter branches that reproduce the three-region PSD model:
 *     L(f) = white_floor + K2/f² + K3/f³
 *
 * DESIGN:
 *   Three parallel paths driven by prng_gauss():
 *     1. White floor: direct scaled white noise
 *     2. 1/f² region: first-order IIR low-pass (corner = f2)
 *     3. 1/f³ region: first-order IIR low-pass (corner = f3)
 *
 *   Coefficients computed via bilinear transform from analog prototypes.
 *   Full determinism is guaranteed through the existing xoshiro256** PRNG.
 *
 * DEFAULT (24 GHz VCO):
 *   white_floor = -155 dBc/Hz, f2_corner = 100 kHz, f3_corner = 10 kHz
 *
 * ============================================================================
 */

#include "phase_noise.h"
#include "math_utils.h"
#include "prng.h"

#include <stdio.h>

int phase_noise_init(PhaseNoiseConfig* cfg) {
    if (!cfg || cfg->sample_rate_hz <= 0.0) {
        return -1;
    }
    if (cfg->f2_corner_hz <= 0.0 || cfg->f3_corner_hz <= 0.0) {
        return -1;
    }
    if (cfg->f2_corner_hz < cfg->f3_corner_hz) {
        fprintf(stderr,
            "phase_noise_init: f2_corner (%.0f Hz) must be >= f3_corner (%.0f Hz)\n",
            cfg->f2_corner_hz, cfg->f3_corner_hz);
        return -1;
    }

    double fs = cfg->sample_rate_hz;
    double dt = 1.0 / fs;

    /* White noise gain: sqrt(S_white * fs) where S_white = 10^(floor/10) */
    double s_white = db_to_lin_power(cfg->white_floor_dbc_hz);
    cfg->white_gain = sqrt(s_white * fs);

    /* 1/f² section: first-order IIR low-pass with corner at f2 */
    double wc2 = 2.0 * M_PI * cfg->f2_corner_hz;
    double alpha2 = 2.0 / (2.0 + wc2 * dt);
    cfg->b2 = 1.0 - alpha2;
    cfg->a2 = alpha2;

    /* 1/f³ section: first-order IIR low-pass with corner at f3 */
    double wc3 = 2.0 * M_PI * cfg->f3_corner_hz;
    double alpha3 = 2.0 / (2.0 + wc3 * dt);
    cfg->b3 = 1.0 - alpha3;
    cfg->a3 = alpha3;

    /* Branch gains from slope PSD levels */
    double s_f2 = db_to_lin_power(cfg->f2_slope_dbc_hz);
    double s_f3 = db_to_lin_power(cfg->f3_slope_dbc_hz);
    cfg->gain_1f2 = sqrt(s_f2 * cfg->f2_corner_hz);
    cfg->gain_1f3 = sqrt(s_f3 * cfg->f3_corner_hz);

    /* Reset filter state */
    cfg->state_f2 = 0.0;
    cfg->state_f3 = 0.0;

    return 0;
}

double phase_noise_generate(PhaseNoiseConfig* cfg, PrngState* prng) {
    double w = prng_gauss(prng);

    double white_out = cfg->white_gain * w;

    double f2_in = cfg->gain_1f2 * w;
    double f2_out = cfg->b2 * f2_in + cfg->a2 * cfg->state_f2;
    cfg->state_f2 = f2_out;

    double f3_in = cfg->gain_1f3 * w;
    double f3_out = cfg->b3 * f3_in + cfg->a3 * cfg->state_f3;
    cfg->state_f3 = f3_out;

    return white_out + f2_out + f3_out;
}

void phase_noise_free(PhaseNoiseConfig* cfg) {
    if (cfg) {
        cfg->state_f2 = 0.0;
        cfg->state_f3 = 0.0;
        cfg->white_gain = 0.0;
        cfg->gain_1f2 = 0.0;
        cfg->gain_1f3 = 0.0;
    }
}
