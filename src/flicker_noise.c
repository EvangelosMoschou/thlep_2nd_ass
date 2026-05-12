/*
 * ============================================================================
 * flicker_noise.c — Flicker (1/f) Noise Generation via Voss-McCartney Algorithm
 * ============================================================================
 *
 * PURPOSE:
 *   Generates 1/f (pink/flicker) noise by summing multiple random-walk
 *   processes updated at octave-spaced intervals. The resulting power
 *   spectral density is proportional to 1/f over the configured bandwidth.
 *
 * DESIGN:
 *   Voss-McCartney algorithm:
 *     - Each octave k has an accumulator updated every 2^k samples
 *     - On update, the accumulator receives a fresh Gaussian random value
 *     - The output is the sum of all accumulators, properly scaled
 *     - This produces a 1/f spectrum from the lowest octave rate to fs/2
 *
 *   The number of octaves is determined by:
 *     N = floor(log2(fs / corner_freq))
 *   ensuring the lowest update rate matches the corner frequency.
 *
 * DETERMINISM:
 *   Uses prng_gauss() from the existing PRNG module, ensuring full
 *   reproducibility given the same seed.
 *
 * DEFAULT:
 *   corner_freq = 1 kHz (typical for LNA baseband flicker noise)
 *
 * ============================================================================
 */

#include "flicker_noise.h"
#include "math_utils.h"
#include "prng.h"

#include <stdio.h>
#include <string.h>

int flicker_noise_init(FlickerNoiseConfig* cfg) {
    if (!cfg) {
        return -1;
    }
    if (cfg->corner_freq_hz <= 0.0) {
        fprintf(stderr,
            "flicker_noise_init: corner_freq_hz must be > 0\n");
        return -1;
    }
    if (cfg->sample_rate_hz <= 0.0) {
        fprintf(stderr,
            "flicker_noise_init: sample_rate_hz must be > 0\n");
        return -1;
    }
    if (cfg->corner_freq_hz > cfg->sample_rate_hz * 0.5) {
        fprintf(stderr,
            "flicker_noise_init: corner_freq_hz (%.0f) must be <= Nyquist (%.0f)\n",
            cfg->corner_freq_hz, cfg->sample_rate_hz * 0.5);
        return -1;
    }

    /* Number of octaves: how many doublings from corner_freq to fs/2 */
    double ratio = cfg->sample_rate_hz * 0.5 / cfg->corner_freq_hz;
    int n_octaves = (int)(log(ratio) / M_LN2);

    if (n_octaves < 1) {
        n_octaves = 1;
    }
    if (n_octaves > FLICKER_NOISE_MAX_OCTAVES) {
        n_octaves = FLICKER_NOISE_MAX_OCTAVES;
    }

    cfg->num_octaves = n_octaves;

    /* Set update intervals: octave k updates every 2^k samples */
    for (int k = 0; k < n_octaves; k++) {
        cfg->update_intervals[k] = (uint64_t)1 << (k + 1);
        cfg->update_counters[k] = 0;
        cfg->accumulators[k] = 0.0;
    }

    cfg->sample_count = 0;
    cfg->running_sum = 0.0;

    /*
     * Output scaling:
     * The variance of the sum of n_octaves independent uniform[-1,1]
     * random walks is n_octaves / 3. We scale so that the output
     * power matches the specified white_noise_power at the corner.
     */
    cfg->output_scale = sqrt(cfg->white_noise_power / (double)n_octaves);

    return 0;
}

double flicker_noise_generate(FlickerNoiseConfig* cfg, PrngState* prng) {
    cfg->sample_count++;

    for (int k = 0; k < cfg->num_octaves; k++) {
        cfg->update_counters[k]++;

        if (cfg->update_counters[k] >= cfg->update_intervals[k]) {
            cfg->running_sum -= cfg->accumulators[k];
            cfg->accumulators[k] = prng_gauss(prng);
            cfg->running_sum += cfg->accumulators[k];

            cfg->update_counters[k] = 0;
        }
    }

    return cfg->running_sum * cfg->output_scale;
}

void flicker_noise_reset(FlickerNoiseConfig* cfg) {
    if (!cfg) {
        return;
    }

    for (int k = 0; k < cfg->num_octaves; k++) {
        cfg->accumulators[k] = 0.0;
        cfg->update_counters[k] = 0;
    }

    cfg->sample_count = 0;
    cfg->running_sum = 0.0;
}

void flicker_noise_free(FlickerNoiseConfig* cfg) {
    (void)cfg;
    /* No dynamic allocation — all state is embedded in the struct. */
}
