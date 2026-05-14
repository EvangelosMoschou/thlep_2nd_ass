#include "flicker_noise.h"
#include <math.h>

int flicker_noise_init(FlickerNoiseConfig* cfg) {
    if (!cfg || cfg->sample_rate_hz <= 0.0) return -1;
    cfg->num_octaves = (int)log2(cfg->sample_rate_hz / cfg->corner_freq_hz);
    if (cfg->num_octaves > FLICKER_NOISE_MAX_OCTAVES) cfg->num_octaves = FLICKER_NOISE_MAX_OCTAVES;
    if (cfg->num_octaves < 1) cfg->num_octaves = 1;

    for (int i = 0; i < cfg->num_octaves; i++) {
        cfg->accumulators[i] = 0.0;
        cfg->update_intervals[i] = (uint64_t)pow(2, i);
        cfg->update_counters[i] = 0;
    }
    cfg->output_scale = sqrt(cfg->white_noise_power / (double)cfg->num_octaves);
    cfg->running_sum = 0.0;
    cfg->sample_count = 0;
    return 0;
}

double flicker_noise_generate(FlickerNoiseConfig* cfg, PrngState* prng) {
    for (int i = 0; i < cfg->num_octaves; i++) {
        if (cfg->sample_count % cfg->update_intervals[i] == 0) {
            cfg->running_sum -= cfg->accumulators[i];
            cfg->accumulators[i] = prng_gauss(prng);
            cfg->running_sum += cfg->accumulators[i];
        }
    }
    cfg->sample_count++;
    return cfg->running_sum * cfg->output_scale;
}

void flicker_noise_reset(FlickerNoiseConfig* cfg) {
    flicker_noise_init(cfg);
}

void flicker_noise_free(FlickerNoiseConfig* cfg) {
    (void)cfg;
}
