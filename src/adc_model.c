/*
 * ============================================================================
 * adc_model.c — ADC Quantization and Clock Jitter Noise Models
 * ============================================================================
 *
 * PURPOSE:
 *   Models two dominant ADC non-idealities:
 *     1. Quantization noise from finite bit-depth
 *     2. Sampling clock jitter causing voltage uncertainty
 *
 * QUANTIZATION:
 *   y = round(x * (2^N - 1) / V_fs) * V_fs / (2^N - 1)
 *
 *   where N = bit_depth, V_fs = full_scale_vpp
 *   The input x is assumed to be in the range [-V_fs/2, +V_fs/2].
 *
 * JITTER NOISE:
 *   SNR_jitter = -20*log10(2*pi*f_in*sigma_jitter)
 *
 *   The jitter-induced noise power is modeled as Gaussian voltage noise:
 *     sigma_v = 2*pi*f_in * A * sigma_jitter
 *
 *   where A = full_scale_vpp / 2 (full-scale sinusoidal amplitude).
 *   The noise sample is generated via prng_gauss() for determinism.
 *
 * DEFAULT (12-bit ADC):
 *   bit_depth = 12, full_scale_vpp = 1.0 V, jitter = 1.0 ps
 *
 * ============================================================================
 */

#include "adc_model.h"
#include "prng.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int adc_model_init(ADCModelConfig* cfg) {
    if (!cfg) {
        return -1;
    }
    if (cfg->bit_depth < 12 || cfg->bit_depth > 16) {
        fprintf(stderr,
            "adc_model_init: bit_depth %d out of range [12, 16]\n",
            cfg->bit_depth);
        return -1;
    }
    if (cfg->full_scale_vpp <= 0.0) {
        fprintf(stderr,
            "adc_model_init: full_scale_vpp must be > 0 (got %f)\n",
            cfg->full_scale_vpp);
        return -1;
    }
    if (cfg->jitter_ps < 0.0) {
        fprintf(stderr,
            "adc_model_init: jitter_ps must be >= 0 (got %f)\n",
            cfg->jitter_ps);
        return -1;
    }

    /* Set defaults for unspecified fields */
    if (cfg->bit_depth == 0) cfg->bit_depth = 12;
    if (cfg->full_scale_vpp == 0.0) cfg->full_scale_vpp = 1.0;
    if (cfg->jitter_ps == 0.0) cfg->jitter_ps = 1.0;

    cfg->levels = pow(2.0, cfg->bit_depth) - 1.0;
    cfg->step_v = cfg->full_scale_vpp / cfg->levels;
    cfg->jitter_s = cfg->jitter_ps * 1e-12;

    return 0;
}

void adc_model_apply(double* sample, double f_in_hz, const ADCModelConfig* cfg) {
    if (!sample || !cfg || cfg->levels <= 0.0) {
        return;
    }

    /* Quantization: round to nearest level */
    double normalized = *sample / cfg->full_scale_vpp;
    double quantized = round(normalized * cfg->levels) / cfg->levels;
    *sample = quantized * cfg->full_scale_vpp;

    /* Jitter noise: skip if no jitter configured */
    if (cfg->jitter_s <= 0.0 || f_in_hz <= 0.0) {
        return;
    }

    /* sigma_v = 2*pi*f_in * A * sigma_jitter, A = V_fs/2 */
    double amplitude = cfg->full_scale_vpp * 0.5;
    double sigma_v = 2.0 * M_PI * f_in_hz * amplitude * cfg->jitter_s;

    *sample += prng_gauss() * sigma_v;
}

void adc_model_free(ADCModelConfig* cfg) {
    (void)cfg;
}
