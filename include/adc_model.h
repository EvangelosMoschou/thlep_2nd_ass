/**
 * @file adc_model.h
 * @brief ADC quantization and clock jitter noise models.
 *
 * Models two dominant ADC non-idealities:
 *   1. Quantization noise: finite bit-depth discretization
 *   2. Sampling jitter: timing uncertainty on the sample clock
 *
 * The quantization model rounds each sample to the nearest quantization level.
 * The jitter model adds Gaussian voltage noise derived from:
 *   SNR_jitter = -20*log10(2*pi*f_in*sigma_jitter)
 *
 * Typical usage:
 * @code
 *   ADCModelConfig cfg = {0};
 *   cfg.bit_depth = 12;
 *   cfg.full_scale_vpp = 1.0;
 *   cfg.jitter_ps = 1.0;
 *   adc_model_init(&cfg);
 *
 *   double sample = 0.35;
 *   double f_in = 10e6;  // 10 MHz input tone
 *   adc_model_apply(&sample, f_in, &cfg);
 *
 *   adc_model_free(&cfg);
 * @endcode
 */

#ifndef ADC_MODEL_H
#define ADC_MODEL_H

#include <stdint.h>

#include "prng.h"

/**
 * @brief Configuration for ADC quantization and jitter noise model.
 *
 * All voltages are peak-to-peak. Jitter is in picoseconds.
 */
typedef struct ADCModelConfig {
    /** ADC resolution in bits (12..16). Default: 12. */
    int bit_depth;

    /** Full-scale peak-to-peak voltage in volts. Default: 1.0 V. */
    double full_scale_vpp;

    /** Sampling clock jitter in picoseconds (rms). Default: 1.0 ps. */
    double jitter_ps;

    /* -- Internal state (filled by adc_model_init) -- */

    /** Number of quantization levels: 2^bit_depth - 1. */
    double levels;

    /** Volts per quantization step. */
    double step_v;

    /** Jitter rms in seconds (converted from jitter_ps). */
    double jitter_s;
} ADCModelConfig;

/**
 * @brief Initialize ADC model from specification parameters.
 *
 * Validates bit_depth range (12..16) and precomputes quantization levels,
 * step size, and jitter in seconds.
 *
 * @param cfg Configuration struct (bit_depth, full_scale_vpp, jitter_ps must be set).
 * @return 0 on success, -1 on invalid parameters.
 */
int adc_model_init(ADCModelConfig* cfg);

/**
 * @brief Apply ADC quantization and jitter noise to a single sample.
 *
 * Quantization is applied first (deterministic rounding), then jitter
 * noise is added as a Gaussian voltage perturbation whose power depends
 * on the input signal frequency and clock jitter.
 *
 * The jitter-induced noise voltage is:
 *   v_jitter = dV/dt * t_jitter ≈ 2*pi*f_in * A * sigma_jitter
 *
 * where A is the signal amplitude (half of full_scale_vpp for a full-scale tone).
 *
 * @param sample    Pointer to the sample value (modified in place).
 * @param f_in_hz   Input signal frequency in Hz (used for jitter noise).
 * @param cfg       Initialized ADC model configuration.
 * @param prng      PRNG state for Gaussian jitter noise generation.
 */
void adc_model_apply(double* sample, double f_in_hz, const ADCModelConfig* cfg, PrngState* prng);

/**
 * @brief Release any internal resources held by the ADC model.
 *
 * Currently a no-op (all state is in the config struct), but provided
 * for API symmetry with other modules.
 *
 * @param cfg Configuration struct to free.
 */
void adc_model_free(ADCModelConfig* cfg);

#endif
