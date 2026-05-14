/**
 * @file adc_model.h
 * @brief Analog-to-Digital Converter (ADC) impairment model.
 *
 * Models quantization noise and aperture jitter for a real-valued signal.
 * The quantization is assumed to be uniform over a full-scale range.
 * Aperture jitter is modeled as time-domain phase noise in the sampling clock.
 */

#ifndef ADC_MODEL_H
#define ADC_MODEL_H

#include "prng.h"

/**
 * @brief Configuration for the ADC model.
 */
typedef struct ADCConfig {
    /** ADC resolution in bits (e.g., 12, 14, 16). 0 = bypass quantization. */
    int bits;

    /** Full-scale peak voltage range (+/- Vfs). */
    double v_fs;

    /** RMS aperture jitter in seconds (e.g., 100e-15 for 100 fs). */
    double jitter_rms_sec;

    /** Sample rate in Hz (used to compute phase error from jitter). */
    double fs_hz;
} ADCConfig;

/**
 * @brief Apply ADC impairments to a single real-valued sample.
 *
 * @param val   Input signal sample (voltage).
 * @param cfg   ADC configuration.
 * @param prng  PRNG state for jitter generation.
 * @param time_sec  Current simulation time (used for jitter phase).
 * @return Quantized and jitter-impaired sample.
 */
double adc_apply(double val, const ADCConfig *cfg, PrngState *prng, double time_sec);

#endif
