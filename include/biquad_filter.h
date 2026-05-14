/**
 * @file biquad_filter.h
 * @brief Digital biquad (Direct Form II) filter implementation.
 *
 * Used for implementing higher-order filters (like Butterworth) as a
 * cascade of second-order sections (SOS).
 */

#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

/**
 * @brief Biquad filter coefficients (Direct Form II).
 * H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 */
typedef struct BiquadCoeffs {
    double b0, b1, b2;
    double a1, a2;
} BiquadCoeffs;

/**
 * @brief Biquad filter state (delay elements).
 */
typedef struct BiquadState {
    double z1, z2;
} BiquadState;

/**
 * @brief Process one sample through a biquad stage.
 */
double biquad_process(double x, const BiquadCoeffs *c, BiquadState *s);

/**
 * @brief Compute Butterworth SOS coefficients for a given order and cutoff.
 */
int design_butterworth_sos(int order, double fc_fs, BiquadCoeffs *out_sos);

#endif
