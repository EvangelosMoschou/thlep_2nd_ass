#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

#include <stddef.h>

#include "sim_types.h"

/*
 * Maximum number of biquad sections supported.
 * Order 6 requires 3 cascaded sections.
 */
#define BIQUAD_MAX_SECTIONS 3

/*
 * BiquadConfig: Configuration parameters for a Butterworth biquad IIR filter.
 *
 *   cutoff_hz  - Cutoff frequency in Hz (-3 dB point)
 *   fs_hz      - Sample rate in Hz
 *   order      - Filter order: 2 (1 section), 4 (2 sections), or 6 (3 sections)
 */
typedef struct BiquadConfig {
    double cutoff_hz;
    double fs_hz;
    int    order;
} BiquadConfig;

/*
 * BiquadCoeffs: Normalized biquad coefficients for one section.
 *
 *   Filter equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
 *                           - a1*y[n-1] - a2*y[n-2]
 *   (a0 is normalized to 1.0)
 */
typedef struct BiquadCoeffs {
    double b0, b1, b2;
    double a1, a2;
} BiquadCoeffs;

/*
 * BiquadSectionState: Delay-line state for one biquad section.
 *
 *   x1, x2 - Previous two input samples
 *   y1, y2 - Previous two output samples
 */
typedef struct BiquadSectionState {
    double x1, x2;
    double y1, y2;
} BiquadSectionState;

/*
 * BiquadState: Complete filter state for a cascaded biquad filter.
 *
 *   coeffs   - Coefficients for each section
 *   sections - State for each section
 *   n_sections - Number of active sections (1, 2, or 3)
 */
typedef struct BiquadState {
    BiquadCoeffs      coeffs[BIQUAD_MAX_SECTIONS];
    BiquadSectionState sections[BIQUAD_MAX_SECTIONS];
    int                n_sections;
} BiquadState;

/*
 * biquad_init: Compute Butterworth lowpass coefficients from config.
 *
 *   Uses RBJ Audio EQ Cookbook formulas for each cascaded section.
 *   Q values are chosen to produce a maximally flat Butterworth response.
 *
 *   Returns 0 on success, -1 on invalid config.
 */
int biquad_init(BiquadState *state, const BiquadConfig *cfg);

/*
 * biquad_reset: Clear all delay-line state (useful between signal bursts).
 */
void biquad_reset(BiquadState *state);

/*
 * biquad_process_real: Apply biquad filter to a real-valued signal.
 *
 *   in, out - Input and output buffers (may alias for in-place filtering)
 *   n       - Number of samples
 */
void biquad_process_real(BiquadState *state, const double *in, double *out, size_t n);

/*
 * biquad_process_complex: Apply biquad filter to a complex (I/Q) signal.
 *
 *   I and Q components are filtered independently using the same coefficients.
 *   in, out - Input and output buffers (may alias for in-place filtering)
 *   n       - Number of samples
 */
void biquad_process_complex(BiquadState *state, const Complex *in, Complex *out, size_t n);

/*
 * biquad_process_soa: Apply biquad filter to a Structure-of-Arrays complex signal.
 *
 *   Filters I and Q arrays independently.
 *   in_re, in_im - Input I and Q arrays
 *   out_re, out_im - Output I and Q arrays
 *   n            - Number of samples
 */
void biquad_process_soa(BiquadState *state,
                        const double *in_re, const double *in_im,
                        double *out_re, double *out_im,
                        size_t n);

#endif
