#include "biquad_filter.h"
#include <math.h>

double biquad_process(double x, const BiquadCoeffs *c, BiquadState *s) {
    /* Direct Form II */
    double w = x - c->a1 * s->z1 - c->a2 * s->z2;
    double y = c->b0 * w + c->b1 * s->z1 + c->b2 * s->z2;
    s->z2 = s->z1;
    s->z1 = w;
    return y;
}

int design_butterworth_sos(int order, double fc_fs, BiquadCoeffs *out_sos) {
    /* Simplified Butterworth SOS design for 2nd order */
    if (order != 2) return -1;
    
    double theta = M_PI * fc_fs;
    double sn = sin(theta);
    double cs = cos(theta);
    double alpha = sn / sqrt(2.0); /* Q = 1/sqrt(2) for Butterworth */

    double a0 = 1.0 + alpha;
    out_sos->b0 = (1.0 - cs) / 2.0 / a0;
    out_sos->b1 = (1.0 - cs) / a0;
    out_sos->b2 = (1.0 - cs) / 2.0 / a0;
    out_sos->a1 = -2.0 * cs / a0;
    out_sos->a2 = (1.0 - alpha) / a0;

    return 1; /* 1 SOS section */
}
