#define _USE_MATH_DEFINES
#include "iq_imbalance.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double db_to_lin(double x_db) { return pow(10.0, x_db / 20.0); }

void iq_imbalance_apply(Complex *s, const IQImbalanceConfig *cfg) {
    if (!s || !cfg) return;

    double gain_lin = db_to_lin(-cfg->gain_error_db);
    double phase_rad = cfg->phase_error_deg * M_PI / 180.0;
    double cos_p = cos(phase_rad);
    double sin_p = sin(phase_rad);

    double q_out = s->im * gain_lin * cos_p - s->re * sin_p;

    s->im = q_out;
}

void iq_imbalance_apply_real(double *i, double *q, const IQImbalanceConfig *cfg) {
    if (!i || !q || !cfg) return;

    double gain_lin = db_to_lin(-cfg->gain_error_db);
    double phase_rad = cfg->phase_error_deg * M_PI / 180.0;
    double cos_p = cos(phase_rad);
    double sin_p = sin(phase_rad);

    double q_out = (*q) * gain_lin * cos_p - (*i) * sin_p;

    *q = q_out;
}
