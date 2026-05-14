#include "iq_imbalance.h"
#include <math.h>

void iq_imbalance_apply(Complex *s, const IQImbalanceConfig *cfg) {
    double g_error_lin = pow(10.0, -cfg->gain_error_db / 20.0);
    double phi_rad = cfg->phase_error_deg * M_PI / 180.0;
    
    double i = s->re;
    double q = s->im;
    
    s->im = q * g_error_lin * cos(phi_rad) - i * sin(phi_rad);
}

void iq_imbalance_apply_real(double *i, double *q, const IQImbalanceConfig *cfg) {
    double g_error_lin = pow(10.0, -cfg->gain_error_db / 20.0);
    double phi_rad = cfg->phase_error_deg * M_PI / 180.0;
    
    *q = (*q) * g_error_lin * cos(phi_rad) - (*i) * sin(phi_rad);
}
