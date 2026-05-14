#ifndef IQ_IMBALANCE_H
#define IQ_IMBALANCE_H

typedef struct {
    double gain_error_db;
    double phase_error_deg;
} IqImbalanceConfig;

void iq_imbalance_apply_real(double* i, double* q, const IqImbalanceConfig* cfg);

#endif
