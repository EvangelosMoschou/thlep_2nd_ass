/**
 * @file iq_imbalance.h
 * @brief I/Q imbalance model for gain and phase errors.
 */

#ifndef IQ_IMBALANCE_H
#define IQ_IMBALANCE_H

#include "sim_types.h"

typedef struct IQImbalanceConfig {
    double gain_error_db;
    double phase_error_deg;
} IQImbalanceConfig;

void iq_imbalance_apply(Complex *s, const IQImbalanceConfig *cfg);
void iq_imbalance_apply_real(double *i, double *q, const IQImbalanceConfig *cfg);

#endif
