#ifndef ADC_MODEL_H
#define ADC_MODEL_H

#include "sim_types.h"

typedef struct {
    int bit_depth;
    double full_scale_vpp;
    double jitter_ps;
} AdcConfig;

int adc_model_init(AdcConfig* cfg);
void adc_model_apply(double* sample, double f_in_hz, const AdcConfig* cfg, PrngState* rng);
void adc_model_free(AdcConfig* cfg);

#endif
