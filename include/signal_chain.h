#ifndef SIGNAL_CHAIN_H
#define SIGNAL_CHAIN_H

#include "sim_types.h"
#include "stage_models.h"

void apply_stage_soa(const StageModel* stg, double* ref_re, double* ref_im, double* sig_re, double* sig_im, size_t n, const char* domain, double N_t0_W, double* N_current, double* Gain_total, double P_sig_in);
StageMetric apply_stage_real_fused(const StageModel* stg, double* ref, double* sig, size_t n, const char* domain, double N_t0_W, double fs_hz, double fc_hz, double* N_current, double* Gain_total, double P_sig_in);

#endif
