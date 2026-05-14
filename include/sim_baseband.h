#ifndef SIM_BASEBAND_H
#define SIM_BASEBAND_H

#include "sim_types.h"
#include "stage_models.h"

typedef struct {
    StageMetric metrics[MAX_METRICS];
    size_t count;
    double final_vpp;
} SimBasebandResult;

int simulate_baseband(const SimConfig* cfg, const StageModelsConfig* stage_cfg, const Complex* constellation, size_t n_const, const Complex* tx_symbols, size_t n_syms, PrngState* rng, SimBasebandResult* out, const char* csv_dir, const char* const_dir, const char* trace_dir);

#endif
