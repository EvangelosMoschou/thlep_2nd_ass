#ifndef SIM_RF_H
#define SIM_RF_H

#include "sim_types.h"
#include "stage_models.h"

int simulate_bruteforce_rf(const SimConfig* cfg, const StageModelsConfig* stage_cfg, const Complex* tx_symbols, const Complex* constellation, size_t n_const, size_t n_syms, StageMetric* metrics, size_t* metric_count, double* final_vpp, int* used_sps, double* used_fs_hz, const char* csv_dir, const char* const_dir, const char* trace_dir);

#endif
