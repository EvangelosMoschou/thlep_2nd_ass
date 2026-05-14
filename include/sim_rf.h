#ifndef SIM_RF_H
#define SIM_RF_H

#include <stddef.h>
#include "sim_types.h"
#include "stage_models.h"
#include "metrics.h"

int simulate_bruteforce_rf(const void *cfg_ptr, const StageModelsConfig *stage_cfg,
                           const Complex *tx_symbols, const Complex *constellation,
                           size_t constellation_size, size_t nsym,
                           StageMetric *metrics_out, size_t *count_out,
                           double *final_vpp_out, int *sps_out, double *fs_out,
                           const char *csv_dir, const char *const_dir,
                           const char *trace_dir);

int simulate_realistic_rf(const void *cfg_ptr, const StageModelsConfig *stage_cfg,
                          const Complex *tx_symbols, const Complex *constellation,
                          size_t constellation_size, size_t nsym,
                          StageMetric *metrics_out, size_t *count_out,
                          double *final_vpp_out, int *sps_out, double *fs_out,
                          const char *csv_dir, const char *const_dir,
                          const char *trace_dir);

#endif
