#ifndef SIM_BASEBAND_H
#define SIM_BASEBAND_H

#include <stddef.h>
#include "sim_types.h"
#include "stage_models.h"
#include "metrics.h"
#include "prng.h"

typedef struct SimBasebandResult {
    StageMetric *metrics;
    size_t       count;
    double       final_vpp;
} SimBasebandResult;

int simulate_baseband(const void *cfg_ptr, const StageModelsConfig *stage_cfg,
                      const Complex *constellation, size_t constellation_size,
                      const Complex *tx_symbols, size_t nsym, PrngState *rng,
                      SimBasebandResult *out_result, const char *csv_dir,
                      const char *const_dir, const char *trace_dir);

#endif
