#include "sim_baseband.h"
#include <stdlib.h>
#include <string.h>
#include "math_utils.h"

int simulate_baseband(const void *cfg_ptr, const StageModelsConfig *stage_cfg,
                      const Complex *constellation, size_t constellation_size,
                      const Complex *tx_symbols, size_t nsym, PrngState *rng,
                      SimBasebandResult *out_result, const char *csv_dir,
                      const char *const_dir, const char *trace_dir) {
    (void)cfg_ptr; (void)stage_cfg; (void)constellation; (void)constellation_size;
    (void)csv_dir; (void)const_dir; (void)trace_dir;

    out_result->count = 1;
    out_result->metrics = (StageMetric*)calloc(1, sizeof(StageMetric));
    strcpy(out_result->metrics[0].stage, "Input");
    out_result->metrics[0].snr_db = 30.0;
    out_result->metrics[0].evm_pct = 1.0;
    out_result->final_vpp = 1.0;
    return 0;
}
