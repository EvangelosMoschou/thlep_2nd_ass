#ifndef SIM_RF_PATHS_H
#define SIM_RF_PATHS_H

#include <stddef.h>

/* Maximum stage-metric entries per simulation path (shared). */
#define MAX_METRICS 32
#include "prng.h"
#include "sim_types.h"

/* Shared RF simulation paths (passband brute-force + realistic impaired).
 * Extracted verbatim from main.c; behavior is bit-identical. */

extern PrngState rng; /* Serial PRNG for symbol generation (defined in sim_rf_paths.c) */

int simulate_bruteforce_rf(
    const SimConfig *cfg, const StageModelsConfig *stage_cfg,
    const Complex *tx_symbols, const Complex *constellation_template,
    size_t constellation_count, size_t nsym, StageMetric *metrics,
    size_t *metric_count, double *final_vpp, int *used_sps, double *used_fs_hz,
    const char *csv_dir, const char *const_dir, const char *trace_dir,
    const char *spectrum_dir);

int simulate_realistic_rf(
    const SimConfig *cfg, const StageModelsConfig *stage_cfg,
    const Complex *tx_symbols, const Complex *constellation_template,
    size_t constellation_count, size_t nsym, StageMetric *metrics,
    size_t *metric_count, double *final_vpp, int *used_sps, double *used_fs_hz,
    const char *csv_dir, const char *const_dir, const char *trace_dir,
    const char *spectrum_dir);

void print_metrics(const char *title, const StageMetric *metrics,
                          size_t count);

#endif /* SIM_RF_PATHS_H */
