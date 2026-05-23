#ifndef SIM_RF_REALISTIC_H
#define SIM_RF_REALISTIC_H

#include "sim_types.h"
#include "stage_models.h"

/**
 * @brief Run the realistic RF simulation with all impairment models.
 *
 * Same signal flow as simulate_bruteforce_rf, plus:
 *   - TX LO phase noise before upconversion
 *   - RX LO phase noise at mixer
 *   - I/Q imbalance after downconversion
 *   - Biquad Butterworth filters in BB stages
 *   - Flicker (1/f) noise in BB stages
 *   - ADC quantization + jitter at final stage
 *   - LO leakage DC offset
 *
 * @param cfg               Simulation configuration
 * @param stage_cfg         Stage chain configuration
 * @param tx_symbols        Transmitted symbols
 * @param constellation_template  Reference constellation for EVM
 * @param constellation_count     Number of constellation points
 * @param nsym              Number of symbols
 * @param metrics           Output: stage metrics array
 * @param metric_count      Output: number of metrics filled
 * @param final_vpp         Output: peak-to-peak voltage of reference
 * @param used_sps          Output: samples per symbol used
 * @param used_fs_hz        Output: actual RF sampling frequency
 * @param csv_dir           Directory for CSV output files
 * @param const_dir         Directory for constellation SVG files
 * @param trace_dir         Directory for trace SVG files
 * @param spectrum_dir      Directory for spectrum CSV/SVG files
 *
 * @return 0 on success, -1 on memory error, -2 if chains missing,
 *         -3 if MAX_METRICS exceeded
 */
int simulate_realistic_rf(
    const SimConfig *cfg, const StageModelsConfig *stage_cfg,
    const Complex *tx_symbols, const Complex *constellation_template,
    size_t constellation_count, size_t nsym, StageMetric *metrics,
    size_t *metric_count, double *final_vpp, int *used_sps, double *used_fs_hz,
    const char *csv_dir, const char *const_dir, const char *trace_dir,
    const char *spectrum_dir);

#endif /* SIM_RF_REALISTIC_H */
