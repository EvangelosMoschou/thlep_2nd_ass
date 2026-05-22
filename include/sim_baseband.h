/**
 * @file sim_baseband.h
 * @brief Complex baseband simulation path — fast analytical simulation.
 *
 * Processes complex I/Q symbols directly through the baseband_rx stage chain
 * without any RF modulation or upconversion. This is the "quick and clean"
 * simulation path: O(N_symbols × N_stages).
 *
 * Signal flow:
 *   1. Copy transmitted symbols as reference (clean) and signal (noisy)
 *   2. Add AWGN noise at the configured input SNR
 *   3. Initialize Friis noise trackers
 *   4. Record input constellation metrics
 *   5. For each stage in the baseband_rx chain:
 *      a. Apply stage (filter -> gain -> noise injection)
 *      b. Record output metric
 *   6. Measure final peak-to-peak voltage
 *
 * Uses reentrant PrngState* for all noise injection (no global PRNG).
 */

#ifndef SIM_BASEBAND_H
#define SIM_BASEBAND_H

#include <stddef.h>

#include "prng.h"
#include "sim_types.h"
#include "stage_models.h"

/** Maximum number of stage metrics the baseband path can record. */
#define MAX_BB_METRICS 32

/**
 * @brief Result container for the complex baseband simulation.
 *
 * Holds per-stage metrics and summary values produced by one run of
 * simulate_baseband().
 */
typedef struct SimBasebandResult {
    StageMetric metrics[MAX_BB_METRICS]; /**< Per-stage SNR/EVM metrics */
    size_t      count;                   /**< Number of metrics actually filled */
    double      final_vpp;               /**< Peak-to-peak voltage of reference signal after all stages */
} SimBasebandResult;

/**
 * @brief Run the complex baseband simulation path.
 *
 * Processes @p nsym complex I/Q symbols through the baseband_rx stage chain,
 * recording SNR, EVM, and other metrics at each stage.  Uses reentrant PRNG
 * for all noise injection.
 *
 * @param cfg                Simulation configuration (SNR, carrier, etc.)
 * @param stage_cfg          Loaded stage chain configuration from CSV.
 * @param constellation_template  The 64-point constellation template (for artifact output).
 * @param constellation_count     Number of constellation points (typically 64).
 * @param tx_symbols         Transmitted complex symbols (nsym elements).
 * @param nsym               Number of symbols.
 * @param rng                Reentrant PRNG state (not a global).
 * @param result             [out] Filled on success with metrics, count, and final_vpp.
 * @param csv_dir            Directory for CSV output (NULL to skip artifact writing).
 * @param const_dir          Directory for constellation SVG output (NULL to skip).
 * @param trace_dir          Directory for trace SVG output (NULL to skip).
 *
 * @return  0   on success.
 * @return -1   on memory allocation failure.
 * @return -2   if the baseband_rx chain is missing or empty.
 * @return -3   if the chain has more stages than MAX_BB_METRICS.
 */
int simulate_baseband(const SimConfig *cfg,
                      const StageModelsConfig *stage_cfg,
                      const Complex *constellation_template,
                      size_t constellation_count,
                      const Complex *tx_symbols,
                      size_t nsym,
                      PrngState *rng,
                      SimBasebandResult *result,
                      const char *csv_dir,
                      const char *const_dir,
                      const char *trace_dir);

/**
 * @brief Run the transmitter chain simulation.
 *
 * Processes @p nsym complex I/Q symbols through the transmitter stage chain
 * loaded from transmitter.csv.  Uses the same linear stage model (gain, NF,
 * filtering) as the baseband path.
 *
 * Output directory layout:
 *   out/tx/constellations/transmitter_input.svg
 *   out/tx/constellations/transmitter_stage_01.svg
 *   ...
 *
 * @param cfg                Simulation configuration.
 * @param tx_stage_cfg       Transmitter stage config loaded from transmitter.csv.
 * @param constellation_template  The 64-point constellation template.
 * @param constellation_count     Number of constellation points (typically 64).
 * @param tx_symbols         Transmitted complex symbols.
 * @param nsym               Number of symbols.
 * @param rng                Reentrant PRNG state.
 * @param const_dir          Directory for constellation SVG output (NULL to skip).
 *
 * @return  0   on success.
 * @return -1   on memory allocation failure.
 * @return -2   if the transmitter chain is missing or empty.
 */
int simulate_transmitter(const SimConfig *cfg,
                         const StageModelsConfig *tx_stage_cfg,
                         const Complex *constellation_template,
                         size_t constellation_count,
                         const Complex *tx_symbols,
                         size_t nsym,
                         PrngState *rng,
                         const char *csv_dir,
                         const char *const_dir,
                         const char *trace_dir,
                         const char *spectrum_dir);

#endif /* SIM_BASEBAND_H */
