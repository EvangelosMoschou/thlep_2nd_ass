#ifndef METRICS_H
#define METRICS_H

#include <stddef.h>
#include "sim_types.h"

/**
 * @brief Metrics recorded after each receiver stage.
 */
typedef struct StageMetric {
    char   stage[64];    /**< Name of the stage. */
    double gain_db;      /**< Nominal gain of this stage. */
    double nf_db;        /**< Nominal noise figure of this stage. */
    double snr_db;       /**< Signal-to-Noise Ratio (dB) after this stage. */
    double evm_pct;      /**< Error Vector Magnitude (%) after this stage. */
    double p_sig_dbm;    /**< Signal power in dBm. */
    double p_noise_dbm;  /**< Noise power in dBm. */
    double v_peak;       /**< Peak absolute voltage seen in the trace. */
    double v_rms;        /**< RMS voltage seen in the trace. */
} StageMetric;

/**
 * @brief Write an array of StageMetric structs to a CSV file.
 */
int write_metrics_csv(const char* path, const StageMetric* metrics, size_t count);

/**
 * @brief Print metrics table to stdout.
 */
void print_metrics(const char* title, const StageMetric* metrics, size_t count);

#endif
