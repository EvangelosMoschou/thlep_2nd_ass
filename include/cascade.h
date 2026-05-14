#ifndef CASCADE_H
#define CASCADE_H

#include "component_catalog.h"
#include "stage_models.h"

/*
 * ============================================================================
 * cascade.h — RF Receiver Cascade Analysis (Friis, IP3, Dynamic Range)
 * ============================================================================
 *
 * Translates CascadeAnalyzer.m into C.  Computes:
 *
 *   1. System power requirements (Ni, Si, Pout for 1 Vpp)
 *   2. Friis cascade: cumulative Gain, Noise Figure, IIP3
 *   3. Dynamic range: noise floor, LDR, SFDR
 *   4. Receiver sensitivity (kTB + NF_total + SNR_req)
 *
 * The analysis runs on the baseband_rx stage chain from the CSV-loaded
 * StageModelsConfig, which exactly matches the CascadeAnalyzer.m stage
 * list (11 stages: switch → BPF → LNA1 → IRF → Mixer1 → BPF2 → LNA2
 * → Mixer2 → BPF3 → LNA3 → Limiter).
 *
 * USAGE:
 *   @code
 *   CascadeResult result;
 *   CascadeParams params = {
 *       .antenna_temp_k = 150.0,
 *       .t0_k           = 290.0,
 *       .bw_hz          = 200.0e6,
 *       .vpp_out        = 1.0,
 *       .snr_target_db  = 20.0
 *   };
 *   compute_cascade(&stage_cfg, &params, STAGE_CHAIN_BASEBAND_RX, &result);
 *   print_cascade(&result);
 *   // result.sensitivity_dbm is ready for the link budget
 *   @endcode
 * ============================================================================
 */

/* ============================================================================
 * CASCADE PARAMETERS
 * ============================================================================ */

typedef struct CascadeParams {
    double antenna_temp_k;   /**< Antenna equivalent noise temperature [K]  (150 K) */
    double t0_k;             /**< Reference temperature [K]                  (290 K) */
    double bw_hz;            /**< Receiver bandwidth [Hz]                    (200 MHz) */
    double vpp_out;          /**< Target output voltage peak-to-peak [V]     (1.0 V) */
    double snr_target_db;    /**< Target input SNR [dB]                      (20 dB) */
    double snr_required_db;  /**< Required SNR for modulation [dB]           (26.5 for 64-APSK) */
} CascadeParams;

/* ============================================================================
 * PER-STAGE CASCADE ENTRY
 * ============================================================================ */

typedef struct StageCascadeEntry {
    const char *name;        /**< Stage name */
    double gain_db;          /**< Stage gain [dB] */
    double nf_db;            /**< Stage noise figure [dB] */
    double iip3_dbm;         /**< Stage IIP3 [dBm] */
    double cum_gain_db;      /**< Cumulative gain to this stage [dB] */
    double cum_nf_db;        /**< Cumulative noise figure to this stage [dB] */
    double cum_iip3_dbm;     /**< Cumulative IIP3 to this stage [dBm] */
} StageCascadeEntry;

/* ============================================================================
 * CASCADE RESULT
 * ============================================================================ */

typedef struct CascadeResult {
    /* --- System requirements --- */
    double ni_dbm;                 /**< Input noise power [dBm] */
    double si_dbm;                 /**< Input signal power [dBm] */
    double pout_dbm;               /**< Target output power [dBm] */
    double total_required_gain_db; /**< Gain needed for 1 Vpp output [dB] */

    /* --- Cascade summary --- */
    int    num_stages;             /**< Number of stages in chain */
    double total_gain_db;          /**< Total cascaded gain [dB] */
    double total_nf_db;            /**< Total cascaded noise figure [dB] */
    double total_iip3_dbm;         /**< Total cascaded input IP3 [dBm] */

    /* --- Output noise & dynamic range --- */
    double n_out_dbm;              /**< Output noise power [dBm] */
    double input_p1db_dbm;         /**< Input-referred P1dB [dBm] */
    double output_p1db_dbm;        /**< Output-referred P1dB [dBm] */
    double oip3_dbm;               /**< Output IP3 [dBm] */
    double ldr_db;                 /**< Linear dynamic range [dB] */
    double sfdr_db;                /**< Spurious-free dynamic range [dB] */

    /* --- Sensitivity --- */
    double noise_floor_dbm;        /**< kTB noise floor [dBm] */
    double sensitivity_dbm;        /**< Receiver sensitivity [dBm] (noise_floor + NF + SNR_req) */

    /* --- Per-stage detail (first NUM_CASCADE_STAGES stages) --- */
    StageCascadeEntry stages[32];
    int                stage_count;
} CascadeResult;

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/**
 * @brief Run the full cascade analysis on a receiver stage chain.
 *
 * Loads the specified chain from the stage models config, then:
 *   1. Computes system power budget (Ni, Si, Pout)
 *   2. Runs Friis noise cascade
 *   3. Runs IIP3 cascade
 *   4. Computes dynamic range (noise floor, LDR, SFDR)
 *   5. Computes receiver sensitivity
 *
 * @param stage_cfg   Loaded stage configuration (from stage_models_load_csv)
 * @param params      System parameters (T_ant, T_0, BW, Vpp, SNR target)
 * @param chain_id    Which chain to analyse (use STAGE_CHAIN_BASEBAND_RX for the full 11-stage chain)
 * @param catalog     Component catalog for datasheet-correct P1dB/IIP3 (NULL to use CSV values)
 * @param result      [out] Filled cascade result
 * @return 0 on success, -1 if chain empty
 */
int compute_cascade(const StageModelsConfig *stage_cfg,
                    const CascadeParams *params,
                    int chain_id,
                    const ComponentCatalog *catalog,
                    CascadeResult *result);

/**
 * @brief Print a formatted cascade analysis table to stdout.
 *
 * Shows per-stage cascade (cumulative gain, NF, IIP3) and summary
 * (system requirements, dynamic range, sensitivity).
 */
void print_cascade(const CascadeResult *result);

#endif /* CASCADE_H */
