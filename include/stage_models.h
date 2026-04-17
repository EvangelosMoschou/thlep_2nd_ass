/**
 * @file stage_models.h
 * @brief Receiver stage model definitions and CSV-loader public API.
 *
 * A "stage" is one element of a receiver chain (e.g., LNA, mixer, filter).
 * Each stage has a gain (dB), noise figure (dB), and an optional FIR-length
 * parameter used by the moving-average filter model.
 *
 * Three chains are supported (see StageChainId):
 *   - BASEBAND_RX    : complex baseband analytical path
 *   - RF_FRONTEND    : real RF path before downconversion
 *   - RF_POSTMIX_BB  : complex baseband path after downconversion
 *
 * Typical usage:
 * @code
 *   StageModelsConfig cfg = {0};
 *   char errbuf[256];
 *   if (stage_models_load_csv("stage_models.csv", &cfg, errbuf, sizeof(errbuf)) != 0) {
 *       fprintf(stderr, "Load failed: %s\n", errbuf);
 *   }
 *   // ... use cfg ...
 *   stage_models_free(&cfg);
 * @endcode
 */

#ifndef STAGE_MODELS_H
#define STAGE_MODELS_H

#include <stddef.h>

/**
 * @brief Parameters for a single receiver stage.
 *
 * Each stage models one physical component (amplifier, filter, mixer, etc.)
 * in a receiver chain.  Instances are owned by StageModelsConfig and must
 * not be modified directly after loading.
 */
typedef struct StageModel {
    /** Human-readable stage identifier (heap-allocated; freed by stage_models_free). */
    char*  name;

    /** Stage gain or insertion loss in dB.  Positive = gain, negative = loss. */
    double gain_db;

    /** Noise Figure in dB.  Must be >= 0 dB (NF < 0 dB is physically impossible). */
    double nf_db;

    /**
     * Moving-average filter tap count used to model band-limiting.
     * A value of 1 (or less) means no filtering is applied.
     */
    int filter_len;

    /**
     * When non-zero, the stage gain is computed automatically at run time so
     * that the signal's peak-to-peak voltage equals @p target_vpp.
     * The gain_db field is then ignored.
     */
    int auto_gain_to_vpp;

    /**
     * Target peak-to-peak voltage (V) used when @p auto_gain_to_vpp is set.
     * Ignored when @p auto_gain_to_vpp is zero.
     */
    double target_vpp;
} StageModel;

/** @brief Number of distinct receiver stage chains (matches the StageChainId enum count). */
#define STAGE_CHAIN_COUNT 3

/**
 * @brief Identifies one of the three receiver stage chains.
 *
 * The numeric values double as array indices into StageModelsConfig::chains[]
 * and ::counts[], so they must remain contiguous starting from 0.
 */
typedef enum StageChainId {
    STAGE_CHAIN_BASEBAND_RX   = 0, /**< Complex baseband analytical path. */
    STAGE_CHAIN_RF_FRONTEND   = 1, /**< Real RF path (before downconversion). */
    STAGE_CHAIN_RF_POSTMIX_BB = 2  /**< Complex baseband path (after downconversion). */
} StageChainId;

/**
 * @brief Container for all three receiver chain stage lists.
 *
 * Both arrays are indexed by StageChainId.  Each element is a heap-allocated
 * array of StageModel structs; call stage_models_free() to release them.
 *
 * Zero-initialise before first use:
 * @code
 *   StageModelsConfig cfg = {0};
 * @endcode
 */
typedef struct StageModelsConfig {
    StageModel* chains[STAGE_CHAIN_COUNT]; /**< Pointer to stage array for each chain. */
    size_t      counts[STAGE_CHAIN_COUNT]; /**< Number of stages in each chain.        */
} StageModelsConfig;

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Load receiver stage configuration from a CSV file.
 *
 * Parses @p csv_path and populates @p out_cfg with stage data.  Two CSV
 * schemas are accepted automatically:
 *
 *   **Canonical** (preferred):
 *     Columns: chain, name, gain_db, nf_db [, filter_len, auto_gain_to_vpp, target_vpp, enabled]
 *
 *   **Legacy** (backward-compatible):
 *     Columns: component, gain_db, nf_db
 *     (component names are mapped to canonical stages internally)
 *
 * On success, the previous contents of @p out_cfg are freed and replaced.
 * On failure, @p out_cfg is left unchanged and @p errbuf is populated.
 *
 * @param csv_path     Path to the CSV file to read.
 * @param out_cfg      Output configuration struct (must not be NULL).
 * @param errbuf       Caller-provided buffer for a human-readable error message.
 * @param errbuf_size  Size of @p errbuf in bytes.
 *
 * @return  0  on success.
 * @return -1  if @p csv_path or @p out_cfg is NULL.
 * @return -2  if the file cannot be opened.
 * @return -3  if the CSV header is missing required columns.
 * @return -4  if a legacy component mapping fails.
 * @return -5  if a canonical row has too few fields.
 * @return -6  if a chain name is unrecognised.
 * @return -7  if a numeric gain/NF value is invalid.
 * @return -8  if allocating a stage entry fails.
 * @return -9  if the file contains no header row.
 * @return -10 if one or more chains have no stages after loading.
 */
int stage_models_load_csv(
    const char*       csv_path,
    StageModelsConfig* out_cfg,
    char*              errbuf,
    size_t             errbuf_size);

/**
 * @brief Release all heap memory owned by a stage configuration.
 *
 * Frees every stage's @p name string and the chain arrays themselves, then
 * resets all pointers and counts to zero.  Safe to call on a zero-initialised
 * struct and safe to call multiple times.
 *
 * @param cfg  Configuration to free.  Ignored (no-op) if NULL.
 */
void stage_models_free(StageModelsConfig* cfg);

/**
 * @brief Retrieve the stage array for a given receiver chain.
 *
 * @param cfg        Loaded configuration (must not be NULL).
 * @param id         Chain to retrieve.
 * @param out_models On success, set to point at the internal stage array.
 *                   Set to NULL on error.  Must not be NULL.
 *
 * @return Number of stages in the chain, or 0 on error / empty chain.
 */
size_t stage_models_get(
    const StageModelsConfig* cfg,
    StageChainId             id,
    const StageModel**       out_models);

/**
 * @brief Return a human-readable name for a chain identifier.
 *
 * The returned pointer refers to a string literal; do not free it.
 *
 * @param id  Chain identifier.
 * @return    "baseband_rx", "rf_frontend", "rf_postmix_bb", or "unknown".
 */
const char* stage_chain_name(StageChainId id);

#endif /* STAGE_MODELS_H */
