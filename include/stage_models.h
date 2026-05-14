#ifndef STAGE_MODELS_H
#define STAGE_MODELS_H

#include <stddef.h>

typedef struct StageModel {
    char*  name;
    double gain_db;
    double nf_db;
    int    filter_len;
    int    is_limiter;
    double p1db_dbm;
    double ip3_dbm;
    double am_pm_coeff;
    double lo_phase_noise_dbc_hz;
    double iq_gain_error_db;
    double iq_phase_error_deg;
    int    filter_type;
    int    filter_order;
} StageModel;

#define STAGE_CHAIN_COUNT 3

typedef enum StageChainId {
    STAGE_CHAIN_BASEBAND_RX   = 0,
    STAGE_CHAIN_RF_FRONTEND   = 1,
    STAGE_CHAIN_RF_POSTMIX_BB = 2
} StageChainId;

typedef struct StageModelsConfig {
    StageModel* chains[STAGE_CHAIN_COUNT];
    size_t      counts[STAGE_CHAIN_COUNT];
} StageModelsConfig;

int stage_models_load_csv(const char* csv_path, StageModelsConfig* out_cfg, char* errbuf, size_t errbuf_size);
void stage_models_free(StageModelsConfig* cfg);
size_t stage_models_get(const StageModelsConfig* cfg, StageChainId id, const StageModel** out_models);
const char* stage_chain_name(StageChainId id);

#endif
