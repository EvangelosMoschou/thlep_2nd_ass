#ifndef SIM_TYPES_H
#define SIM_TYPES_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    double re;
    double im;
} Complex;

typedef struct {
    uint32_t state;
} PrngState;

typedef struct {
    char stage[64];
    char domain[32];
    double gain_db;
    double nf_db;
    double snr_db;
    double evm_pct;
    double noise_power;
    double signal_power;
    int filter_len;
    int is_limiter;
} StageMetric;

typedef struct {
    unsigned int seed;
    int symbols;
    double symbol_rate_hz;
    double rf_sample_rate_hz;
    double carrier_hz;
    double input_snr_db;
    double antenna_temp_k;
    double t0_k;
    double rolloff;
    int run_bb;
    int run_rf;
    int run_realistic;
} SimConfig;

#define MAX_METRICS 64

#endif
