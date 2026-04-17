#ifndef SIM_TYPES_H
#define SIM_TYPES_H

/*
 * Complex number struct (I/Q format)
 * - re: Real part (In-phase)
 * - im: Imaginary part (Quadrature)
 */
typedef struct Complex {
    double re;
    double im;
} Complex;

/*
 * StageMetric: Measurements at each receiver stage
 * Tracks signal quality degradation through the chain
 */
typedef struct StageMetric {
    const char* stage;        /* Stage name (e.g., "LNA", "Mixer") */
    const char* domain;       /* Signal domain: "complex_baseband", "rf_real", "rf_to_bb" */
    double signal_power;      /* Reference signal power (dBm-equivalent linear) */
    double noise_power;       /* Noise power added at this stage */
    double snr_db;            /* Signal-to-Noise Ratio in dB */
    double evm_pct;           /* Error Vector Magnitude as percentage (I/Q error) */
} StageMetric;

/*
 * SimConfig: Top-level simulation parameters
 * Contains carrier frequency, symbol rate, SNR, antenna temperature, etc.
 */
typedef struct SimConfig {
    double carrier_hz;        /* RF carrier frequency (e.g., 24 GHz = 24e9) */
    double symbol_rate_hz;    /* Symbol clock rate (e.g., 10 MHz = 10e6) */
    int symbols;              /* Number of symbols to transmit */
    double rolloff;           /* RRC filter roll-off factor (0..1, typically 0.2) */
    double input_snr_db;      /* Input SNR in dB (added at antenna input) */
    double antenna_temp_k;    /* Antenna noise temperature in Kelvin */
    double t0_k;              /* Reference temperature (290 K standard) */
    double rf_sample_rate_hz; /* RF sampling rate for brute-force sim (e.g., 96 GHz) */
    unsigned int seed;        /* PRNG seed for reproducibility */
} SimConfig;

#endif
