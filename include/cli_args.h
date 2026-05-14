#ifndef CLI_ARGS_H
#define CLI_ARGS_H

#include <stdint.h>

/*
 * CliArgs — Command line configuration for the receiver simulator
 */
typedef struct CliArgs {
    uint32_t seed;           /* PRNG seed (default=42) */
    int      symbols;        /* Number of symbols to simulate */
    double   input_snr_db;   /* Override input SNR (optional) */
    char     stage_csv[256]; /* Path to stage model CSV */
    int      topology_sim;   /* Simulation slot/id (default=1) */
    int      run_bb;         /* Run complex-baseband analytical path (0/1) */
    int      run_rf;         /* Run brute-force RF path (0/1) */
    int      run_realistic;  /* Run realistic RF path with impairments (0/1) */
    double   carrier_hz;     /* Carrier frequency in Hz */
    double   antenna_temp_k; /* Antenna noise temperature in Kelvin */
} CliArgs;

/*
 * parse_cli_args — Populate CliArgs from argv
 *
 * Returns:
 *   0 on success
 *   1 on help/usage requested
 *   2 on invalid numeric value
 */
int parse_cli_args(int argc, char **argv, CliArgs *out);

/*
 * cli_print_usage — Print usage/help text to stdout
 */
void cli_print_usage(const char *exe);

#endif /* CLI_ARGS_H */
