#ifndef CLI_ARGS_H
#define CLI_ARGS_H

#include "sim_types.h"

/*
 * CLI_CSV_PATH_MAX — Maximum length of a stage-model CSV path string
 */
#define CLI_CSV_PATH_MAX 512

/*
 * CliArgs — All values parsed from the command line
 *
 * This struct aggregates everything the user can set via CLI flags,
 * including fields that go into SimConfig plus path/slot/impairment
 * config that are stored separately in main().
 */
typedef struct CliArgs {
    /* --- SimConfig fields set from CLI --- */
    unsigned int seed;           /* PRNG seed (default: time-based) */
    int          symbols;        /* Number of symbols to transmit */
    double       symbol_rate_hz; /* Symbol clock rate [Hz] */
    double       rf_sample_rate_hz; /* RF brute-force sample rate [Hz] */
    double       carrier_hz;     /* RF carrier frequency [Hz] */
    double       input_snr_db;   /* Input SNR [dB] */
    int          run_bb;         /* Run complex baseband path */
    int          run_rf;         /* Run RF baseline path */
    int          run_realistic;  /* Run realistic impairment path */

    /* --- Path / slot config --- */
    char stage_csv_path[CLI_CSV_PATH_MAX]; /* Stage-model CSV file or folder */
    int  topology_sim_id;                  /* Output simulation slot (1..N) */

    /* --- Realistic-path impairment toggles --- */
    RealisticPathConfig realistic_cfg;
} CliArgs;

/*
 * parse_cli_args — Parse command-line arguments into CliArgs
 *
 * Sets documented defaults first, then walks argv to apply overrides.
 *
 * Returns:
 *   0 on success
 *   1 on unknown argument (usage error)
 *   2 on invalid numeric value
 */
int parse_cli_args(int argc, char **argv, CliArgs *out);

/*
 * cli_print_usage — Print usage/help text to stdout
 */
void cli_print_usage(const char *exe);

#endif /* CLI_ARGS_H */
