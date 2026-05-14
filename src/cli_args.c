#include "cli_args.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int parse_cli_args(int argc, char **argv, CliArgs *out) {
    /* Initialize defaults */
    out->seed = 42;
    out->symbols = 1000;
    out->input_snr_db = 20.0;
    strncpy(out->stage_csv, "stage_models/stage_models.csv", sizeof(out->stage_csv));
    out->topology_sim = 1;
    out->run_bb = 1;
    out->run_rf = 1;
    out->run_realistic = 1;
    out->carrier_hz = 24.0e9;
    out->antenna_temp_k = 290.0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            return 1;
        } else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            out->seed = (uint32_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "--symbols") == 0 && i + 1 < argc) {
            out->symbols = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
            out->input_snr_db = atof(argv[++i]);
        } else if (strcmp(argv[i], "--stage-csv") == 0 && i + 1 < argc) {
            strncpy(out->stage_csv, argv[++i], sizeof(out->stage_csv));
        } else if (strcmp(argv[i], "--topology-sim") == 0 && i + 1 < argc) {
            out->topology_sim = atoi(argv[++i]);
        }
    }
    return 0;
}

void cli_print_usage(const char *exe) {
    printf("Usage: %s [options]\n", exe);
    printf("Options:\n");
    printf("  --seed <n>         PRNG seed (default 42)\n");
    printf("  --symbols <n>      Number of symbols (default 1000)\n");
    printf("  --snr <db>         Input SNR in dB (default 20)\n");
    printf("  --stage-csv <path> Path to stage models CSV\n");
    printf("  --topology-sim <n> Run ID/Slot (default 1)\n");
}
