#include "cli_args.h"

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*
 * TOPOLOGY_SIM_COUNT — Number of simulation "slots" (separate output
 * directories). Users can run up to N independent simulations (e.g., with
 * different stage CSV files or different SNR values) and compare results
 * side-by-side.
 */
#define TOPOLOGY_SIM_COUNT 1

/*
 * parse_u32 — Parse a string as an unsigned 32-bit integer with validation
 *
 * Returns: 0 on success, -1 if NULL, -2 if not a valid unsigned integer
 */
static int parse_u32(const char *s, unsigned int *out) {
  char *end = NULL;
  unsigned long v;

  if (!s || !out)
    return -1;

  errno = 0;
  v = strtoul(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || v > UINT_MAX) {
    return -2;
  }

  *out = (unsigned int)v;
  return 0;
}

/*
 * parse_i32 — Parse a string as a signed 32-bit integer with validation
 */
static int parse_i32(const char *s, int *out) {
  char *end = NULL;
  long v;

  if (!s || !out)
    return -1;

  errno = 0;
  v = strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || v < INT_MIN || v > INT_MAX) {
    return -2;
  }

  *out = (int)v;
  return 0;
}

/*
 * parse_double — Parse a string as a double-precision float with validation
 */
static int parse_double(const char *s, double *out) {
  char *end = NULL;
  double v;

  if (!s || !out)
    return -1;

  errno = 0;
  v = strtod(s, &end);
  if (errno != 0 || end == s || *end != '\0') {
    return -2;
  }

  *out = v;
  return 0;
}

/*
 * cli_print_usage — Display command-line usage help
 */
void cli_print_usage(const char *exe) {
  printf("Usage: %s [options]\n", exe);
  printf("Options:\n");
  printf("  --seed <int>          RNG seed (default: time-based)\n");
  printf("  --symbols <int>       Number of symbols (default: 256)\n");
  printf("  --symbol-rate <Hz>    Symbol rate in Hz (default: 1e7)\n");
  printf("  --rf-fs <Hz>          RF brute-force sample rate in Hz (default: "
         "9.6e10)\n");
  printf("  --carrier <Hz>        Carrier frequency in Hz (default: 2.0e10)\n");
  printf("  --snr <dB>            Input SNR in dB (default: 20)\n");
  printf("  --stage-csv <path>    Stage-model CSV file or folder (default: "
          "data_input/20ghz/receiver.csv)\n");
  printf("  --topology-sim <1..4> Output simulation slot under "
         "out/topology_sim_N (default: 1)\n");
  printf("  --stage-sim <1..4>    Alias for --topology-sim\n");
  printf("  --disable-bb          Enable complex baseband path (default: disabled)\n");
  printf("  --enable-rf           Enable RF baseline path (default: enabled)\n");
  printf("  --enable-realistic    Enable realistic impairment path (default: enabled)\n");
  printf("  --disable-rf          Disable RF baseline path\n");
  printf("  --disable-realistic   Disable realistic impairment path\n");
}

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
int parse_cli_args(int argc, char **argv, CliArgs *out) {
  int i;

  if (!out)
    return 1;

  /* --- Set default simulation parameters --- */
  out->carrier_hz = 20.0e9;         /* 20 GHz carrier (K-band satellite) */
  out->symbol_rate_hz = 10.0e6;     /* 10 MegaSymbols/sec */
  out->symbols = 3000;              /* 3000 symbols */
  out->input_snr_db = 20.0;         /* 20 dB input SNR */
  out->rf_sample_rate_hz = 96.0e9;  /* 96 GHz RF sampling */
  out->seed = (unsigned int)time(NULL); /* Default seed: current time */
  out->run_bb = 0;                  /* Baseband path disabled by default */
  out->run_rf = 1;                  /* RF baseline path enabled by default */
  out->run_realistic = 1;           /* Realistic path enabled by default */
  out->topology_sim_id = 1;         /* Default output slot: topology_sim_1 */
  snprintf(out->stage_csv_path, CLI_CSV_PATH_MAX, "%s",
            "data_input/20ghz/receiver.csv");

  out->realistic_cfg.enable_lo_phase_noise = 1;
  out->realistic_cfg.enable_iq_gain_error = 1;
  out->realistic_cfg.enable_iq_phase_error = 1;
  out->realistic_cfg.enable_am_pm = 1;
  out->realistic_cfg.enable_butterworth = 1;

  /* --- Parse command-line arguments --- */
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      if (parse_u32(argv[++i], &out->seed) != 0) {
        fprintf(stderr, "Invalid --seed value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--symbols") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &out->symbols) != 0 || out->symbols <= 0) {
        fprintf(stderr, "Invalid --symbols value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--symbol-rate") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &out->symbol_rate_hz) != 0 ||
          out->symbol_rate_hz <= 0.0) {
        fprintf(stderr, "Invalid --symbol-rate value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--rf-fs") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &out->rf_sample_rate_hz) != 0 ||
          out->rf_sample_rate_hz <= 0.0) {
        fprintf(stderr, "Invalid --rf-fs value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--carrier") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &out->carrier_hz) != 0 ||
          out->carrier_hz <= 0.0) {
        fprintf(stderr, "Invalid --carrier value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &out->input_snr_db) != 0) {
        fprintf(stderr, "Invalid --snr value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--stage-csv") == 0 && i + 1 < argc) {
      snprintf(out->stage_csv_path, CLI_CSV_PATH_MAX, "%s", argv[++i]);
    } else if (strcmp(argv[i], "--topology-sim") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &out->topology_sim_id) != 0 ||
          out->topology_sim_id < 1 ||
          out->topology_sim_id > TOPOLOGY_SIM_COUNT) {
        fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n",
                TOPOLOGY_SIM_COUNT);
        return 2;
      }
    } else if (strcmp(argv[i], "--stage-sim") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &out->topology_sim_id) != 0 ||
          out->topology_sim_id < 1 ||
          out->topology_sim_id > TOPOLOGY_SIM_COUNT) {
        fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n",
                TOPOLOGY_SIM_COUNT);
        return 2;
      }
    } else if (strcmp(argv[i], "--disable-bb") == 0) {
      out->run_bb = 0;
    } else if (strcmp(argv[i], "--enable-rf") == 0) {
      out->run_rf = 1;
    } else if (strcmp(argv[i], "--enable-realistic") == 0) {
      out->run_realistic = 1;
    } else if (strcmp(argv[i], "--disable-rf") == 0) {
      out->run_rf = 0;
    } else if (strcmp(argv[i], "--disable-realistic") == 0) {
      out->run_realistic = 0;
    } else {
      cli_print_usage(argv[0]);
      return 1;
    }
  }

  return 0;
}
