/*
 * main.c — 64-APSK Receiver Dual-Mode Simulator (Orchestrator)
 *
 * Thin orchestrator that delegates to extracted modules:
 *   constellation, metrics, cli_args, output_mgr, signal_chain, sim_baseband, prng
 *
 * The RF simulation code (simulate_bruteforce_rf, simulate_realistic_rf) and
 * their supporting SoA/FFT helpers remain here because they were too tightly
 * coupled to extract.
 */

#define _USE_MATH_DEFINES
#define _GNU_SOURCE
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <math.h>
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/stat.h>
#include <time.h>

/* --- Extracted module headers --- */
#include "cli_args.h"
#include "constellation.h"
#include "math_utils.h"
#include "metrics.h"
#include "output_mgr.h"
#include "physics.h"
#include "prng.h"
#include "signal_chain.h"
#include "sim_baseband.h"
#include "sim_types.h"
#include "stage_artifacts.h"
#include "stage_models.h"
#include "fft.h"
#include "soa_utils.h"
#include "spectrum.h"
#include "perf_timer.h"
#include "rf_setup.h"

/* --- Propagation analysis (Part D: FSPL, rain, fog, gas, link margin) --- */
#include "propagation.h"

/* --- Cascade analysis (Part E: Friis, IP3, dynamic range, sensitivity) --- */
#include "cascade.h"

/* --- RF Simulation Engines --- */
#include "sim_rf.h"
#include "sim_rf_realistic.h"


/* ============================================================================
 * CONSTANTS
 * ============================================================================ */

#define OUTPUT_ROOT_DIR "out"
#define TOPOLOGY_SIM_COUNT 1

/*
 * MAX_METRICS — Maximum number of stage metric entries per simulation path.
 * 32 is enough for chains with up to ~30 stages.
 */
#define MAX_METRICS 32

/* ============================================================================
 * GLOBAL PRNG STATE
 * ============================================================================ */

PrngState rng;                              /* Serial PRNG for symbol generation */


/* math_utils.h provides db_to_lin_power, lin_to_db */

/* ============================================================================
 * 64-APSK CONSTELLATION CONSTRUCTION
 * ============================================================================
 *
 * A "constellation" is the set of all possible symbols a transmitter can send.
 * 64-APSK has 64 symbols, each represented as a complex number (I + jQ).
 *
 * Visually, on the I/Q plane (a 2D plot where X=I and Y=Q), the 64 symbols
 * form 4 concentric rings, each with a different number of evenly-spaced
 * points at different radii. This is specified by the DVB-S2X standard.
 * ============================================================================
 */

static void generate_symbols(const Complex *constellation, int m, Complex *out,
                             unsigned short *labels, size_t n) {
  size_t i;

  for (i = 0; i < n; ++i) {
    /*
     * Pick a random index between 0 and m-1 (inclusive).
     * prng_uint32() returns a random 32-bit integer; modulo m maps it
     * to the valid range. Since m=64 is a power of 2, there's zero bias.
     */
    const unsigned int idx = prng_uint32(&rng) % (unsigned int)m;

    /* Copy the selected constellation point into the output array */
    out[i] = constellation[idx];

    /* Record which symbol was transmitted (for potential SER analysis) */
    labels[i] = (unsigned short)idx;
  }
}






/* ============================================================================
 * SIMULATION ENGINES
 * ============================================================================
 *
 * These are the two main simulation functions that orchestrate the entire
 * signal processing chain. Each one runs the signal through all stages,
 * measures quality, and writes output files.
 * ============================================================================ */

/*
 * print_metrics — Display a formatted table of cumulative end-to-end stage
 * metrics on the console
 *
 * What it does:
 *   Prints a table with columns: Stage, Domain, SNR(dB), EVM(%), NoisePow
 *   Each row shows the signal quality at one point in the receiver chain.
 *
 * Parameters:
 *   title   — Section heading (printed above the table)
 *   metrics — Array of StageMetric structs
 *   count   — Number of entries to print
 */
static void print_metrics(const char *title, const StageMetric *metrics,
                          size_t count) {
  size_t i;

  printf("\n%s\n", title);
  printf("%-18s %-14s %12s %12s %12s\n", "Stage", "Domain", "SNR(dB)", "EVM(%)",
         "NoisePow");
  for (i = 0; i < count; ++i) {
    char stage_label[128];
    humanize_stage_name(metrics[i].stage, stage_label, sizeof(stage_label));
    {
      char numbered_label[256];
      snprintf(numbered_label, sizeof(numbered_label), "%zu. %s", i + 1u,
               stage_label);
      strncpy(stage_label, numbered_label, sizeof(stage_label));
      stage_label[sizeof(stage_label) - 1u] = '\0';
    }
    printf("%-18s %-14s %12.3f %12.3f %12.4e\n", stage_label, metrics[i].domain,
           metrics[i].snr_db, metrics[i].evm_pct, metrics[i].noise_power);
  }
}

/* ============================================================================
 * COMMAND-LINE ARGUMENT PARSING
 * ============================================================================
 */

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
 * resolve_project_root — Find the project root directory from the executable path.
 *
 * Uses /proc/self/exe to get the binary's absolute path, then strips
 * "/bin/dual_receiver_sim" to get the project root.  This makes all
 * relative paths (data_input/, out/) work regardless of how the binary
 * is launched (terminal, double-click, etc.).
 *
 * Parameters:
 *   buf       — Output buffer for the project root path
 *   buf_size  — Size of the output buffer
 *
 * Returns:
 *   0 on success, -1 on error
 */
static int resolve_project_root(char *buf, size_t buf_size) {
    char exe_path[1024];
    ssize_t len;
    char *p;

    len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0) return -1;
    exe_path[len] = '\0';

    /* Strip "/bin/dual_receiver_sim" to get project root.
     * Works for any binary name under any "bin/"-like dir. */
    p = strrchr(exe_path, '/');
    if (!p) return -1;
    *p = '\0';          /* remove binary name */
    p = strrchr(exe_path, '/');
    if (!p) return -1;
    *p = '\0';          /* remove bin/ directory */

    if (strlen(exe_path) >= buf_size) return -1;
    snprintf(buf, buf_size, "%s", exe_path);
    return 0;
}

/*
 * resolve_stage_csv_path — Resolve a stage CSV path (file or directory)
 *
 * What it does:
 *   If the user provides a DIRECTORY path (e.g., "data_input/"), this
 * function resolves it to the specific "runtime_stage_models.csv" file inside
 * that directory. If the user provides a FILE path, it's used as-is.
 *
 *   This exists because the data_input/ directory may contain multiple CSV
 * files:
 *   - stage_models.csv               — design reference / topology sketch
 *   - runtime_stage_models.csv       — simulator-ready canonical configuration
 *   Automatically choosing the runtime file prevents accidentally loading
 *   the design sketch (which may have incomplete data).
 *
 * Parameters:
 *   input_path    — User-provided path (file or directory)
 *   resolved_path — Output buffer for the resolved file path
 *   resolved_size — Size of the output buffer
 *
 * Returns:
 *   0 on success, negative on error
 */
static int resolve_stage_csv_path(const char *input_path, char *resolved_path,
                                  size_t resolved_size) {
  struct stat st;
  int written;
  size_t input_len;

  if (!input_path || !resolved_path || resolved_size == 0u) {
    return -1;
  }

  /* Check if the input is a directory */
  if (stat(input_path, &st) == 0 && S_ISDIR(st.st_mode)) {
    /* Strip trailing slashes from the directory path */
    input_len = strlen(input_path);
    while (input_len > 1u && input_path[input_len - 1u] == '/') {
      --input_len;
    }

    /* Append the runtime CSV filename */
    written =
        snprintf(resolved_path, resolved_size, "%.*s/runtime_stage_models.csv",
                 (int)input_len, input_path);
    if (written < 0 || (size_t)written >= resolved_size) {
      return -2; /* Path too long */
    }

    /* Verify the resolved file exists */
    if (stat(resolved_path, &st) != 0) {
      return -3; /* File doesn't exist */
    }

    return 0;
  }

  /* Not a directory — use the path as-is */
  written = snprintf(resolved_path, resolved_size, "%s", input_path);
  if (written < 0 || (size_t)written >= resolved_size) {
    return -4;
  }

  return 0;
}

/*
 * print_usage — Display command-line usage help
 */
static void print_usage(const char *exe) {
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
}

/* ============================================================================
 * MAIN — Program Entry Point
 * ============================================================================
 */

/*
 * main — Orchestrate the complete 64-APSK receiver simulation
 *
 * Execution sequence:
 *   1.  Parse command-line arguments (--seed, --symbols, --snr, etc.)
 *   2.  Initialize the PRNG with the seed
 *   3.  Create/verify output directory structure
 *   4.  Resolve the stage-model CSV path
 *   5.  Build the DVB-S2X 64-APSK constellation (64 fixed I/Q points)
 *   6.  Generate a random transmitted symbol sequence
 *   7.  Calculate the link budget (thermal noise power + signal power)
 *   8.  Write input budget CSV and SVG
 *   9.  Load receiver stage configuration from CSV
 *  10.  Run complex-baseband simulation
 *  11.  Run brute-force RF simulation
 *  12.  Write stage metrics CSV and SVG for both paths
 *  13.  Print console summary
 *  14.  Free all allocated memory and exit
 *
 * Exit codes:
 *   0 = Success
 *   1 = Usage error (unknown argument)
 *   2 = Argument parse error (invalid number for --seed, --snr, etc.)
 *   3 = Constellation build error
 *   4 = Memory allocation error
 *   5 = CSV load error or directory error
 *   6 = Complex-baseband simulation error
 *   7 = Brute-force RF simulation error
 *   8 = Output directory creation error
 *   9 = Realistic RF simulation error
 */
int main(int argc, char **argv) {
  char project_root[512] = "";
  SimConfig cfg;               /* Top-level simulation parameters */
  StageModelsConfig stage_cfg; /* Loaded stage chain configuration */
  StageModelsConfig tx_stage_cfg; /* Transmitter stage chain configuration */
  char stage_csv_path[512] =
      "data_input/20ghz/receiver.csv"; /* Default CSV path */
  char tx_stage_csv_path[512] =
      "data_input/20ghz/transmitter.csv"; /* Default transmitter CSV path */
  char resolved_stage_csv_path[512]; /* Resolved path after directory resolution
                                       */
  char stage_err[256];               /* Error message buffer for CSV loading */

  char out_dir[512] = "out";
  /* Resolve project root from executable location so paths work even when
   * the binary is launched by double-click (working dir ≠ project root). */
  if (resolve_project_root(project_root, sizeof(project_root)) == 0) {
      /* Prepend project root to the default CSV paths */
      char tmp[512];
      snprintf(tmp, sizeof(tmp), "%s/%s", project_root, stage_csv_path);
      snprintf(stage_csv_path, sizeof(stage_csv_path), "%s", tmp);
      snprintf(tmp, sizeof(tmp), "%s/%s", project_root, tx_stage_csv_path);
      snprintf(tx_stage_csv_path, sizeof(tx_stage_csv_path), "%s", tmp);
      /* Build absolute output directory path */
      snprintf(out_dir, sizeof(out_dir), "%s/out", project_root);
  }
  Complex constellation[64]; /* The 64-APSK constellation (stack-allocated) */
  Complex *tx_symbols;       /* Transmitted symbol sequence (heap-allocated) */
  unsigned short *labels; /* Transmitted symbol indices (for potential SER) */
  StageMetric metrics_bb[MAX_METRICS]; /* Baseband path metrics */
  StageMetric metrics_rf[MAX_METRICS]; /* RF path metrics */
  StageMetric metrics_realistic[MAX_METRICS]; /* Realistic path metrics */
  PerfTimer timer_bb;
  PerfTimer timer_rf;
  PerfTimer timer_realistic;
  size_t count_bb = 0u;                /* Number of baseband metrics */
  size_t count_rf = 0u;                /* Number of RF metrics */
  size_t count_realistic = 0u;         /* Number of realistic metrics */
  double final_vpp_bb = 0.0;           /* Final Vpp after baseband path */
  double final_vpp_rf = 0.0;           /* Final Vpp after RF path */
  double final_vpp_realistic = 0.0;    /* Final Vpp after realistic path */
  int rf_sps = 0;                      /* RF samples per symbol (output) */
  int realistic_sps = 0;               /* Realistic samples per symbol (output) */
  double rf_fs_used = 0.0;             /* RF sampling frequency used (output) */
  double realistic_fs_used = 0.0;      /* Realistic sampling frequency used (output) */
  int i;                               /* Loop variable for CLI parsing */

  /* Link budget variables */
  double noise_bw_hz;  /* Noise bandwidth (MATLAB-aligned 200 MHz) */
  double noise_w;      /* Thermal noise power in watts */
  double noise_dbm;    /* Noise power in dBm */
  double signal_dbm;   /* Signal power in dBm */
  int topology_sim_id; /* Output simulation slot (1–4) */

  /* --- Set default simulation parameters --- */
      cfg.carrier_hz = 20.0e9;     /* 20 GHz carrier (K-band satellite) */
  cfg.symbol_rate_hz = 10.0e6; /* 10 MegaSymbols/sec */
  cfg.symbols = 3000;          /* 3000 symbols - reasonable run time */
  cfg.rolloff = 0.0;           /* Rectangular pulses (fastest) */
  cfg.input_snr_db = 20.0;     /* 20 dB input SNR based on assignment */
    cfg.antenna_temp_k =
        91.0;         /* 91 K antenna noise temp @ 44° elev (Viasat 13.5m, Athens→SES-17) */
  cfg.t0_k = 290.0; /* 290 K reference temperature (room temp) */
  cfg.rf_sample_rate_hz = 96.0e9;      /* 96 GHz RF sampling */
  cfg.seed = (unsigned int)time(NULL); /* Default seed: current time */
  cfg.run_bb = 0;                      /* Baseband path disabled by default */
  cfg.run_rf = 1;                      /* RF baseline path enabled by default */
  cfg.run_realistic = 1;               /* Realistic path enabled by default */
  topology_sim_id = 1;                 /* Default output slot: topology_sim_1 */
  memset(&stage_cfg, 0, sizeof(stage_cfg));
  stage_err[0] = '\0';

  /* --- Parse command-line arguments --- */
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      if (parse_u32(argv[++i], &cfg.seed) != 0) {
        fprintf(stderr, "Invalid --seed value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--symbols") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &cfg.symbols) != 0 || cfg.symbols <= 0) {
        fprintf(stderr, "Invalid --symbols value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--symbol-rate") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.symbol_rate_hz) != 0 ||
          cfg.symbol_rate_hz <= 0.0) {
        fprintf(stderr, "Invalid --symbol-rate value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--rf-fs") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.rf_sample_rate_hz) != 0 ||
          cfg.rf_sample_rate_hz <= 0.0) {
        fprintf(stderr, "Invalid --rf-fs value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--carrier") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.carrier_hz) != 0 ||
          cfg.carrier_hz <= 0.0) {
        fprintf(stderr, "Invalid --carrier value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.input_snr_db) != 0) {
        fprintf(stderr, "Invalid --snr value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--stage-csv") == 0 && i + 1 < argc) {
      snprintf(stage_csv_path, sizeof(stage_csv_path), "%s", argv[++i]);
    } else if (strcmp(argv[i], "--tx-csv") == 0 && i + 1 < argc) {
      snprintf(tx_stage_csv_path, sizeof(tx_stage_csv_path), "%s", argv[++i]);
    } else if (strcmp(argv[i], "--topology-sim") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &topology_sim_id) != 0 || topology_sim_id < 1 ||
          topology_sim_id > TOPOLOGY_SIM_COUNT) {
        fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n",
                TOPOLOGY_SIM_COUNT);
        return 2;
      }
    } else if (strcmp(argv[i], "--stage-sim") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &topology_sim_id) != 0 || topology_sim_id < 1 ||
          topology_sim_id > TOPOLOGY_SIM_COUNT) {
        fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n",
                TOPOLOGY_SIM_COUNT);
        return 2;
      }
    } else if (strcmp(argv[i], "--disable-bb") == 0) {
      cfg.run_bb = 0;
    } else if (strcmp(argv[i], "--enable-rf") == 0) {
      cfg.run_rf = 1;
    } else if (strcmp(argv[i], "--enable-realistic") == 0) {
      cfg.run_realistic = 1;
    } else if (strcmp(argv[i], "--disable-rf") == 0) {
      cfg.run_rf = 0;
    } else if (strcmp(argv[i], "--disable-realistic") == 0) {
      cfg.run_realistic = 0;
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }

  /* --- Initialize the PRNG --- */
  prng_init(&rng, cfg.seed);

  /* --- Create output directories --- */
  if (ensure_output_dirs(out_dir, topology_sim_id) != 0) {
    fprintf(stderr, "Failed to create output directories under ./out\n");
    return 8;
  }

  /* --- Resolve the stage-model CSV path --- */
  if (resolve_stage_csv_path(stage_csv_path, resolved_stage_csv_path,
                             sizeof(resolved_stage_csv_path)) != 0) {
    fprintf(stderr, "Failed to resolve stage-model CSV path '%s'\n",
            stage_csv_path);
    return 5;
  }

  /* --- Build output directory paths --- */
  char rf_csv_dir[256];
  char rf_const_dir[256];
  char rf_trace_dir[256];
  char realistic_csv_dir[256];
  char realistic_const_dir[256];
  char realistic_trace_dir[256];
  char realistic_spectrum_dir[256];
  char tx_csv_dir[256];
  char tx_const_dir[256];
  char tx_trace_dir[256];
  char tx_spectrum_dir[256];
  char rf_spectrum_dir[256];
  if (snprintf(rf_csv_dir, sizeof(rf_csv_dir), "%s/rf_baseline/Rx/csv", out_dir) >= (int)sizeof(rf_csv_dir) ||
      snprintf(rf_const_dir, sizeof(rf_const_dir), "%s/rf_baseline/Rx/constellations", out_dir) >= (int)sizeof(rf_const_dir) ||
      snprintf(rf_trace_dir, sizeof(rf_trace_dir), "%s/rf_baseline/Rx/traces", out_dir) >= (int)sizeof(rf_trace_dir) ||
      snprintf(rf_spectrum_dir, sizeof(rf_spectrum_dir), "%s/rf_baseline/Rx/spectrum", out_dir) >= (int)sizeof(rf_spectrum_dir) ||
      snprintf(realistic_csv_dir, sizeof(realistic_csv_dir), "%s/realistic/Rx/csv", out_dir) >= (int)sizeof(realistic_csv_dir) ||
      snprintf(realistic_const_dir, sizeof(realistic_const_dir), "%s/realistic/Rx/constellations", out_dir) >= (int)sizeof(realistic_const_dir) ||
      snprintf(realistic_trace_dir, sizeof(realistic_trace_dir), "%s/realistic/Rx/traces", out_dir) >= (int)sizeof(realistic_trace_dir) ||
      snprintf(realistic_spectrum_dir, sizeof(realistic_spectrum_dir), "%s/realistic/Rx/spectrum", out_dir) >= (int)sizeof(realistic_spectrum_dir) ||
      snprintf(tx_csv_dir, sizeof(tx_csv_dir), "%s/realistic/Tx/csv", out_dir) >= (int)sizeof(tx_csv_dir) ||
      snprintf(tx_const_dir, sizeof(tx_const_dir), "%s/realistic/Tx/constellations", out_dir) >= (int)sizeof(tx_const_dir) ||
      snprintf(tx_trace_dir, sizeof(tx_trace_dir), "%s/realistic/Tx/traces", out_dir) >= (int)sizeof(tx_trace_dir) ||
      snprintf(tx_spectrum_dir, sizeof(tx_spectrum_dir), "%s/realistic/Tx/spectrum", out_dir) >= (int)sizeof(tx_spectrum_dir)) {
    fprintf(stderr, "Failed to build output subdirectories\n");
    return 5;
  }

  /* Clear old output files from the selected slot */
  if (clean_output_dir(rf_csv_dir) != 0 ||
      clean_output_dir(rf_const_dir) != 0 ||
      clean_output_dir(rf_trace_dir) != 0 ||
      clean_output_dir(rf_spectrum_dir) != 0 ||
      clean_output_dir(realistic_csv_dir) != 0 ||
      clean_output_dir(realistic_const_dir) != 0 ||
      clean_output_dir(realistic_trace_dir) != 0 ||
      clean_output_dir(realistic_spectrum_dir) != 0 ||
      clean_output_dir(tx_csv_dir) != 0 ||
      clean_output_dir(tx_const_dir) != 0 ||
      clean_output_dir(tx_trace_dir) != 0 ||
      clean_output_dir(tx_spectrum_dir) != 0) {
    fprintf(
        stderr,
        "Failed to clear output directories before writing new artifacts\n");
    return 5;
  }

  /* --- Build the 64-APSK constellation --- */
  if (build_dvbs2_64apsk_constellation(constellation, 64) != 0) {
    fprintf(stderr, "Failed to build 64-APSK constellation\n");
    return 3;
  }

  /* --- Allocate and generate transmitted symbols --- */
  tx_symbols = (Complex *)calloc((size_t)cfg.symbols, sizeof(Complex));
  labels =
      (unsigned short *)calloc((size_t)cfg.symbols, sizeof(unsigned short));
  if (!tx_symbols || !labels) {
    fprintf(stderr, "Allocation failed for symbols\n");
    free(tx_symbols);
    free(labels);
    return 4;
  }

  generate_symbols(constellation, 64, tx_symbols, labels, (size_t)cfg.symbols);

  /* --- Noise floor calculation (used later for input SNR) --- */
  noise_bw_hz = 200.0e6;
  noise_w = K_BOLTZMANN * cfg.antenna_temp_k * noise_bw_hz;
  noise_dbm = lin_to_db(noise_w) + 30.0;
  /* Default input SNR — overridden after link budget computation */
  cfg.input_snr_db = 20.0;

  /* --- Load receiver stage configuration from CSV --- */
  if (stage_models_load_csv(resolved_stage_csv_path, &stage_cfg, stage_err,
                            sizeof(stage_err)) != 0) {
    fprintf(stderr, "Failed to load stage-model CSV '%s': %s\n",
            resolved_stage_csv_path, stage_err);
    free(tx_symbols);
    free(labels);
    return 5;
  }

  /* --- Load transmitter stage configuration from CSV --- */
  memset(&tx_stage_cfg, 0, sizeof(tx_stage_cfg));
  if (stage_models_load_csv(tx_stage_csv_path, &tx_stage_cfg, stage_err,
                            sizeof(stage_err)) != 0) {
    fprintf(stderr, "Warning: could not load transmitter CSV '%s' — skipping TX simulation\n",
            tx_stage_csv_path);
  }

  /* --- Load component catalog (datasheet values for IIP3/P1dB) --- */
  ComponentCatalog component_catalog;
  memset(&component_catalog, 0, sizeof(component_catalog));
  {
      const char *cat_path = resolved_stage_csv_path;
      if (cat_path[0] == '\0') cat_path = "data_input/20ghz/receiver.csv";
      if (component_catalog_load(cat_path, &component_catalog) != 0) {
          fprintf(stderr, "Warning: could not load '%s' — cascade will use CSV values\n",
                  cat_path);
      }
  }

  /* --- Override stage model IIP3/P1dB with catalog datasheet values --- */
  {
      int chain_id;
      for (chain_id = 0; chain_id < STAGE_CHAIN_COUNT; chain_id++) {
          const StageModel *stages = NULL;
          size_t n = stage_models_get(&stage_cfg, (StageChainId)chain_id, &stages);
          size_t i;
          for (i = 0; i < n; i++) {
              /* stages[i] is actually mutable since we own stage_cfg */
              component_catalog_override_stage(
                  &stage_cfg.chains[chain_id][i], &component_catalog);
          }
      }
  }

  /* --- Part E: Cascade Analysis (Friis, IP3, dynamic range, sensitivity) --- */
  {
    CascadeParams    cascade_params;
    CascadeResult    cascade_result;
    int              cascade_ret;

    cascade_params.antenna_temp_k = cfg.antenna_temp_k;     /* 91 K (Viasat 13.5m @ 44° elev, Athens→SES-17) */
    cascade_params.t0_k           = cfg.t0_k;               /* 290 K */
    cascade_params.bw_hz          = B_NOISE_HZ;             /* 200 MHz */
    cascade_params.vpp_out        = 1.0;                    /* 1 Vpp */
    cascade_params.snr_target_db  = cfg.input_snr_db;       /* computed from link budget */
    cascade_params.snr_required_db = 26.5;                  /* 64-APSK required SNR from assignment table */

    cascade_ret = compute_cascade(&stage_cfg, &cascade_params,
                                   STAGE_CHAIN_BASEBAND_RX,
                                   &component_catalog, &cascade_result);
    if (cascade_ret == 0) {
      print_cascade(&cascade_result);
    }
  }

  /* --- Part D: Propagation Analysis & Link Budget (uses cascade sensitivity) --- */
  {
    PropagationScenario prop_scenario;
    LinkBudgetResult    prop_budget;
    CascadeParams       tmp_params;
    CascadeResult       tmp_result;
    double sensitivity_dbm = -70.0; /* fallback if cascade fails */

    /* Run cascade to get real sensitivity */
    tmp_params.antenna_temp_k = cfg.antenna_temp_k;
    tmp_params.t0_k           = cfg.t0_k;
    tmp_params.bw_hz          = B_NOISE_HZ;
    tmp_params.vpp_out        = 1.0;
    tmp_params.snr_target_db  = cfg.input_snr_db;
    tmp_params.snr_required_db = 26.5;

    if (compute_cascade(&stage_cfg, &tmp_params,
                         STAGE_CHAIN_BASEBAND_RX,
                         &component_catalog, &tmp_result) == 0) {
      sensitivity_dbm = tmp_result.sensitivity_dbm;
    }

    prop_scenario.frequency_hz         = cfg.carrier_hz;
    prop_scenario.distance_km          = 36000.0;
    prop_scenario.elevation_deg        = 44.0;                /* Athens → SES-17 (18°E) */
    prop_scenario.polarization_deg     = 45.0;                /* circular, TBD with team */
    prop_scenario.rain_rate_mmh        = 10.0;                /* 0.01% exceedance, generic */
    prop_scenario.surface_temp_k       = 288.15;
    prop_scenario.surface_pressure_hpa = 1013.25;
    prop_scenario.water_vapor_gm3      = 7.5;
    prop_scenario.liquid_water_gm3     = 0.05;                /* medium fog */
    prop_scenario.eirp_dbm             = 85.0;                 /* 32 W + 40 dBi (Beyond Gravity 0.6m sat antenna) */
    prop_scenario.rx_gain_dbi          = 68.70;                /* Viasat 13.5m earth station @ 24 GHz: 67.2+20·log10(24/20.2) */
    prop_scenario.rx_sensitivity_dbm   = sensitivity_dbm;     /* from cascade above */

    compute_link_budget(&prop_scenario, &prop_budget);
    print_link_budget(&prop_budget);

    /* Override simulation input SNR with real link budget received power */
    signal_dbm = prop_budget.rx_power_dbm;
    cfg.input_snr_db = signal_dbm - noise_dbm;

    /* Actual SNR at receiver:
     *   Ni = k · T_ant · B  (noise floor at antenna temperature)
     *   SNR at detector = P_rx − (Ni + NF_total) */
    {
        double ni_dbm = K_BOLTZMANN * cfg.antenna_temp_k * B_NOISE_HZ;
        ni_dbm = lin_to_db(ni_dbm) + 30.0;
        double snr_at_detector = prop_budget.rx_power_dbm
                               - (ni_dbm + tmp_result.total_nf_db);
        print_modcod_table(snr_at_detector);
     }
  }

  /* --- Run Simulation Paths --- */
  if (cfg.run_bb) {
    SimBasebandResult bb_result;
    memset(&bb_result, 0, sizeof(bb_result));
    perf_timer_start(&timer_bb);
    if (simulate_baseband(&cfg, &stage_cfg, constellation, 64u,
                          tx_symbols, (size_t)cfg.symbols, &rng, &bb_result,
                          rf_csv_dir, rf_const_dir, rf_trace_dir) != 0) {
      fprintf(stderr, "Complex-baseband simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 6;
    }
    perf_timer_stop(&timer_bb);
    memcpy(metrics_bb, bb_result.metrics, bb_result.count * sizeof(StageMetric));
    count_bb = bb_result.count;
    final_vpp_bb = bb_result.final_vpp;
  } else {
    (void)metrics_bb;
    (void)count_bb;
    (void)final_vpp_bb;
  }

  if (cfg.run_rf) {
    perf_timer_start(&timer_rf);
    if (simulate_bruteforce_rf(&cfg, &stage_cfg, tx_symbols, constellation, 64u,
                               (size_t)cfg.symbols, metrics_rf, &count_rf,
                               &final_vpp_rf, &rf_sps, &rf_fs_used, rf_csv_dir,
                               rf_const_dir, rf_trace_dir, rf_spectrum_dir) != 0) {
      fprintf(stderr, "Brute-force RF simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 7;
    }
    perf_timer_stop(&timer_rf);

    {
      char rf_metrics_path[512];
      snprintf(rf_metrics_path, sizeof(rf_metrics_path), "%s/rf_metrics.csv", rf_csv_dir);
      if (count_rf > 0u &&
          write_metrics_csv(rf_metrics_path, metrics_rf, count_rf) != 0) {
        fprintf(stderr, "Warning: failed to write aggregate RF metrics CSV '%s'\n",
                rf_metrics_path);
      }
    }
  } else {
    (void)metrics_rf;
    (void)count_rf;
    (void)final_vpp_rf;
    (void)rf_sps;
    (void)rf_fs_used;
    (void)rf_csv_dir;
    (void)rf_const_dir;
    (void)rf_trace_dir;
  }

  if (cfg.run_realistic) {
    perf_timer_start(&timer_realistic);
    if (simulate_realistic_rf(&cfg, &stage_cfg, tx_symbols, constellation, 64u,
                              (size_t)cfg.symbols, metrics_realistic, &count_realistic,
                              &final_vpp_realistic, &realistic_sps, &realistic_fs_used,
                              realistic_csv_dir, realistic_const_dir, realistic_trace_dir,
                              realistic_spectrum_dir) != 0) {
      fprintf(stderr, "Realistic RF simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 9;
    }
    perf_timer_stop(&timer_realistic);

    {
      char realistic_metrics_path[512];
      snprintf(realistic_metrics_path, sizeof(realistic_metrics_path), "%s/realistic_metrics.csv", realistic_csv_dir);
      if (count_realistic > 0u &&
          write_metrics_csv(realistic_metrics_path, metrics_realistic, count_realistic) != 0) {
        fprintf(stderr, "Warning: failed to write aggregate realistic metrics CSV '%s'\n",
                realistic_metrics_path);
      }
    }
  } else {
    (void)metrics_realistic;
    (void)count_realistic;
    (void)final_vpp_realistic;
    (void)realistic_sps;
    (void)realistic_fs_used;
    (void)realistic_csv_dir;
    (void)realistic_const_dir;
    (void)realistic_trace_dir;
  }

  /* --- Transmitter chain simulation --- */
  if (tx_stage_cfg.counts[STAGE_CHAIN_BASEBAND_RX] > 0u) {
    printf("\n  --- Transmitter chain simulation ---\n");
    if (simulate_transmitter(&cfg, &tx_stage_cfg, constellation, 64u,
                             tx_symbols, (size_t)cfg.symbols, &rng,
                             tx_csv_dir, tx_const_dir, tx_trace_dir,
                             tx_spectrum_dir) != 0) {
      fprintf(stderr, "Transmitter simulation failed\n");
    } else {
      printf("  Transmitter artifacts written to %s\n", tx_const_dir);
    }
  }

  /* --- Print console summary --- */
  printf("seed=%u\n", cfg.seed);
  printf("Stage CSV: %s\n", resolved_stage_csv_path);
  printf("Input budget: Noise=%.4f dBm, Signal=%.4f dBm (SNR=%.2f dB)\n",
         noise_dbm, signal_dbm, cfg.input_snr_db);
  printf("Paths: baseband=%s, rf_baseline=%s, realistic=%s\n",
         cfg.run_bb ? "ON" : "OFF",
         cfg.run_rf ? "ON" : "OFF",
         cfg.run_realistic ? "ON" : "OFF");

  if (cfg.run_bb && count_bb > 0) {
    print_metrics("Complex Baseband Stages Metrics", metrics_bb, count_bb);
  }
  if (cfg.run_rf && count_rf > 0) {
    print_metrics("RF Brute-Force Stages Metrics (SNR/EVM after each stage)",
                  metrics_rf, count_rf);
  }
  if (cfg.run_realistic && count_realistic > 0) {
    print_metrics("Realistic RF Stages Metrics (with impairments)",
                  metrics_realistic, count_realistic);
  }

  if (cfg.run_rf && cfg.run_realistic && count_rf > 0 && count_realistic > 0) {
    printf("\n=== RF Baseline vs Realistic Path Comparison ===\n");
    printf("%-20s %12s %12s %12s %12s\n", "Stage", "RF SNR(dB)", "RF EVM(%)", "Real SNR(dB)", "Real EVM(%)");
    printf("%-20s %12s %12s %12s %12s\n", "--------------------", "------------", "------------", "------------", "------------");
    {
      size_t ri = 0, si = 0;
      while (ri < count_rf || si < count_realistic) {
        const char *stage_name = NULL;
        if (ri < count_rf) stage_name = metrics_rf[ri].stage;
        if (si < count_realistic && (stage_name == NULL || strcmp(stage_name, metrics_realistic[si].stage) > 0))
          stage_name = metrics_realistic[si].stage;

        double rf_snr = ri < count_rf && strcmp(metrics_rf[ri].stage, stage_name) == 0 ? metrics_rf[ri].snr_db : 0.0;
        double rf_evm = ri < count_rf && strcmp(metrics_rf[ri].stage, stage_name) == 0 ? metrics_rf[ri].evm_pct : 0.0;
        double rl_snr = si < count_realistic && strcmp(metrics_realistic[si].stage, stage_name) == 0 ? metrics_realistic[si].snr_db : 0.0;
        double rl_evm = si < count_realistic && strcmp(metrics_realistic[si].stage, stage_name) == 0 ? metrics_realistic[si].evm_pct : 0.0;

        printf("%-20s %12.2f %12.2f %12.2f %12.2f\n", stage_name, rf_snr, rf_evm, rl_snr, rl_evm);

        if (ri < count_rf && strcmp(metrics_rf[ri].stage, stage_name) == 0) ri++;
        if (si < count_realistic && strcmp(metrics_realistic[si].stage, stage_name) == 0) si++;
      }
    }
  }

  printf("\n=== PERFORMANCE TIMING SUMMARY ===\n");
  if (cfg.run_bb) perf_timer_report("Complex Baseband Path", &timer_bb, (size_t)cfg.symbols);
  if (cfg.run_rf) perf_timer_report("RF Brute-Force Path", &timer_rf, (size_t)cfg.symbols);
  if (cfg.run_realistic) perf_timer_report("Realistic RF Path", &timer_realistic, (size_t)cfg.symbols);
  printf("==================================\n");

  printf("\nOutputs written under ./out/\n");
  printf("Generated files strictly follow the assignment requirements (Trace, "
         "Constellation, SNR, EVM).\n");

  /* --- Cleanup --- */
  stage_models_free(&stage_cfg);
  stage_models_free(&tx_stage_cfg);
  free(tx_symbols);
  free(labels);
  return 0; /* Success! */
}
