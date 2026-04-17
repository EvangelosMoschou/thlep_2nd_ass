/*
 * ============================================================================
 * stage_artifacts.c — Visualization & Data Export for Receiver Simulation
 * ============================================================================
 *
 * PURPOSE:
 *   This file generates all the output files that the simulator produces.
 *   After each stage of the receiver processes the signal, this module
 *   writes out two types of artifacts:
 *
 *   1. CSV DATA FILES — Machine-readable spreadsheet data containing:
 *      - Constellation diagrams (I/Q coordinates of reference vs. received symbols)
 *      - Signal traces (time-domain waveforms for RF signals)
 *      - Stage metrics tables (SNR, EVM at each stage)
 *      - Input link budget summary (carrier frequency, noise power, etc.)
 *
 *   2. SVG VECTOR GRAPHICS — Human-viewable charts and diagrams:
 *      - Constellation scatter plots (showing how noise distorts symbols)
 *      - Time-domain waveform overlay plots (reference vs. received)
 *      - Metrics summary tables (rendered as formatted SVG text)
 *      - Link budget parameter cards
 *
 * WHY SVG?
 *   SVG (Scalable Vector Graphics) is an XML-based image format that:
 *   - Can be opened in any web browser (no special software needed)
 *   - Scales perfectly to any zoom level (vector graphics, not pixels)
 *   - Is human-readable XML text (can be inspected with a text editor)
 *   - Doesn't require any external plotting library (generated purely in C)
 *
 * FILE NAMING CONVENTION:
 *   Output files follow a structured naming pattern:
 *     {prefix}_input.csv             — Input signal before any processing
 *     {prefix}_stage_01_{name}.csv   — Signal after stage 1
 *     {prefix}_stage_02_{name}.csv   — Signal after stage 2
 *     ...etc...
 *
 *   SVG filenames additionally include metric tags:
 *     {prefix}_stage_01_{name}_snr_20p00db_evm_3p15pct.svg
 *   This embeds the SNR and EVM values directly in the filename for
 *   quick identification when browsing output directories.
 *
 * HELPER FUNCTIONS:
 *   This file also contains several text-processing utilities for generating
 *   human-readable stage names, URL-safe file slugs, and metric tags.
 *
 * ============================================================================
 */

#include <ctype.h>     /* For toupper(), tolower(), isalnum(), isdigit() — character classification */
#include <float.h>     /* For DBL_MAX, DBL_MIN — double-precision extremes for finding min/max */
#include <math.h>      /* For fmin(), fmax(), isfinite(), sqrt() — math operations */
#include <stdio.h>     /* For FILE*, fopen(), fprintf(), fclose(), snprintf() — file I/O */
#include <string.h>    /* For strcmp(), strlen(), memcpy() — string operations */
#include <stdlib.h>    /* For malloc(), free(), calloc() */

#include "stage_artifacts.h"   /* Public interface: write_*() function declarations, type imports */
#include "prng.h"      /* For prng_gauss() */


/* ============================================================================
 * STAGE NAME HUMANIZATION
 * ============================================================================
 *
 * The internal stage names are terse (e.g., "rf_bpf_eq", "bb_amp_1vpp").
 * For chart titles and human-readable output, we convert them to prettier
 * forms like "RF BPF EQ" or "Baseband Amp 1 Vpp".
 *
 * The process works in two levels:
 *   1. humanize_stage_token() — converts a single word/token
 *   2. humanize_stage_name()  — splits a name on underscores and
 *      humanizes each token, then joins with spaces
 * ============================================================================ */

/*
 * humanize_stage_token — Convert a single word token to its human-readable form
 *
 * What it does:
 *   Takes a single "token" (a word between underscores in a stage name) and
 *   converts it to a nicely formatted version. The function has a dictionary
 *   of known abbreviations and special cases:
 *
 *   Known abbreviations (returned as ALL-CAPS):
 *     "bb" → "BB"         (Baseband)
 *     "rf" → "RF"         (Radio Frequency)
 *     "lna" → "LNA"       (Low-Noise Amplifier)
 *     "bpf" → "BPF"       (Band-Pass Filter)
 *     "lpf" → "LPF"       (Low-Pass Filter)
 *     "snr" → "SNR"       (Signal-to-Noise Ratio)
 *     "evm" → "EVM"       (Error Vector Magnitude)
 *     "adc" → "ADC"       (Analog-to-Digital Converter)
 *     "dac" → "DAC"       (Digital-to-Analog Converter)
 *     "iq" → "IQ"         (In-phase / Quadrature)
 *     "lo" → "LO"         (Local Oscillator)
 *
 *   Known words (returned with specific capitalization):
 *     "vpp" → "Vpp"       (Volts peak-to-peak)
 *     "input" → "Input"
 *     "mixer" → "Mixer"
 *     "amp" → "Amp"       (Amplifier)
 *     "eq" → "EQ"         (Equalizer)
 *     "downconv" → "Downconversion"
 *     "1vpp" → "1 Vpp"
 *
 *   All-digit tokens: returned as-is (e.g., "5" → "5")
 *   Short tokens (≤3 chars): returned as ALL-CAPS (e.g., "adc" → "ADC")
 *   Other tokens: returned Title-Cased (first letter uppercase, rest lowercase)
 *
 * Parameters:
 *   token    — The input word to humanize (e.g., "bpf", "amp", "filter")
 *   out      — Buffer to write the humanized version into
 *   out_size — Size of the output buffer
 */
static void humanize_stage_token(const char* token, char* out, size_t out_size) {
    size_t i;
    size_t len;

    /* Guard against NULL or empty output buffer */
    if (!out || out_size == 0u) {
        return;
    }

    out[0] = '\0';
    if (!token || token[0] == '\0') {
        return;
    }

    /* --- Dictionary of known abbreviations --- */

    /* Two-letter engineering abbreviations → ALL-CAPS */
    if (strcmp(token, "bb") == 0) {
        snprintf(out, out_size, "BB");       /* BB = Baseband */
        return;
    }
    if (strcmp(token, "rf") == 0) {
        snprintf(out, out_size, "RF");       /* RF = Radio Frequency */
        return;
    }
    if (strcmp(token, "lna") == 0) {
        snprintf(out, out_size, "LNA");      /* LNA = Low-Noise Amplifier */
        return;
    }
    if (strcmp(token, "bpf") == 0) {
        snprintf(out, out_size, "BPF");      /* BPF = Band-Pass Filter */
        return;
    }
    if (strcmp(token, "lpf") == 0) {
        snprintf(out, out_size, "LPF");      /* LPF = Low-Pass Filter */
        return;
    }
    if (strcmp(token, "snr") == 0) {
        snprintf(out, out_size, "SNR");      /* SNR = Signal-to-Noise Ratio */
        return;
    }
    if (strcmp(token, "evm") == 0) {
        snprintf(out, out_size, "EVM");      /* EVM = Error Vector Magnitude */
        return;
    }
    if (strcmp(token, "vpp") == 0) {
        snprintf(out, out_size, "Vpp");      /* Vpp = Volts peak-to-peak */
        return;
    }
    if (strcmp(token, "input") == 0) {
        snprintf(out, out_size, "Input");
        return;
    }
    if (strcmp(token, "mixer") == 0) {
        snprintf(out, out_size, "Mixer");
        return;
    }
    if (strcmp(token, "amp") == 0) {
        snprintf(out, out_size, "Amp");      /* Amp = Amplifier */
        return;
    }
    if (strcmp(token, "eq") == 0) {
        snprintf(out, out_size, "EQ");       /* EQ = Equalizer */
        return;
    }
    if (strcmp(token, "adc") == 0) {
        snprintf(out, out_size, "ADC");      /* ADC = Analog-to-Digital Converter */
        return;
    }
    if (strcmp(token, "dac") == 0) {
        snprintf(out, out_size, "DAC");      /* DAC = Digital-to-Analog Converter */
        return;
    }
    if (strcmp(token, "iq") == 0) {
        snprintf(out, out_size, "IQ");       /* IQ = In-phase/Quadrature */
        return;
    }
    if (strcmp(token, "lo") == 0) {
        snprintf(out, out_size, "LO");       /* LO = Local Oscillator */
        return;
    }
    if (strcmp(token, "downconv") == 0) {
        snprintf(out, out_size, "Downconversion");
        return;
    }
    if (strcmp(token, "1vpp") == 0) {
        snprintf(out, out_size, "1 Vpp");    /* 1 Vpp = 1 Volt peak-to-peak */
        return;
    }

    /* --- Fallback rules for unknown tokens --- */

    len = strlen(token);
    if (len > 0u) {
        /*
         * Check if the entire token is numeric (e.g., "5", "10").
         * If so, return it unchanged.
         */
        int all_digits = 1;
        for (i = 0u; i < len; ++i) {
            if (!isdigit((unsigned char)token[i])) {
                all_digits = 0;
                break;
            }
        }

        if (all_digits) {
            snprintf(out, out_size, "%s", token);
            return;
        }

        /*
         * Short tokens (3 chars or less, e.g., "agc") → ALL-CAPS
         * This is a heuristic: most 2-3 letter engineering terms are acronyms.
         */
        if (len <= 3u) {
            size_t j;
            for (j = 0u; j < len && j + 1u < out_size; ++j) {
                out[j] = (char)toupper((unsigned char)token[j]);
            }
            out[j] = '\0';
            return;
        }

        /*
         * Longer tokens → Title Case (first letter upper, rest lower)
         * e.g., "filter" → "Filter", "amplitude" → "Amplitude"
         */
        out[0] = (char)toupper((unsigned char)token[0]);
        for (i = 1u; i < len && i + 1u < out_size; ++i) {
            out[i] = (char)tolower((unsigned char)token[i]);
        }
        out[i < out_size ? i : out_size - 1u] = '\0';
    }
}


/*
 * humanize_stage_name — Convert a full internal stage name to human-readable form
 *
 * What it does:
 *   Takes a complete stage name like "rf_bpf_eq" and converts it to
 *   "RF BPF EQ" by:
 *   1. Checking for exact-match overrides (common full names)
 *   2. If no exact match, splitting on underscores/hyphens/spaces
 *   3. Humanizing each token individually via humanize_stage_token()
 *   4. Joining the humanized tokens with spaces
 *
 * Exact-match overrides (for common compound names):
 *   "input"          → "Input"
 *   "input_rf"       → "RF Input"
 *   "rf_bpf_eq"      → "RF BPF EQ"
 *   "rf_bpf"         → "RF BPF"
 *   "mixer_downconv"  → "Mixer Downconversion"
 *   "bb_lpf"         → "Baseband LPF"
 *   "bb_amp_1vpp"    → "Baseband Amp 1 Vpp"
 *
 * Parameters:
 *   raw      — The internal stage name string
 *   out      — Buffer for the humanized result
 *   out_size — Size of the output buffer
 */
void humanize_stage_name(const char* raw, char* out, size_t out_size) {
    size_t out_len = 0u;    /* Current write position in the output buffer */
    const char* p;           /* Scanning pointer through the raw name */

    if (!out || out_size == 0u) {
        return;
    }

    out[0] = '\0';
    if (!raw || raw[0] == '\0') {
        return;
    }

    /* --- Check for exact-match overrides first --- */

    if (strcmp(raw, "input") == 0) {
        snprintf(out, out_size, "Input");
        return;
    }
    if (strcmp(raw, "input_rf") == 0) {
        snprintf(out, out_size, "RF Input");
        return;
    }
    if (strcmp(raw, "rf_bpf_eq") == 0) {
        snprintf(out, out_size, "RF BPF EQ");
        return;
    }
    if (strcmp(raw, "rf_bpf") == 0) {
        snprintf(out, out_size, "RF BPF");
        return;
    }
    if (strcmp(raw, "mixer_downconv") == 0) {
        snprintf(out, out_size, "Mixer Downconversion");
        return;
    }
    if (strcmp(raw, "bb_lpf") == 0) {
        snprintf(out, out_size, "Baseband LPF");
        return;
    }
    if (strcmp(raw, "bb_amp_1vpp") == 0) {
        snprintf(out, out_size, "Baseband Amp 1 Vpp");
        return;
    }

    /* --- Generic tokenize-and-humanize path --- */

    p = raw;
    while (*p != '\0' && out_len + 1u < out_size) {
        char token[64];      /* Buffer for one token (word between delimiters) */
        char pretty[64];     /* Humanized version of the token */
        size_t token_len = 0u;
        size_t k;

        /* Skip delimiter characters (underscores, hyphens, spaces) */
        while (*p == '_' || *p == '-' || *p == ' ') {
            ++p;
        }
        if (*p == '\0') {
            break;
        }

        /* Extract one token (sequence of non-delimiter characters) */
        while (*p != '\0' && *p != '_' && *p != '-' && *p != ' ' && token_len + 1u < sizeof(token)) {
            token[token_len++] = *p;
            ++p;
        }
        token[token_len] = '\0';

        /* Humanize this individual token */
        humanize_stage_token(token, pretty, sizeof(pretty));
        if (pretty[0] == '\0') {
            continue;
        }

        /* Add a space separator between tokens (but not before the first one) */
        if (out_len > 0u && out_len + 1u < out_size) {
            out[out_len++] = ' ';
        }

        /* Copy the humanized token into the output buffer */
        for (k = 0u; pretty[k] != '\0' && out_len + 1u < out_size; ++k) {
            out[out_len++] = pretty[k];
        }
    }

    out[out_len] = '\0';

    /* If nothing was written (e.g., empty or all-delimiter input), use the raw name */
    if (out[0] == '\0') {
        snprintf(out, out_size, "%s", raw);
    }
}


/* ============================================================================
 * FILENAME GENERATION HELPERS
 * ============================================================================ */

/*
 * slugify_text — Convert human-readable text to a filesystem-safe slug
 *
 * What it does:
 *   Converts a string like "RF BPF EQ" into "rf_bpf_eq" — a lowercase,
 *   underscore-separated string safe for use in filenames and URLs.
 *
 * Rules:
 *   - Alphanumeric characters → kept (lowercased)
 *   - Non-alphanumeric characters → replaced with underscore
 *   - Consecutive non-alphanumeric characters → single underscore (no doubles)
 *   - Trailing underscores → removed
 *   - Empty result → defaults to "stage"
 *
 * Parameters:
 *   text     — The input text to slugify
 *   out      — Buffer for the slugified result
 *   out_size — Size of the output buffer
 *
 * Example:
 *   "Mixer Downconversion" → "mixer_downconversion"
 *   "RF BPF EQ"            → "rf_bpf_eq"
 */
static void slugify_text(const char* text, char* out, size_t out_size) {
    size_t i;
    size_t j = 0u;
    int last_was_sep = 1;   /* Start as "true" to avoid leading underscore */

    if (!out || out_size == 0u) {
        return;
    }

    out[0] = '\0';
    if (!text || text[0] == '\0') {
        return;
    }

    for (i = 0u; text[i] != '\0' && j + 1u < out_size; ++i) {
        const unsigned char c = (unsigned char)text[i];
        if (isalnum(c)) {
            out[j++] = (char)tolower(c);
            last_was_sep = 0;
        } else if (!last_was_sep) {
            /* Replace non-alphanumeric with underscore (only if last char wasn't also a separator) */
            out[j++] = '_';
            last_was_sep = 1;
        }
        /* If last_was_sep is already 1, skip this non-alphanumeric char (avoid double underscores) */
    }

    /* Remove trailing underscores */
    while (j > 0u && out[j - 1u] == '_') {
        --j;
    }
    out[j] = '\0';

    /* Default to "stage" if the result is empty */
    if (out[0] == '\0') {
        snprintf(out, out_size, "stage");
    }
}


/*
 * format_decimal_tag — Format a floating-point number into a filename-safe tag
 *
 * What it does:
 *   Converts a decimal number into a string where:
 *   - The minus sign '-' is replaced with 'm' (for "minus")
 *   - The decimal point '.' is replaced with 'p' (for "point")
 *
 *   This makes the number safe to embed in filenames (some systems don't
 *   allow certain characters in filenames).
 *
 * Examples:
 *   20.15  → "20p15"
 *   -3.50  → "m3p50"
 *   0.00   → "0p00"
 *
 * Parameters:
 *   value    — The floating-point value to format
 *   out      — Buffer for the formatted result
 *   out_size — Size of the output buffer
 */
static void format_decimal_tag(double value, char* out, size_t out_size) {
    char tmp[64];           /* Temporary buffer for sprintf */
    size_t i;
    size_t j = 0u;          /* Write position in output */

    if (!out || out_size == 0u) {
        return;
    }

    out[0] = '\0';
    if (!isfinite(value)) {  /* Don't format infinity or NaN */
        return;
    }

    /* Format the number with 2 decimal places */
    snprintf(tmp, sizeof(tmp), "%.2f", value);

    /* Character-by-character replacement */
    for (i = 0u; tmp[i] != '\0' && j + 1u < out_size; ++i) {
        if (tmp[i] == '-') {
            out[j++] = 'm';     /* Replace minus with 'm' */
        } else if (tmp[i] == '.') {
            out[j++] = 'p';     /* Replace decimal point with 'p' */
        } else {
            out[j++] = tmp[i];  /* Keep digits as-is */
        }
    }
    out[j] = '\0';
}


/*
 * format_metric_suffix — Create a human-readable metric label for chart titles
 *
 * What it does:
 *   Builds a string like " | SNR 20.15 dB | EVM 3.50%" that can be appended
 *   to a chart title to show the current signal quality metrics.
 *
 * Parameters:
 *   metric   — The stage metric containing SNR and EVM values
 *   out      — Buffer for the formatted suffix string
 *   out_size — Size of the output buffer
 */
static void format_metric_suffix(const StageMetric* metric, char* out, size_t out_size) {
    if (!out || out_size == 0u) {
        return;
    }

    out[0] = '\0';
    if (!metric) {
        return;
    }

    /* Append SNR if it's a valid number (not infinity or NaN) */
    if (isfinite(metric->snr_db)) {
        size_t len = strlen(out);
        snprintf(out + len, out_size - len, " | SNR %.2f dB", metric->snr_db);
    }

    /* Append EVM if it's a valid number */
    if (isfinite(metric->evm_pct)) {
        size_t len = strlen(out);
        snprintf(out + len, out_size - len, " | EVM %.2f%%", metric->evm_pct);
    }
}


/*
 * format_metric_tag — Create a filename-safe metric tag for SVG file names
 *
 * What it does:
 *   Builds a string like "_snr_20p15db_evm_3p50pct" that can be embedded
 *   in output filenames to identify the signal quality at that point.
 *
 * Parameters:
 *   metric   — The stage metric containing SNR and EVM values
 *   out      — Buffer for the formatted tag string
 *   out_size — Size of the output buffer
 */
static void format_metric_tag(const StageMetric* metric, char* out, size_t out_size) {
    if (!out || out_size == 0u) {
        return;
    }

    out[0] = '\0';
    if (!metric) {
        return;
    }

    /* Append SNR tag if valid */
    if (isfinite(metric->snr_db)) {
        char snr_tag[32];
        format_decimal_tag(metric->snr_db, snr_tag, sizeof(snr_tag));
        snprintf(out + strlen(out), out_size - strlen(out), "_snr_%sdb", snr_tag);
    }

    /* Append EVM tag if valid */
    if (isfinite(metric->evm_pct)) {
        char evm_tag[32];
        format_decimal_tag(metric->evm_pct, evm_tag, sizeof(evm_tag));
        snprintf(out + strlen(out), out_size - strlen(out), "_evm_%spct", evm_tag);
    }
}


/* ============================================================================
 * CSV DATA OUTPUT FUNCTIONS
 * ============================================================================ */

/*
 * write_constellation_csv — Save constellation diagram data to a CSV file
 *
 * What it does:
 *   Writes a CSV file containing the I/Q coordinates of both the reference
 *   (ideal, noise-free) constellation points and the received (noisy) points.
 *   This data can be imported into MATLAB, Python, Excel, etc. to create
 *   custom constellation diagram plots.
 *
 * Output format:
 *   idx, ref_i, ref_q, sig_i, sig_q
 *   0, 0.707106781187, 0.707106781187, 0.715234521983, 0.698765432101
 *   1, -0.707106781187, 0.707106781187, -0.702345678901, 0.712345678901
 *   ...
 *
 *   Where:
 *     idx   — Symbol index (0, 1, 2, ...)
 *     ref_i — Reference (clean) In-phase component
 *     ref_q — Reference (clean) Quadrature component
 *     sig_i — Received (noisy) In-phase component
 *     sig_q — Received (noisy) Quadrature component
 *
 * Parameters:
 *   path — File path to write (e.g., "out/topology_sim_1/csv/baseband_input.csv")
 *   ref  — Array of reference constellation points (Complex structs)
 *   sig  — Array of received signal points (Complex structs)
 *   n    — Number of symbols
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
static int write_constellation_csv(const char* path, const Complex* ref, const Complex* sig, size_t n) {
    FILE* f;
    size_t i;

    if (!path || !ref || !sig || n == 0u) {
        return -1;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    /* Write CSV header */
    fprintf(f, "idx,ref_i,ref_q,sig_i,sig_q\n");

    /* Write one row per symbol with 12 decimal digits of precision */
    for (i = 0; i < n; ++i) {
        fprintf(f, "%zu,%.12f,%.12f,%.12f,%.12f\n", i, ref[i].re, ref[i].im, sig[i].re, sig[i].im);
    }

    fclose(f);
    return 0;
}


/*
 * write_real_trace_csv — Save real-valued time-domain signal traces to CSV
 *
 * What it does:
 *   Writes a CSV file containing the reference and received real-valued signals.
 *   Used for RF waveform debugging — the RF path uses real (not complex) signals
 *   sampled at very high rates (e.g., 96 GHz).
 *
 * Output limiting:
 *   The RF signal can have millions of samples (e.g., 256 symbols × 9600 samples/symbol
 *   = 2.46 million samples). Writing all of them would create enormous files.
 *   The `max_points` parameter limits output to a manageable number (typically 6000),
 *   which is enough for visual inspection without bloating disk usage.
 *
 * Parameters:
 *   path       — File path to write
 *   ref        — Reference (clean) real signal array
 *   sig        — Received (noisy) real signal array
 *   n          — Total number of samples
 *   max_points — Maximum number of samples to write (truncation limit)
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
static int write_real_trace_csv(const char* path, const double* ref, const double* sig, size_t n, size_t max_points) {
    FILE* f;
    size_t i;
    size_t nout;   /* Actual number of points to write */

    if (!path || !ref || !sig || n == 0u) {
        return -1;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    /* Limit output to max_points */
    nout = (n < max_points) ? n : max_points;

    fprintf(f, "idx,ref,sig\n");
    for (i = 0; i < nout; ++i) {
        fprintf(f, "%zu,%.12f,%.12f\n", i, ref[i], sig[i]);
    }

    fclose(f);
    return 0;
}


/*
 * write_metrics_csv — Save per-stage performance metrics to a CSV file
 *
 * What it does:
 *   Writes a summary table showing signal quality at each receiver stage.
 *   This is the primary output for link budget analysis — it shows how
 *   SNR degrades and EVM grows as the signal passes through each component.
 *
 * Output format:
 *   stage, domain, signal_power, noise_power, snr_db, evm_pct
 *   input, complex_baseband, 1.000000e+00, 1.234567e-02, 19.085, 11.11
 *   lna, complex_baseband, 3.162278e+02, 4.567890e+00, 18.402, 12.01
 *   ...
 *
 * Parameters:
 *   path    — File path to write
 *   metrics — Array of StageMetric structs (one per stage)
 *   count   — Number of stages
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
int write_metrics_csv(const char* path, const StageMetric* metrics, size_t count) {
    FILE* f;
    size_t i;

    if (!path || !metrics || count == 0u) {
        return -1;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    /* CSV header */
    fprintf(f, "stage,domain,signal_power,noise_power,snr_db,evm_pct\n");

    /* One row per stage with scientific notation for power values */
    for (i = 0; i < count; ++i) {
        fprintf(
            f,
            "%s,%s,%.12e,%.12e,%.6f,%.6f\n",
            metrics[i].stage,
            metrics[i].domain,
            metrics[i].signal_power,
            metrics[i].noise_power,
            metrics[i].snr_db,
            metrics[i].evm_pct);
    }

    fclose(f);
    return 0;
}


/*
 * write_input_budget_csv — Save the input link budget summary to CSV
 *
 * What it does:
 *   Writes a single-row CSV file containing all the top-level simulation
 *   parameters: carrier frequency, symbol rate, noise temperature,
 *   calculated noise power, signal power, etc.
 *
 *   This documents the exact configuration used for a simulation run,
 *   making it possible to reproduce results later.
 *
 * Parameters:
 *   path         — File path to write
 *   cfg          — Simulation configuration struct
 *   noise_w      — Calculated thermal noise power in watts
 *   noise_dbm    — Noise power in dBm (decibels relative to 1 milliwatt)
 *   signal_dbm   — Signal power in dBm
 *   bandwidth_hz — Noise bandwidth in Hz
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
int write_input_budget_csv(const char* path, const SimConfig* cfg, double noise_w, double noise_dbm, double signal_dbm, double bandwidth_hz) {
    FILE* f;

    if (!path || !cfg) {
        return -1;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    fprintf(f, "carrier_hz,symbol_rate_hz,rolloff,noise_bandwidth_hz,antenna_temp_k,input_snr_db,noise_power_w,noise_power_dbm,signal_power_dbm\n");
    fprintf(
        f,
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.12e,%.6f,%.6f\n",
        cfg->carrier_hz,
        cfg->symbol_rate_hz,
        cfg->rolloff,
        bandwidth_hz,
        cfg->antenna_temp_k,
        cfg->input_snr_db,
        noise_w,
        noise_dbm,
        signal_dbm);

    fclose(f);
    return 0;
}


/* ============================================================================
 * SVG VISUALIZATION FUNCTIONS
 * ============================================================================
 *
 * These functions generate SVG (Scalable Vector Graphics) files that can be
 * opened in any web browser. Each function writes raw SVG XML directly to a
 * file using fprintf() — no external charting library is needed.
 *
 * Color scheme (consistent across all charts):
 *   Blue (#1d4ed8)   — Reference / ideal signal
 *   Orange (#f97316)  — Received / actual signal
 *   Background (#f8fafc) — Light gray page background
 *   Grid lines (#e5e7eb) — Light gray
 *   Text (#111827)    — Near-black
 *   Axis labels (#475569) — Medium gray
 * ============================================================================ */

/*
 * write_constellation_svg — Generate a constellation scatter plot as SVG
 *
 * What it does:
 *   Creates an SVG image showing a 2D scatter plot of I/Q constellation points.
 *   Two layers of dots are drawn:
 *   - Blue circles: reference (ideal) constellation points
 *   - Orange circles: received (noisy) constellation points
 *
 *   The displacement between each blue-orange pair visualizes the error caused
 *   by noise and stage imperfections. More spread = worse signal quality.
 *
 * Layout:
 *   - 980×760 pixel canvas with margins (80 left, 30 right, 50 top, 80 bottom)
 *   - X-axis: In-phase component (I) — the "horizontal" part of the signal
 *   - Y-axis: Quadrature component (Q) — the "vertical" part of the signal
 *   - Auto-scaled axes with 8% padding to prevent clipping at edges
 *   - 6 grid lines on each axis for visual reference
 *   - Legend showing "Reference" (blue) and "Received" (orange)
 *
 * Parameters:
 *   path  — SVG file path to write
 *   ref   — Array of reference I/Q points (Complex structs)
 *   sig   — Array of received I/Q points
 *   n     — Number of symbols
 *   title — Chart title string (displayed at top of the image)
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
static int write_constellation_svg(
    const char* path,
    const Complex* constellation_template,
    size_t constellation_count,
    const Complex* ref,
    const Complex* sig,
    size_t n,
    const char* title) {
    FILE* f;
    size_t i;

    /* Extreme values for auto-scaling the axes */
    double xmin = DBL_MAX;     /* Will be updated to the minimum I value */
    double xmax = -DBL_MAX;    /* Will be updated to the maximum I value */
    double ymin = DBL_MAX;     /* Will be updated to the minimum Q value */
    double ymax = -DBL_MAX;    /* Will be updated to the maximum Q value */

    /* Canvas dimensions in pixels */
    const int width = 980;
    const int height = 760;

    /* Margins: left, right, top, bottom */
    const int ml = 80;    /* Left margin — room for Y-axis labels */
    const int mr = 30;    /* Right margin */
    const int mt = 50;    /* Top margin — room for title */
    const int mb = 80;    /* Bottom margin — room for X-axis labels */

    /* Plot area dimensions (canvas minus margins) */
    const double plot_w = (double)(width - ml - mr);
    const double plot_h = (double)(height - mt - mb);

    /* Computed scaling variables */
    double span_x;       /* Data range in X (xmax - xmin) */
    double span_y;       /* Data range in Y */
    double scale;        /* Pixels per data unit (uniform for both axes to maintain aspect ratio) */
    double scaled_w;     /* Actual plot width in pixels after uniform scaling */
    double scaled_h;     /* Actual plot height in pixels after uniform scaling */
    double x_origin;     /* Pixel X position of data xmin */
    double y_origin;     /* Pixel Y position of data ymin */

    if (!path || !ref || !sig || n == 0u) {
        return -1;
    }

    /* --- Step 1: Find data ranges for auto-scaling --- */
    for (i = 0u; i < n; ++i) {
        xmin = fmin(xmin, fmin(ref[i].re, sig[i].re));
        xmax = fmax(xmax, fmax(ref[i].re, sig[i].re));
        ymin = fmin(ymin, fmin(ref[i].im, sig[i].im));
        ymax = fmax(ymax, fmax(ref[i].im, sig[i].im));
    }
    for (i = 0u; constellation_template && i < constellation_count; ++i) {
        xmin = fmin(xmin, constellation_template[i].re);
        xmax = fmax(xmax, constellation_template[i].re);
        ymin = fmin(ymin, constellation_template[i].im);
        ymax = fmax(ymax, constellation_template[i].im);
    }

    span_x = xmax - xmin;
    span_y = ymax - ymin;

    /* Prevent division by zero for degenerate data */
    if (span_x <= 0.0) {
        span_x = 1.0;
    }
    if (span_y <= 0.0) {
        span_y = 1.0;
    }

    /* Add 8% padding around the data to prevent clipping at edges */
    {
        const double pad = 0.08 * fmax(span_x, span_y);
        xmin -= pad;
        xmax += pad;
        ymin -= pad;
        ymax += pad;
    }

    /* Recalculate spans after padding */
    span_x = xmax - xmin;
    span_y = ymax - ymin;

    /*
     * Compute UNIFORM scale factor (same for both axes).
     * This preserves the aspect ratio of the constellation — a circle in
     * data space should look like a circle on screen, not an ellipse.
     */
    scale = fmin(plot_w / span_x, plot_h / span_y);
    scaled_w = span_x * scale;
    scaled_h = span_y * scale;

    /* Center the scaled plot within the available area */
    x_origin = (double)ml + (plot_w - scaled_w) * 0.5;
    y_origin = (double)(height - mb) - (plot_h - scaled_h) * 0.5;

    /* --- Step 2: Open the file and write SVG header --- */
    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    /* SVG document root with dimensions and viewBox */
    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);

    /* Background rectangle (light gray) */
    fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");

    /* Title text */
    fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">%s</text>\n", 80, title ? title : "64-APSK Constellation");

    /* Plot area background (white rectangle with gray border) */
    fprintf(f, "<rect x=\"%.2f\" y=\"%.2f\" width=\"%.2f\" height=\"%.2f\" fill=\"white\" stroke=\"#d1d5db\"/>\n", (double)ml, (double)mt, plot_w, plot_h);

    /* --- Step 3: Draw grid lines and axis labels --- */
    for (i = 0u; i <= 6u; ++i) {
        /* Calculate data values at 7 evenly-spaced grid positions */
        const double xv = xmin + (span_x * (double)i / 6.0);
        const double yv = ymin + (span_y * (double)i / 6.0);

        /* Convert data coordinates to pixel coordinates */
        const double x = x_origin + (xv - xmin) * scale;
        const double y = y_origin - (yv - ymin) * scale;

        /* Vertical grid line */
        fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, (double)mt, x, (double)(height - mb));

        /* Horizontal grid line */
        fprintf(f, "<line x1=\"%d\" y1=\"%.2f\" x2=\"%d\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", ml, y, width - mr, y);

        /* X-axis tick label (below the plot area) */
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.2f</text>\n", x, height - mb + 18.0, xv);

        /* Y-axis tick label (to the left of the plot area) */
        fprintf(f, "<text x=\"%d\" y=\"%.2f\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.2f</text>\n", ml - 8, y + 4.0, yv);
    }

    /* X-axis title */
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">In-phase (I)</text>\n", (double)(ml + (width - ml - mr) / 2), (double)(height - 28));

    /* Y-axis title (rotated 90° counterclockwise) */
    fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Quadrature (Q)</text>\n", (double)(mt + plot_h * 0.5));

    /* --- Step 4: Draw legend --- */
    /* Blue circle + "Reference" label */
    fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"5\" fill=\"#1d4ed8\" fill-opacity=\"0.55\" stroke=\"#1d4ed8\" stroke-width=\"0.8\"/>\n", (double)ml + 18.0, (double)mt + 20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Reference</text>\n", (double)ml + 32.0, (double)mt + 24.0);

    /* Orange circle + "Received" label */
    fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"5\" fill=\"#f97316\" fill-opacity=\"0.55\" stroke=\"#f97316\" stroke-width=\"0.8\"/>\n", (double)ml + 118.0, (double)mt + 20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Received</text>\n", (double)ml + 132.0, (double)mt + 24.0);

    /* --- Step 5: Draw data points --- */
    if (constellation_template && constellation_count > 0u) {
        for (i = 0u; i < constellation_count; ++i) {
            const double gx = x_origin + (constellation_template[i].re - xmin) * scale;
            const double gy = y_origin - (constellation_template[i].im - ymin) * scale;
            fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"2.8\" fill=\"#94a3b8\" fill-opacity=\"0.18\" stroke=\"#64748b\" stroke-opacity=\"0.28\" stroke-width=\"0.4\"/>\n", gx, gy);
        }
    }

    for (i = 0u; i < n; ++i) {
        /* Convert reference point's data coordinates to pixel coordinates */
        const double rx = x_origin + (ref[i].re - xmin) * scale;
        const double ry = y_origin - (ref[i].im - ymin) * scale;

        /* Convert received point's data coordinates to pixel coordinates */
        const double sx = x_origin + (sig[i].re - xmin) * scale;
        const double sy = y_origin - (sig[i].im - ymin) * scale;

        /* Blue dot for reference point (semi-transparent) */
        fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"4\" fill=\"#1d4ed8\" fill-opacity=\"0.35\"/>\n", rx, ry);

        /* Orange dot for received point (more opaque, drawn on top) */
        fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"3\" fill=\"#f97316\" fill-opacity=\"0.65\"/>\n", sx, sy);
    }

    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}


/*
 * write_trace_svg — Generate a time-domain waveform overlay plot as SVG
 *
 * What it does:
 *   Creates an SVG image showing two overlapping line charts:
 *   - Blue line: reference (clean) signal waveform
 *   - Orange line: received (noisy) signal waveform
 *
 *   Used for RF-domain visualization where the signal is a real-valued
 *   waveform (not complex I/Q), showing how noise and stage processing
 *   affect the signal shape over time.
 *
 * Layout:
 *   - 1100×600 pixel canvas with margins
 *   - X-axis: Sample index
 *   - Y-axis: Amplitude (auto-scaled with 5% padding)
 *   - Polyline rendering (connected line segments, not dots)
 *   - Limited to max_points to keep SVG file size manageable
 *
 * Parameters:
 *   path       — SVG file path to write
 *   ref        — Reference signal array
 *   sig        — Received signal array
 *   n          — Total number of samples
 *   max_points — Maximum samples to render (truncation limit)
 *   title      — Chart title
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
static int write_trace_svg(const char* path, const double* ref, const double* sig, size_t n, size_t max_points, const char* title, double fs_hz) {
    FILE* f;
    size_t i;
    size_t nout;

    double ymin = DBL_MAX;
    double ymax = -DBL_MAX;

    const int width = 1100;
    const int height = 600;
    const int ml = 80;
    const int mr = 30;
    const int mt = 50;
    const int mb = 80;
    const double plot_w = (double)(width - ml - mr);
    const double plot_h = (double)(height - mt - mb);
    double y_span;
    double x_scale;
    double y_scale;

    if (!path || !ref || !sig || n == 0u) {
        return -1;
    }

    nout = (n < max_points) ? n : max_points;

    /* Find Y-axis range */
    for (i = 0u; i < nout; ++i) {
        ymin = fmin(ymin, fmin(ref[i], sig[i]));
        ymax = fmax(ymax, fmax(ref[i], sig[i]));
    }

    y_span = ymax - ymin;
    if (y_span <= 0.0) {
        y_span = 1.0;
    }

    /* Add 5% vertical padding */
    {
        const double pad = 0.05 * y_span;
        ymin -= pad;
        ymax += pad;
    }

    y_span = ymax - ymin;
    x_scale = plot_w / (double)((nout > 1u) ? (nout - 1u) : 1u);
    y_scale = plot_h / y_span;

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    /* SVG header, background, title, plot area */
    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);
    fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");
    fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">%s</text>\n", 80, title ? title : "Trace");
    fprintf(f, "<rect x=\"%.2f\" y=\"%.2f\" width=\"%.2f\" height=\"%.2f\" fill=\"white\" stroke=\"#d1d5db\"/>\n", (double)ml, (double)mt, plot_w, plot_h);

    /* Vertical grid lines */
    for (i = 0u; i <= 6u; ++i) {
        const double xv = (double)(nout - 1u) * (double)i / 6.0;
        const double x = (double)ml + xv * x_scale;
        
        fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, (double)mt, x, (double)(height - mb));
        
        if (fs_hz > 0.0) {
            /* If we have a sampling rate, convert sample index to microseconds */
            const double t_us = (xv / fs_hz) * 1e6;
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.3f</text>\n", x, (double)(height - mb + 18), t_us);
        } else {
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", x, (double)(height - mb + 18), xv);
        }
    }

    /* Horizontal grid lines */
    for (i = 0u; i <= 6u; ++i) {
        const double yv = ymin + y_span * (double)i / 6.0;
        const double y = (double)(height - mb) - (yv - ymin) * y_scale;
        fprintf(f, "<line x1=\"%d\" y1=\"%.2f\" x2=\"%d\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", ml, y, width - mr, y);
        fprintf(f, "<text x=\"%d\" y=\"%.2f\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.3f</text>\n", ml - 8, y + 4.0, yv);
    }

    /* Axis labels */
    if (fs_hz > 0.0) {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Time (\xC2\xB5s)</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    } else {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Sample index</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    }
    fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Amplitude</text>\n", (double)(mt + plot_h * 0.5));

    /* Reference signal polyline (blue) */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#1d4ed8\" stroke-width=\"1.8\" points=\"");
    for (i = 0u; i < nout; ++i) {
        const double x = (double)ml + (double)i * x_scale;
        const double y = (double)(height - mb) - (ref[i] - ymin) * y_scale;
        fprintf(f, "%.2f,%.2f ", x, y);
    }
    fprintf(f, "\"/>\n");

    /* Received signal polyline (orange) */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#f97316\" stroke-width=\"1.8\" points=\"");
    for (i = 0u; i < nout; ++i) {
        const double x = (double)ml + (double)i * x_scale;
        const double y = (double)(height - mb) - (sig[i] - ymin) * y_scale;
        fprintf(f, "%.2f,%.2f ", x, y);
    }
    fprintf(f, "\"/>\n");

    /* Legend */
    fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"5\" fill=\"#1d4ed8\"/>\n", (double)ml + 18.0, (double)mt + 20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Reference</text>\n", (double)ml + 32.0, (double)mt + 24.0);
    fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"5\" fill=\"#f97316\"/>\n", (double)ml + 118.0, (double)mt + 20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Received</text>\n", (double)ml + 132.0, (double)mt + 24.0);

    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}


/*
 * write_metrics_svg — Generate a stage metrics summary table as SVG
 *
 * What it does:
 *   Creates an SVG image containing a formatted data table showing
 *   performance metrics at each receiver stage. The table has:
 *
 *   Columns: Stage | Domain | Signal Power | Noise Power | SNR (dB) | EVM (%)
 *
 *   Each row represents one stage of the receiver chain. Alternating rows
 *   have slightly different background colors for readability.
 *
 * Parameters:
 *   path    — SVG file path to write
 *   metrics — Array of StageMetric structs
 *   count   — Number of stages
 *   title   — Table title (displayed above the table)
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
int write_metrics_svg(const char* path, const StageMetric* metrics, size_t count, const char* title) {
    FILE* f;
    size_t i;

    /* Layout constants */
    const int width = 1120;
    const int row_h = 28;       /* Height of each data row in pixels */
    const int header_h = 34;    /* Height of the header row */
    const int top = 52;         /* Y position of the table start */
    const int left = 36;        /* X position of the table start */

    /* Column widths */
    const int col_stage = 220;
    const int col_domain = 180;
    const int col_sig = 160;
    const int col_noise = 160;
    const int col_snr = 120;
    const int col_evm = 120;
    const int total_w = col_stage + col_domain + col_sig + col_noise + col_snr + col_evm;

    /* Dynamic height based on number of rows */
    const int height = top + header_h + (int)count * row_h + 40;

    if (!path || !metrics || count == 0u) {
        return -1;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    /* SVG header and background */
    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);
    fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");
    fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">%s</text>\n", left, title ? title : "Stage Metrics");

    /* Table border */
    fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"white\" stroke=\"#d1d5db\"/>\n", left, top, total_w, header_h + (int)count * row_h);

    /* Header row (dark background with white text) */
    fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"#1f2937\"/>\n", left, top, total_w, header_h);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"white\">Stage</text>\n", left + 10, top + 22);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"white\">Domain</text>\n", left + col_stage + 10, top + 22);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"white\">Signal Pwr</text>\n", left + col_stage + col_domain + 10, top + 22);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"white\">Noise Pwr</text>\n", left + col_stage + col_domain + col_sig + 10, top + 22);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"white\">SNR dB</text>\n", left + col_stage + col_domain + col_sig + col_noise + 10, top + 22);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"white\">EVM %%</text>\n", left + col_stage + col_domain + col_sig + col_noise + col_snr + 10, top + 22);

    /* Data rows */
    for (i = 0u; i < count; ++i) {
        char stage_label[128];
        const int y = top + header_h + (int)i * row_h;

        /* Alternating row colors: white and light gray */
        const int alt = ((i & 1u) == 0u) ? 0 : 1;

        /* Convert the internal stage name to a human-readable label */
        humanize_stage_name(metrics[i].stage, stage_label, sizeof(stage_label));
        {
            char numbered_label[256];
            snprintf(numbered_label, sizeof(numbered_label), "%zu. %s", i + 1u, stage_label);
            strncpy(stage_label, numbered_label, sizeof(stage_label));
            stage_label[sizeof(stage_label) - 1u] = '\0';
        }

        fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"%s\"/>\n", left, y, total_w, row_h, alt ? "#f8fafc" : "#ffffff");
        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111827\">%s</text>\n", left + 10, y + 19, stage_label);
        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111827\">%s</text>\n", left + col_stage + 10, y + 19, metrics[i].domain);
        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111827\">%.6e</text>\n", left + col_stage + col_domain + 10, y + 19, metrics[i].signal_power);
        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111827\">%.6e</text>\n", left + col_stage + col_domain + col_sig + 10, y + 19, metrics[i].noise_power);
        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111827\">%.3f</text>\n", left + col_stage + col_domain + col_sig + col_noise + 10, y + 19, metrics[i].snr_db);
        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#111827\">%.3f</text>\n", left + col_stage + col_domain + col_sig + col_noise + col_snr + 10, y + 19, metrics[i].evm_pct);
    }

    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}


/*
 * write_budget_svg — Generate a link budget parameter card as SVG
 *
 * What it does:
 *   Creates a simple SVG "card" that lists the key simulation parameters:
 *   carrier frequency, symbol rate, roll-off, noise bandwidth, antenna
 *   temperature, input SNR, noise power, and signal power.
 *
 *   This provides a quick visual summary of the simulation configuration,
 *   useful when comparing multiple simulation runs.
 *
 * Parameters:
 *   path         — SVG file path to write
 *   cfg          — Simulation configuration struct
 *   noise_w      — Thermal noise power (watts)
 *   noise_dbm    — Noise power (dBm)
 *   signal_dbm   — Signal power (dBm)
 *   bandwidth_hz — Noise bandwidth (Hz)
 *
 * Returns:
 *   0 on success
 */
int write_budget_svg(const char* path, const SimConfig* cfg, double noise_w, double noise_dbm, double signal_dbm, double bandwidth_hz) {
    FILE* f;
    const int width = 900;
    const int height = 360;
    const int left = 40;
    const int top = 60;
    const int row_h = 30;

    if (!path || !cfg) {
        return -1;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);
    fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");
    fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">Input Budget</text>\n", left);
    fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" fill=\"white\" stroke=\"#d1d5db\"/>\n", left, top, 820, 240);

    /* Each row has a label on the left and a value on the right */
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Carrier frequency</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.3f GHz</text>\n", left + 12, top + row_h, left + 280, top + row_h, cfg->carrier_hz / 1e9);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Symbol rate</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.3f Msym/s</text>\n", left + 12, top + 2 * row_h, left + 280, top + 2 * row_h, cfg->symbol_rate_hz / 1e6);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Roll-off</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.3f</text>\n", left + 12, top + 3 * row_h, left + 280, top + 3 * row_h, cfg->rolloff);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Noise bandwidth</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.3f MHz</text>\n", left + 12, top + 4 * row_h, left + 280, top + 4 * row_h, bandwidth_hz / 1e6);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Antenna temp</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.1f K</text>\n", left + 12, top + 5 * row_h, left + 280, top + 5 * row_h, cfg->antenna_temp_k);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Input SNR</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.2f dB</text>\n", left + 12, top + 6 * row_h, left + 280, top + 6 * row_h, cfg->input_snr_db);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Noise power</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.6e W / %.2f dBm</text>\n", left + 12, top + 7 * row_h, left + 280, top + 7 * row_h, noise_w, noise_dbm);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Signal power</text><text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">%.2f dBm</text>\n", left + 12, top + 8 * row_h, left + 280, top + 8 * row_h, signal_dbm);

    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}


/* ============================================================================
 * COMPOSITE ARTIFACT WRITERS (Public API)
 * ============================================================================
 *
 * These functions combine multiple steps:
 *   1. Build human-readable names and file-safe slugs from the stage name
 *   2. Format metric tags for filename embedding
 *   3. Construct the full file paths for both CSV and SVG outputs
 *   4. Write both the CSV data file and the SVG visualization
 * ============================================================================ */

/*
 * StageRenderInfo — Temporary struct holding pre-computed name variants
 *
 * Members:
 *   stage_label    — Human-readable name (e.g., "RF BPF EQ")
 *   stage_slug     — Filename-safe slug (e.g., "rf_bpf_eq")
 *   metric_tag     — Filename metric tag (e.g., "_snr_20p15db_evm_3p15pct")
 *   metric_suffix  — Chart title suffix (e.g., " | SNR 20.15 dB | EVM 3.15%")
 */
typedef struct StageRenderInfo {
    char stage_label[128];
    char stage_slug[128];
    char metric_tag[128];
    char metric_suffix[128];
} StageRenderInfo;


/*
 * build_stage_render_info — Pre-compute all name variants for a stage
 *
 * What it does:
 *   Takes a raw stage name and a metric, and fills in a StageRenderInfo
 *   struct with all four variants (label, slug, metric_tag, metric_suffix).
 *   This avoids duplicating the name-formatting logic in every caller.
 */
static void build_stage_render_info(const char* raw_stage_name, const StageMetric* metric, StageRenderInfo* info) {
    if (!info) {
        return;
    }

    /* Initialize all fields to empty strings */
    info->stage_label[0] = '\0';
    info->stage_slug[0] = '\0';
    info->metric_tag[0] = '\0';
    info->metric_suffix[0] = '\0';

    humanize_stage_name(raw_stage_name, info->stage_label, sizeof(info->stage_label));
    slugify_text(info->stage_label, info->stage_slug, sizeof(info->stage_slug));
    format_metric_tag(metric, info->metric_tag, sizeof(info->metric_tag));
    format_metric_suffix(metric, info->metric_suffix, sizeof(info->metric_suffix));
}


/*
 * write_constellation_stage_artifacts — Write both CSV + SVG for a constellation snapshot
 *
 * What it does:
 *   This is the high-level function called from main.c (from both
 *   simulate_complex_baseband and simulate_bruteforce_rf) every time
 *   the simulator wants to save a constellation diagram.
 *
 *   It handles:
 *   1. Building human-readable/filename-safe names from the raw stage name
 *   2. Constructing proper file paths with stage numbering and metric tags
 *   3. Writing the CSV data file (for external analysis)
 *   4. Writing the SVG chart file (for visual inspection)
 *
 * File naming:
 *   Input stage:  {prefix}_input.csv and {prefix}_input{metric_tag}.svg
 *   Other stages: {prefix}_stage_{NN}_{slug}.csv and {prefix}_stage_{NN}_{slug}{metric_tag}.svg
 *
 * Parameters:
 *   csv_run_dir    — Directory for CSV output (e.g., "out/topology_sim_1/csv")
 *   svg_run_dir    — Directory for SVG output (e.g., "out/topology_sim_1/svg")
 *   file_prefix    — Common file prefix (e.g., "baseband" or "rf")
 *   stage_number   — 0-based stage index (used in filenames as 2-digit number)
 *   is_input       — 1 if this is the input measurement (before any stages)
 *   raw_stage_name — Internal stage name (e.g., "lna", "rf_bpf_eq", "input")
 *   metric         — Stage metric (SNR, EVM values for filename/title embedding)
 *   title_prefix   — Chart title prefix (e.g., "Baseband", "RF")
 *   ref            — Reference constellation points
 *   sig            — Received constellation points
 *   nsym           — Number of symbols
 */
static size_t build_realistic_constellation_subset(
    const Complex* ref,
    const Complex* sig,
    size_t nsym,
    size_t max_points,
    Complex** plot_ref_out,
    Complex** plot_sig_out);

void write_constellation_stage_artifacts(
    const char* csv_run_dir,
    const char* svg_run_dir,
    const char* file_prefix,
    size_t stage_number,
    int is_input,
    const char* raw_stage_name,
    const StageMetric* metric,
    const char* title_prefix,
    const Complex* constellation_template,
    size_t constellation_count,
    const Complex* ref,
    const Complex* sig,
    size_t nsym) {
    char csv_path[1024];
    char svg_path[1024];
    char title[384];
    StageRenderInfo info;

    if (!csv_run_dir || !svg_run_dir || !file_prefix || !title_prefix || !ref || !sig || nsym == 0u) {
        return;
    }

    /* Pre-compute name variants */
    build_stage_render_info(raw_stage_name, metric, &info);

    /* Construct file paths and chart title based on whether this is the input or a stage */
    if (is_input) {
        snprintf(csv_path, sizeof(csv_path), "%s/%s_input.csv", csv_run_dir, file_prefix);
        
        snprintf(svg_path, sizeof(svg_path), "%s/%s_input_realistic%s.svg", svg_run_dir, file_prefix, info.metric_tag);
        snprintf(title, sizeof(title), "%.32s input (Realistic)%.160s", title_prefix, info.metric_suffix);
        
        {
            Complex* plot_ref = NULL;
            Complex* plot_sig = NULL;
            const Complex* plot_ref_view = ref;
            const Complex* plot_sig_view = sig;
            size_t plot_count = build_realistic_constellation_subset(ref, sig, nsym, 512u, &plot_ref, &plot_sig);

            if (plot_ref && plot_sig) {
                plot_ref_view = plot_ref;
                plot_sig_view = plot_sig;
            }

        write_constellation_csv(csv_path, ref, sig, nsym);
            write_constellation_svg(svg_path, constellation_template, constellation_count, plot_ref_view, plot_sig_view, plot_count, title);

            free(plot_ref);
            free(plot_sig);
        }
        
    } else {
        snprintf(csv_path, sizeof(csv_path), "%s/%s_stage_%02zu_%s.csv", csv_run_dir, file_prefix, stage_number, info.stage_slug);
        
        /* 1) Generate the REALISTIC chart */
        snprintf(svg_path, sizeof(svg_path), "%s/%s_stage_%02zu_%s_realistic%s.svg", svg_run_dir, file_prefix, stage_number, info.stage_slug, info.metric_tag);
        snprintf(title, sizeof(title), "%.32s stage %02zu (Realistic) - %.120s%.80s", title_prefix, stage_number, info.stage_label, info.metric_suffix);
        
        {
            Complex* plot_ref = NULL;
            Complex* plot_sig = NULL;
            const Complex* plot_ref_view = ref;
            const Complex* plot_sig_view = sig;
            size_t plot_count = build_realistic_constellation_subset(ref, sig, nsym, 512u, &plot_ref, &plot_sig);

            if (plot_ref && plot_sig) {
                plot_ref_view = plot_ref;
                plot_sig_view = plot_sig;
            }

        write_constellation_csv(csv_path, ref, sig, nsym);
            write_constellation_svg(svg_path, constellation_template, constellation_count, plot_ref_view, plot_sig_view, plot_count, title);

            free(plot_ref);
            free(plot_sig);
        }
        
        /* 2) Synthesize and Output the PERFECT (Mathematically Evaluated AWGN) chart */
        {
            Complex* perf_ref = (Complex*)calloc(nsym, sizeof(Complex));
            Complex* perf_sig = (Complex*)calloc(nsym, sizeof(Complex));
            if (perf_ref && perf_sig) {
                size_t k, c;
                
                /* Match scale of `ref` to `constellation_template` before finding nearest neighbors */
                {
                    double p_ref_avg = 0.0;
                    double p_tmpl_avg = 0.0;
                    double pre_scale = 1.0;
                    
                    for(k = 0; k < nsym; ++k) {
                        p_ref_avg += ref[k].re * ref[k].re + ref[k].im * ref[k].im;
                    }
                    for(c = 0; c < constellation_count; ++c) {
                        p_tmpl_avg += constellation_template[c].re * constellation_template[c].re + constellation_template[c].im * constellation_template[c].im;
                    }
                    
                    if (p_ref_avg > 0.0) {
                        p_ref_avg /= (double)nsym;
                        p_tmpl_avg /= (double)constellation_count;
                        pre_scale = sqrt(p_tmpl_avg / p_ref_avg);
                    }
                    
                    /* Project `ref` (scaled to template power) to the closest ideal `constellation_template` symbol.
                       This effectively removes all inter-symbol-interference and sampling offsets. */
                    for(k = 0; k < nsym; ++k) {
                        double scaled_re = ref[k].re * pre_scale;
                        double scaled_im = ref[k].im * pre_scale;
                        double best_dist = 1e20;
                        size_t best_idx = 0;
                        for(c = 0; c < constellation_count; ++c) {
                            double dx = scaled_re - constellation_template[c].re;
                            double dy = scaled_im - constellation_template[c].im;
                            double d = dx*dx + dy*dy;
                            if(d < best_dist) { best_dist = d; best_idx = c; }
                        }
                        perf_ref[k] = constellation_template[best_idx];
                    }
                }
                
                /* Ensure power of the projected ideal symbol matches the actual reference power exactly */
                {
                    double p_perf = 0.0;
                    double p_ref = 0.0;
                    double scale = 1.0;
                    for(k = 0; k < nsym; ++k) {
                        p_perf += perf_ref[k].re * perf_ref[k].re + perf_ref[k].im * perf_ref[k].im;
                        p_ref += ref[k].re * ref[k].re + ref[k].im * ref[k].im;
                    }
                    if(p_perf > 0.0) {
                        scale = sqrt(p_ref / p_perf);
                    }
                    for(k = 0; k < nsym; ++k) {
                        perf_ref[k].re *= scale;
                        perf_ref[k].im *= scale;
                        perf_sig[k] = perf_ref[k]; /* Duplicate into signal buffer for noise injection */
                    }
                }
                
                /* Add ideal AWGN using the exact theoretical SNR determined for this stage */
                {
                    double req_ps = 0.0;
                    for(k = 0; k < nsym; ++k) {
                        req_ps += perf_ref[k].re * perf_ref[k].re + perf_ref[k].im * perf_ref[k].im;
                    }
                    req_ps /= (double)nsym;
                    
                    /* Convert dB noise out to linear scale using metric->snr_db */
                    if(metric->snr_db > -900.0) {
                        /* 
                         * Convert EVM into physical variance.
                         * The value metric->snr_db is the wideband SNR for RF stages, causing collapsed references. 
                         * Using EVM strictly maps to the True Baseband symbol-rate Es/N0 correctly for ALL stages.
                         */
                        double es_n0_db;
                        if (!isnan(metric->evm_pct) && metric->evm_pct > 0.0) {
                            es_n0_db = -20.0 * log10(metric->evm_pct / 100.0);
                        } else {
                            es_n0_db = metric->snr_db;
                        }
                        
                        double req_pn = req_ps / pow(10.0, es_n0_db / 10.0);
                        double sigma = sqrt(req_pn / 2.0);
                        for(k = 0; k < nsym; ++k) {
                            perf_sig[k].re += prng_gauss() * sigma;
                            perf_sig[k].im += prng_gauss() * sigma;
                        }
                    }
                }
                
                /* Output synthetic perfect SVGs */
                snprintf(svg_path, sizeof(svg_path), "%s/%s_stage_%02zu_%s_perfect%s.svg", svg_run_dir, file_prefix, stage_number, info.stage_slug, info.metric_tag);
                snprintf(title, sizeof(title), "%.32s stage %02zu (Perfect) - %.120s%.80s", title_prefix, stage_number, info.stage_label, info.metric_suffix);
                
                write_constellation_svg(svg_path, constellation_template, constellation_count, perf_ref, perf_sig, nsym, title);
                
                free(perf_ref);
                free(perf_sig);
            }
        }
    }
}


static size_t build_realistic_constellation_subset(
    const Complex* ref,
    const Complex* sig,
    size_t nsym,
    size_t max_points,
    Complex** plot_ref_out,
    Complex** plot_sig_out) {
    size_t plot_n;
    size_t step;
    size_t k;

    if (!ref || !sig || !plot_ref_out || !plot_sig_out || nsym == 0u) {
        return 0u;
    }

    *plot_ref_out = NULL;
    *plot_sig_out = NULL;

    plot_n = (nsym > max_points) ? max_points : nsym;
    if (plot_n == nsym) {
        return nsym;
    }

    *plot_ref_out = (Complex*)malloc(plot_n * sizeof(Complex));
    *plot_sig_out = (Complex*)malloc(plot_n * sizeof(Complex));
    if (!*plot_ref_out || !*plot_sig_out) {
        free(*plot_ref_out);
        free(*plot_sig_out);
        *plot_ref_out = NULL;
        *plot_sig_out = NULL;
        return nsym;
    }

    step = (nsym + plot_n - 1u) / plot_n;
    for (k = 0u; k < plot_n; ++k) {
        size_t idx = k * step;
        if (idx >= nsym) {
            idx = nsym - 1u;
        }
        (*plot_ref_out)[k] = ref[idx];
        (*plot_sig_out)[k] = sig[idx];
    }

    return plot_n;
}


/*
 * write_trace_stage_artifacts — Write both CSV + SVG for a real signal trace snapshot
 *
 * What it does:
 *   Same concept as write_constellation_stage_artifacts, but for real-valued
 *   (non-complex) RF waveforms. Writes a time-domain trace CSV and SVG.
 *
 * Used by:
 *   simulate_bruteforce_rf() — for the RF frontend path where signals are
 *   real-valued waveforms rather than complex I/Q pairs.
 *
 * Parameters:
 *   Same as write_constellation_stage_artifacts, but:
 *   ref, sig — are double arrays (real signals, not Complex)
 *   n        — number of samples (not symbols)
 *   max_points — maximum samples to render/write (truncation limit)
 */
void write_trace_stage_artifacts(
    const char* csv_run_dir,
    const char* svg_run_dir,
    const char* file_prefix,
    size_t stage_number,
    int is_input,
    const char* raw_stage_name,
    const StageMetric* metric,
    const char* title_prefix,
    const double* ref,
    const double* sig,
    size_t n,
    size_t max_points,
    double fs_hz) {
    char csv_path[1024];
    char svg_path[1024];
    char title[384];
    StageRenderInfo info;

    if (!csv_run_dir || !svg_run_dir || !file_prefix || !title_prefix || !ref || !sig || n == 0u) {
        return;
    }

    build_stage_render_info(raw_stage_name, metric, &info);

    if (is_input) {
        snprintf(csv_path, sizeof(csv_path), "%s/%s_input.csv", csv_run_dir, file_prefix);
        snprintf(svg_path, sizeof(svg_path), "%s/%s_input%s.svg", svg_run_dir, file_prefix, info.metric_tag);
        snprintf(title, sizeof(title), "%.32s input%.160s", title_prefix, info.metric_suffix);
    } else {
        snprintf(csv_path, sizeof(csv_path), "%s/%s_stage_%02zu_%s.csv", csv_run_dir, file_prefix, stage_number, info.stage_slug);
        snprintf(svg_path, sizeof(svg_path), "%s/%s_stage_%02zu_%s%s.svg", svg_run_dir, file_prefix, stage_number, info.stage_slug, info.metric_tag);
        snprintf(title, sizeof(title), "%.32s stage %02zu - %.120s%.80s", title_prefix, stage_number, info.stage_label, info.metric_suffix);
    }

    write_real_trace_csv(csv_path, ref, sig, n, max_points);
    write_trace_svg(svg_path, ref, sig, n, max_points, title, fs_hz);
}


/*
 * write_chain_architecture_mermaid — Write a markdown file containing a Mermaid diagram
 */
#include "stage_models.h"

int write_chain_architecture_mermaid(
    const char* path,
    const void* cfg_ptr, /* typed as void* in header to avoid circular dependency if not careful, but we can cast to const StageModelsConfig* */
    double final_snr_db,
    double target_vpp,
    double input_snr_db) {
    const StageModelsConfig* cfg = (const StageModelsConfig*)cfg_ptr;
    FILE* f;
    const StageModel* rf_stages;
    const StageModel* mix_stages;
    size_t rf_count, mix_count;
    char prev_id[32] = "ANT";
    char bb_prev[32] = "MIX";
    size_t i;

    if (!path || !cfg) {
        return -1;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    fprintf(f, "```mermaid\n");
    fprintf(f, "flowchart TD\n");
    fprintf(f, "    classDef generic fill:#2b2b2b,stroke:#555,stroke-width:2px,color:#fff,rx:5px\n");
    fprintf(f, "    classDef antenna fill:#1e4c27,stroke:#2e753d,stroke-width:2px,color:#fff,opacity:0.9,rx:20px\n");
    fprintf(f, "    classDef filter fill:#144163,stroke:#2a6d9e,stroke-width:2px,color:#fff,rx:5px\n");
    fprintf(f, "    classDef amp fill:#5a2323,stroke:#913838,stroke-width:2px,color:#fff,rx:5px\n");
    fprintf(f, "    classDef mixer fill:#4c3561,stroke:#774f9e,stroke-width:2px,color:#fff,rx:50px\n");
    fprintf(f, "    classDef outcome fill:#4a4a4a,stroke:#888,stroke-width:3px,color:#fff,rx:10px\n\n");

    fprintf(f, "    subgraph RF_Domain[\"Radio Frequency (RF) Domain\"]\n");
    fprintf(f, "        direction LR\n");
    fprintf(f, "        ANT([\"📡 Antenna<br/>Input SNR: %.2f dB\"]):::antenna\n", input_snr_db);

    rf_count = stage_models_get(cfg, STAGE_CHAIN_RF_FRONTEND, &rf_stages);
    for (i = 0; i < rf_count; i++) {
        char human[64];
        char class_name[32] = "generic";
        
        humanize_stage_name(rf_stages[i].name, human, sizeof(human));
        if (strstr(human, "BPF") || strstr(human, "LPF") || strstr(human, "Filter") || strstr(human, "EQ")) strcpy(class_name, "filter");
        if (strstr(human, "LNA") || strstr(human, "Amp") || strstr(human, "Gain")) strcpy(class_name, "amp");
        if (strstr(human, "Mixer")) strcpy(class_name, "mixer");
        
        fprintf(f, "        RF%zu[\"%s<br/>Gain: %.1f dB<br/>NF: %.1f dB\"]:::%s\n", i, human, rf_stages[i].gain_db, rf_stages[i].nf_db, class_name);
        if (i == 0) {
            fprintf(f, "        %s ==>|Wideband SNR| RF%zu\n", prev_id, i);
        } else {
            fprintf(f, "        %s ==> RF%zu\n", prev_id, i);
        }
        snprintf(prev_id, sizeof(prev_id), "RF%zu", i);
    }
    fprintf(f, "    end\n\n");

    mix_count = stage_models_get(cfg, STAGE_CHAIN_RF_POSTMIX_BB, &mix_stages);

    fprintf(f, "    MIX{\"✖<br/>Downconversion<br/>Mixer\"}:::mixer\n\n");

    fprintf(f, "    subgraph BB_Domain[\"Intermediate Baseband (BB) Domain\"]\n");
    fprintf(f, "        direction LR\n");
    
    for (i = 0; i < mix_count; i++) {
        char human[64];
        char class_name[32] = "generic";
        
        humanize_stage_name(mix_stages[i].name, human, sizeof(human));
        if (strstr(human, "BPF") || strstr(human, "LPF") || strstr(human, "Filter") || strstr(human, "EQ")) strcpy(class_name, "filter");
        if (strstr(human, "LNA") || strstr(human, "Amp") || strstr(human, "Vpp")) strcpy(class_name, "amp");
        
        fprintf(f, "        BB%zu[\"%s<br/>Gain: %.1f dB<br/>NF: %.1f dB\"]:::%s\n", i, human, mix_stages[i].gain_db, mix_stages[i].nf_db, class_name);
        
        if (i > 0) {
            fprintf(f, "        BB%zu ==> BB%zu\n", i-1, i);
        }
        snprintf(bb_prev, sizeof(bb_prev), "BB%zu", i);
    }
    fprintf(f, "    end\n\n");

    fprintf(f, "    OUT([\"💻 Digital Demodulator<br/>Final Baseband SNR: %.2f dB<br/>Output Target: %.1f Vpp\"]):::outcome\n\n", final_snr_db, target_vpp);

    fprintf(f, "    %s ==>|RF Frequency| MIX\n", prev_id);
    if (mix_count > 0) {
        fprintf(f, "    MIX ==>|BB Frequency| BB0\n");
        fprintf(f, "    %s ==> OUT\n", bb_prev);
    } else {
        fprintf(f, "    MIX ==> OUT\n");
    }

    fprintf(f, "\n    style RF_Domain fill:#111,stroke:#333,stroke-width:2px,color:#ddd\n");
    fprintf(f, "    style BB_Domain fill:#0a111a,stroke:#203a5c,stroke-width:2px,color:#ddd\n");
    fprintf(f, "```\n");

    fclose(f);
    return 0;
}
static int write_complex_trace_svg(const char* path, const Complex* sig, size_t n, size_t max_points, const char* title, double fs_hz) {
    FILE* f;
    size_t i;
    size_t nout = (n < max_points) ? n : max_points;
    const int width = 1100, height = 600;
    const int ml = 80, mr = 30, mt = 50, mb = 80;
    const double plot_w = width - ml - mr;
    const double plot_h = height - mt - mb;
    
    double ymin = DBL_MAX;
    double ymax = -DBL_MAX;

    for (i = 0; i < nout; ++i) {
        ymin = fmin(ymin, fmin(sig[i].re, sig[i].im));
        ymax = fmax(ymax, fmax(sig[i].re, sig[i].im));
    }
    
    if (ymax - ymin <= 0.0) {
        ymin -= 1.0;
        ymax += 1.0;
    }
    
    /* Add padding */
    {
        double pad = (ymax - ymin) * 0.1;
        ymin -= pad;
        ymax += pad;
    }

    double y_span = ymax - ymin;
    double x_scale = plot_w / ((nout > 1) ? (nout - 1) : 1);
    double y_scale = plot_h / y_span;
    
    f = fopen(path, "w");
    if(!f) return -1;
    
    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);
    fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");
    fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">%s</text>\n", ml, title ? title : "Signal Trace");
    fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%.2f\" height=\"%.2f\" fill=\"white\" stroke=\"#d1d5db\"/>\n", ml, mt, plot_w, plot_h);
    
    /* Horizontal grid lines with labels */
    for (i = 0u; i <= 6u; ++i) {
        const double yv = ymin + y_span * (double)i / 6.0;
        const double y = (double)(height - mb) - (yv - ymin) * y_scale;
        fprintf(f, "<line x1=\"%d\" y1=\"%.2f\" x2=\"%d\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", ml, y, width - mr, y);
        fprintf(f, "<text x=\"%d\" y=\"%.2f\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.3f</text>\n", ml - 8, y + 4.0, yv);
    }
    
    /* Vertical grid lines with labels */
    for (i = 0u; i <= 6u; ++i) {
        const double xv = (double)(nout - 1u) * (double)i / 6.0;
        const double x = (double)ml + xv * x_scale;
        fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, (double)mt, x, (double)(height - mb));
        
        if (fs_hz > 0.0) {
            const double t_us = (xv / fs_hz) * 1e6;
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.3f</text>\n", x, (double)(height - mb + 18), t_us);
        } else {
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", x, (double)(height - mb + 18), xv);
        }
    }
    
    if (fs_hz > 0.0) {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Time (\xC2\xB5s)</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    } else {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Sample index</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    }
    fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Amplitude</text>\n", (double)(mt + plot_h * 0.5));
    
    /* Vertical grid lines with labels */
    for (i = 0u; i <= 6u; ++i) {
        const double xv = (double)(nout - 1u) * (double)i / 6.0;
        const double x = (double)ml + xv * x_scale;
        fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, (double)mt, x, (double)(height - mb));
        
        if (fs_hz > 0.0) {
            const double t_us = (xv / fs_hz) * 1e6;
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.3f</text>\n", x, (double)(height - mb + 18), t_us);
        } else {
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", x, (double)(height - mb + 18), xv);
        }
    }
    
    if (fs_hz > 0.0) {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Time (\xC2\xB5s)</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    } else {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Sample index</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    }
    fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Amplitude</text>\n", (double)(mt + plot_h * 0.5));
    
    /* I curve */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#3b82f6\" stroke-width=\"1.5\" stroke-opacity=\"0.8\" points=\"");
    for(i=0; i<nout; ++i) fprintf(f, "%.2f,%.2f ", ml + i*x_scale, mt + (ymax - sig[i].re)*y_scale);
    fprintf(f, "\"/>\n");
    
    /* Q curve */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#ef4444\" stroke-width=\"1.5\" stroke-opacity=\"0.8\" points=\"");
    for(i=0; i<nout; ++i) fprintf(f, "%.2f,%.2f ", ml + i*x_scale, mt + (ymax - sig[i].im)*y_scale);
    fprintf(f, "\"/>\n");
    
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"14\" fill=\"#3b82f6\">In-Phase (I)</text>\n", ml+10, mt+20);
    fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"14\" fill=\"#ef4444\">Quadrature (Q)</text>\n", ml+10, mt+40);
    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}

void write_complex_trace_stage_artifacts(const char* svg_run_dir, const char* file_prefix, size_t stage_number, int is_input, const char* raw_stage_name, const StageMetric* metric, const Complex* sig, size_t nsym, double fs_hz) {
    char svg_path[1024];
    char title[384];
    StageRenderInfo info;
    if (!svg_run_dir || !file_prefix || !sig || nsym == 0) return;
    build_stage_render_info(raw_stage_name, metric, &info);
    if(is_input) {
        snprintf(svg_path, sizeof(svg_path), "%s/%s_input_trace%s.svg", svg_run_dir, file_prefix, info.metric_tag);
        snprintf(title, sizeof(title), "Input Signal Trace%.160s", info.metric_suffix);
    } else {
        snprintf(svg_path, sizeof(svg_path), "%s/%s_stage_%02zu_%s_trace%s.svg", svg_run_dir, file_prefix, stage_number, info.stage_slug, info.metric_tag);
        snprintf(title, sizeof(title), "Stage %02zu - %.120s Trace%.80s", stage_number, info.stage_label, info.metric_suffix);
    }
    write_complex_trace_svg(svg_path, sig, nsym, 200, title, fs_hz);
}
