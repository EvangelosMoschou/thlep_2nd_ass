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
 *   path — File path to write (e.g., "out/baseband/baseband_input.csv")
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

    double display_scale = 1.0;
    double ref_power = 0.0;
    for (i = 0u; i < n; ++i) {
        ref_power += ref[i].re * ref[i].re + ref[i].im * ref[i].im;
    }
    if (n > 0 && ref_power > 0.0) {
        display_scale = 1.0 / sqrt(ref_power / (double)n);
    }

    /* Write CSV header */
    fprintf(f, "idx,ref_i,ref_q,sig_i,sig_q\n");

    /* Write one row per symbol with 12 decimal digits of precision */
    for (i = 0; i < n; ++i) {
        fprintf(f, "%zu,%.12f,%.12f,%.12f,%.12f\n", i, ref[i].re * display_scale, ref[i].im * display_scale, sig[i].re * display_scale, sig[i].im * display_scale);
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
 * Output format:
 *   idx,ref,sig
 *   0,-0.407051012007,-0.298324517176
 *   1,0.137357702640,0.023676809993
 *   ...
 *
 * Parameters:
 *   path       — File path to write
 *   ref        — Reference waveform
 *   sig        — Received waveform
 *   n          — Number of samples
 *   max_points — Maximum rows to write (0 means write all)
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
static int write_real_trace_csv(const char* path, const double* ref, const double* sig, size_t n, size_t max_points) {
    FILE* f;
    size_t i;
    size_t count = n;

    if (!path || !ref || !sig || n == 0u) {
        return -1;
    }

    if (max_points > 0u && count > max_points) {
        count = max_points;
    }

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    fprintf(f, "idx,ref,sig\n");
    for (i = 0u; i < count; ++i) {
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
 *   stage_index,stage,domain,gain_db,nf_db,filter_len,is_limiter,signal_power,noise_power,snr_db,evm_pct
 *   1,input_rf,rf_real,nan,nan,0,0,1.000000e+00,1.234567e-02,19.085,11.11
 *   2,rf_00_switch,rf_real,-0.300000,0.300000,1,0,3.162278e+02,4.567890e+00,18.402,12.01
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
    fprintf(f, "stage_index,stage,domain,gain_db,nf_db,filter_len,is_limiter,signal_power,noise_power,snr_db,evm_pct\n");

    /* One row per stage with scientific notation for power values */
    for (i = 0; i < count; ++i) {
        fprintf(
            f,
            "%zu,%s,%s,%.6f,%.6f,%d,%d,%.12e,%.12e,%.6f,%.6f\n",
            i + 1u,
            metrics[i].stage,
            metrics[i].domain,
            metrics[i].gain_db,
            metrics[i].nf_db,
            metrics[i].filter_len,
            metrics[i].is_limiter,
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


/* ==========================================================================
 * SVG VISUALIZATION FUNCTIONS
 * ==========================================================================
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
 * ========================================================================== */

/*
 * write_constellation_svg — Generate a constellation scatter plot as SVG
 *
 * What it does:
 *   Creates an SVG image showing a 2D scatter plot of I/Q constellation points.
 *   The chart uses a MATLAB-like overlay mode:
 *   - Dots (blue): ideal constellation points
 *   - Clouds (orange): received symbol cloud
 *
 *   The cloud spread around the ideal dots visualizes stage-induced degradation.
 *   More spread = worse signal quality.
 *
 * Parameters:
 *   path    — Output SVG file path
 *   constellation_template — Ideal constellation points for reference markers
 *   constellation_count    — Number of reference constellation points
 *   ref     — Clean reference symbols
 *   sig     — Received/degraded symbols
 *   n       — Number of symbols
 *   title   — Chart title string (displayed at top of the image)
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
const double cloud_opacity = 0.50;
const double cloud_radius = 1.70;
    const double ref_sample_opacity = 0.22;
    const double ref_sample_radius = 0.92;
    const double ideal_dot_radius = 2.28;
const size_t density_replicas = 6u;
const double replica_step = 0.0040;
const double replica_dx[6] = {0.0, 1.0, -1.0, 0.0, 1.5, -1.5};
    const double replica_dy[6] = {0.0, 0.0, 0.0, 1.0, 0.5, -0.5};

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

    /* Match MATLAB visual range for direct 1:1 constellation comparison. */
    xmin = -2.0;
    xmax = 2.0;
    ymin = -2.0;
    ymax = 2.0;
    span_x = xmax - xmin;
    span_y = ymax - ymin;
    scale = fmin(plot_w / span_x, plot_h / span_y);
    scaled_w = span_x * scale;
    scaled_h = span_y * scale;
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
    fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"#1d4ed8\" fill-opacity=\"0.95\"/>\n", (double)ml + 18.0, (double)mt + 20.0, ideal_dot_radius);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Dots</text>\n", (double)ml + 32.0, (double)mt + 24.0);

    fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"#f97316\" fill-opacity=\"%.2f\"/>\n", (double)ml + 108.0, (double)mt + 20.0, cloud_radius, cloud_opacity);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Clouds</text>\n", (double)ml + 122.0, (double)mt + 24.0);

    /* --- Step 5: Calculate display scale to normalize to unit power --- */
    double display_scale = 1.0;
    double ref_power = 0.0;
    for (i = 0u; i < n; ++i) {
        ref_power += ref[i].re * ref[i].re + ref[i].im * ref[i].im;
    }
    if (n > 0 && ref_power > 0.0) {
        display_scale = 1.0 / sqrt(ref_power / (double)n);
    }

    /* --- Step 6: Draw data points --- */
    /* 1) Clouds first (received points) — 4x dense rendering for readability */
    for (i = 0u; i < n; ++i) {
        size_t r;
        for (r = 0u; r < density_replicas; ++r) {
            const double sx = x_origin + ((sig[i].re * display_scale) + replica_dx[r] * replica_step - xmin) * scale;
            const double sy = y_origin - ((sig[i].im * display_scale) + replica_dy[r] * replica_step - ymin) * scale;
            fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"#f97316\" fill-opacity=\"%.2f\"/>\n", sx, sy, cloud_radius, cloud_opacity);
        }
    }

    /* 2) Reference dots layer — 4x dense rendering */
    for (i = 0u; i < n; ++i) {
        size_t r;
        for (r = 0u; r < density_replicas; ++r) {
            const double rx = x_origin + ((ref[i].re * display_scale) + replica_dx[r] * replica_step - xmin) * scale;
            const double ry = y_origin - ((ref[i].im * display_scale) + replica_dy[r] * replica_step - ymin) * scale;
            fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"#1d4ed8\" fill-opacity=\"%.2f\"/>\n", rx, ry, ref_sample_radius, ref_sample_opacity);
        }
    }

    /* 3) Ideal constellation dots on top */
    if (constellation_template && constellation_count > 0u) {
        for (i = 0u; i < constellation_count; ++i) {
            const double gx = x_origin + (constellation_template[i].re - xmin) * scale;
            const double gy = y_origin - (constellation_template[i].im - ymin) * scale;
            fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%.2f\" fill=\"#1d4ed8\" fill-opacity=\"0.94\"/>\n", gx, gy, ideal_dot_radius);
        }
    }

    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}


/* Configuration for time-domain trace zooming (number of signal periods to show) */
#define TRACE_ZOOM_PERIODS_DEFAULT 50.0
#define TRACE_ZOOM_PERIODS_EARLY_RF 50.0

/*
 * trace_zoom_window_samples — Time-window policy for stage trace zoom
 *
 * All stage diagrams: zoom-in with period-based window
 *   - Show ~TRACE_ZOOM_PERIODS_DEFAULT periods of the signal when signal_hz is provided
 *   - Fallback to fixed microsecond window only when signal_hz is unavailable
 */
static size_t trace_zoom_window_samples(size_t total_samples, double fs_hz, size_t stage_number, int is_input, double signal_hz, double display_window_us) {
    const int is_high_rate_rf = (fs_hz >= 1.0e9);
    const double fallback_zoom_us = is_high_rate_rf ? 0.02 : 0.75;
    double periods_to_show = TRACE_ZOOM_PERIODS_DEFAULT;
    size_t window_samples;

    if (total_samples == 0u) {
        return 0u;
    }
    if (fs_hz <= 0.0) {
        return total_samples;
    }

    if (display_window_us > 0.0) {
        window_samples = (size_t)llround(display_window_us * 1e-6 * fs_hz);
    } else {
        if (is_high_rate_rf && !is_input && stage_number >= 1u && stage_number <= 5u) {
            periods_to_show = TRACE_ZOOM_PERIODS_EARLY_RF;
        }
        if (!is_input && stage_number >= 6u) {
            periods_to_show = 30.0;
        }

        if (signal_hz > 0.0) {
            window_samples = (size_t)llround((periods_to_show * fs_hz) / signal_hz);
        } else {
            const double window_us = fallback_zoom_us;
            window_samples = (size_t)llround(window_us * 1e-6 * fs_hz);
        }
    }

    if (window_samples < 2u) {
        window_samples = (total_samples >= 2u) ? 2u : total_samples;
    }
    if (window_samples > total_samples) {
        window_samples = total_samples;
    }
    return window_samples;
}

/* Fractional source position for interpolation-based rendering. */
static double trace_source_pos(size_t plot_idx, size_t plotted_points, size_t window_samples) {
    if (plotted_points <= 1u || window_samples <= 1u) {
        return 0.0;
    }
    return ((double)plot_idx * (double)(window_samples - 1u)) / (double)(plotted_points - 1u);
}

/* Format axis tick values so tiny/huge voltages remain readable. */
static void format_axis_tick_value(double value, char* out, size_t out_size) {
    const double abs_v = fabs(value);

    if (!out || out_size == 0u) {
        return;
    }

    if (abs_v > 0.0 && (abs_v < 1.0e-3 || abs_v >= 1.0e4)) {
        snprintf(out, out_size, "%.3e", value);
    } else if (abs_v < 0.1) {
        snprintf(out, out_size, "%.5f", value);
    } else if (abs_v < 10.0) {
        snprintf(out, out_size, "%.3f", value);
    } else {
        snprintf(out, out_size, "%.2f", value);
    }
}

/* Linear interpolation sampler for real traces. */
static double trace_sample_real_linear(const double* sig, size_t window_samples, size_t plot_idx, size_t plotted_points) {
    size_t i0;
    size_t i1;
    double pos;
    double frac;

    if (!sig || window_samples == 0u) {
        return 0.0;
    }
    if (window_samples == 1u || plotted_points <= 1u) {
        return sig[0];
    }

    pos = trace_source_pos(plot_idx, plotted_points, window_samples);
    i0 = (size_t)floor(pos);
    if (i0 >= window_samples - 1u) {
        return sig[window_samples - 1u];
    }
    i1 = i0 + 1u;
    frac = pos - (double)i0;
    return sig[i0] + (sig[i1] - sig[i0]) * frac;
}

/* Linear interpolation sampler for complex traces. */
static Complex trace_sample_complex_linear(const Complex* sig, size_t window_samples, size_t plot_idx, size_t plotted_points) {
    Complex out;
    size_t i0;
    size_t i1;
    double pos;
    double frac;

    out.re = 0.0;
    out.im = 0.0;

    if (!sig || window_samples == 0u) {
        return out;
    }
    if (window_samples == 1u || plotted_points <= 1u) {
        return sig[0];
    }

    pos = trace_source_pos(plot_idx, plotted_points, window_samples);
    i0 = (size_t)floor(pos);
    if (i0 >= window_samples - 1u) {
        return sig[window_samples - 1u];
    }
    i1 = i0 + 1u;
    frac = pos - (double)i0;

    out.re = sig[i0].re + (sig[i1].re - sig[i0].re) * frac;
    out.im = sig[i0].im + (sig[i1].im - sig[i0].im) * frac;
    return out;
}


/*
 * write_trace_svg — Generate a time-domain waveform plot as SVG
 *
 * What it does:
 *   Creates an SVG image showing a received-signal line chart.
 *
 *   Used for RF-domain visualization where the signal is a real-valued
 *   waveform (not complex I/Q), showing how noise and stage processing
 *   affect the signal shape over time.
 *
 * Layout:
 *   - 1100×600 pixel canvas with margins
 *   - X-axis: Time in us when fs_hz is known, sample index otherwise
 *   - Y-axis: Amplitude (auto-scaled with 5% padding)
 *   - Polyline rendering with source-index decimation for large windows
 *   - Limited to max_points after windowing to keep SVG size manageable
 *
 * Parameters:
 *   path       — SVG file path to write
 *   ref        — Reference signal array (kept for API compatibility)
 *   sig        — Received signal array
 *   n          — Total number of samples
 *   max_points — Maximum plotted points after windowing/decimation
 *   title      — Chart title
 *   fs_hz      — Sampling rate in Hz (if >0, X labels in us)
 *   window_samples — Time-window samples to show from the beginning of signal
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
static int write_trace_svg(const char* path, const double* ref, const double* sig, size_t n, size_t max_points, const char* title, double fs_hz, size_t window_samples) {
    FILE* f;
    size_t i;
    size_t nout;
    size_t window_n;

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

    window_n = n;
    if (window_samples > 1u && window_samples < window_n) {
        window_n = window_samples;
    }
    if (window_n <= 1u) {
        nout = window_n;
    } else {
        nout = (max_points >= 2u) ? max_points : 2u;
    }
    if (nout == 0u) {
        return -1;
    }

    /* Find Y-axis range */
    for (i = 0u; i < nout; ++i) {
        const double v = trace_sample_real_linear(sig, window_n, i, nout);
        ymin = fmin(ymin, v);
        ymax = fmax(ymax, v);
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
        const double xv = (double)((window_n > 1u) ? (window_n - 1u) : 0u) * (double)i / 6.0;
        const double x = (double)ml + plot_w * (double)i / 6.0;
        
        fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, (double)mt, x, (double)(height - mb));
        
        if (fs_hz > 0.0) {
            /* If we have a sampling rate, convert sample index to microseconds */
            const double t_us = (xv / fs_hz) * 1e6;
            if (fabs(t_us) < 0.01) {
                fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.6f</text>\n", x, (double)(height - mb + 18), t_us);
            } else {
                fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.3f</text>\n", x, (double)(height - mb + 18), t_us);
            }
        } else {
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", x, (double)(height - mb + 18), xv);
        }
    }

    /* Horizontal grid lines */
    for (i = 0u; i <= 6u; ++i) {
        char y_label[32];
        const double yv = ymin + y_span * (double)i / 6.0;
        const double y = (double)(height - mb) - (yv - ymin) * y_scale;
        format_axis_tick_value(yv, y_label, sizeof(y_label));
        fprintf(f, "<line x1=\"%d\" y1=\"%.2f\" x2=\"%d\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", ml, y, width - mr, y);
        fprintf(f, "<text x=\"%d\" y=\"%.2f\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%s</text>\n", ml - 8, y + 4.0, y_label);
    }

    /* Axis labels */
    if (fs_hz > 0.0) {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Time (us)</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    } else {
        fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Sample index</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
    }
    fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Amplitude (V)</text>\n", (double)(mt + plot_h * 0.5));

    /* Received signal polyline (orange) */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#f97316\" stroke-width=\"1.8\" points=\"");
    for (i = 0u; i < nout; ++i) {
        const double v = trace_sample_real_linear(sig, window_n, i, nout);
        const double x = (double)ml + (double)i * x_scale;
        const double y = (double)(height - mb) - (v - ymin) * y_scale;
        fprintf(f, "%.2f,%.2f ", x, y);
    }
    fprintf(f, "\"/>\n");

    /* Legend */
    fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"5\" fill=\"#f97316\"/>\n", (double)ml + 18.0, (double)mt + 20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Received</text>\n", (double)ml + 32.0, (double)mt + 24.0);

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
 * write_constellation_stage_artifacts — Write CSV + single constellation SVG per stage
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
 *   Input stage:  {prefix}_input.csv and {prefix}_input_constellation{metric_tag}.svg
 *   Other stages: {prefix}_stage_{NN}_{slug}.csv and {prefix}_stage_{NN}_{slug}_constellation{metric_tag}.svg
 *
 * Parameters:
 *   csv_run_dir    — Directory for CSV output (e.g., "out/baseband")
 *   svg_run_dir    — Directory for SVG output (e.g., "out/baseband")
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

    /* Emit one dots/clouds SVG per stage using the current 4x density renderer. */
    if (is_input) {
        snprintf(csv_path, sizeof(csv_path), "%s/%s_input.csv", csv_run_dir, file_prefix);
        snprintf(svg_path, sizeof(svg_path), "%s/%s_input.svg", svg_run_dir, file_prefix);
        snprintf(title, sizeof(title), "%.32s input%.160s", title_prefix, info.metric_suffix);
    } else {
        snprintf(csv_path, sizeof(csv_path), "%s/%s_stage_%02zu_%s.csv", csv_run_dir, file_prefix, stage_number, info.stage_slug);
        snprintf(svg_path, sizeof(svg_path), "%s/%s_stage_%02zu.svg", svg_run_dir, file_prefix, stage_number);
        snprintf(title, sizeof(title), "%.32s Stage %02zu: %s%.160s", title_prefix, stage_number, info.stage_label, info.metric_suffix);
    }

    write_constellation_csv(csv_path, ref, sig, nsym);
    /* Generate constellation SVG for dense output matching old version */
    if (1) {
        write_constellation_svg(svg_path, constellation_template, constellation_count, ref, sig, nsym, title);
    }
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
    double fs_hz,
    double signal_hz) {
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
        snprintf(svg_path, sizeof(svg_path), "%s/%s_input.svg", svg_run_dir, file_prefix);
        snprintf(title, sizeof(title), "%.32s input", title_prefix);
    } else {
        snprintf(csv_path, sizeof(csv_path), "%s/%s_stage_%02zu_%s.csv", csv_run_dir, file_prefix, stage_number, info.stage_slug);
        snprintf(svg_path, sizeof(svg_path), "%s/%s_stage_%02zu_%s.svg", svg_run_dir, file_prefix, stage_number, info.stage_slug);
        snprintf(title, sizeof(title), "%.32s stage %02zu - %.120s", title_prefix, stage_number, info.stage_label);
    }

    write_real_trace_csv(csv_path, ref, sig, n, max_points);
    {
        const size_t trace_window_samples = trace_zoom_window_samples(n, fs_hz, stage_number, is_input, signal_hz, -1.0);
        write_trace_svg(svg_path, ref, sig, n, max_points, title, fs_hz, trace_window_samples);
    }
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
static int write_complex_trace_svg(
    const char* path,
    const Complex* sig,
    size_t n,
    size_t max_points,
    const char* title,
    double fs_hz,
    size_t window_samples,
    const Complex* overlay_sig,
    size_t overlay_n,
    const char* overlay_label) {
    FILE* f;
    size_t i;
    size_t window_n = n;
    size_t nout;
    const int width = 1100, height = 600;
    const int ml = 80, mr = 30, mt = 50, mb = 80;
    const double plot_w = width - ml - mr;
    const double plot_h = height - mt - mb;
    
    double ymin = DBL_MAX;
    double ymax = -DBL_MAX;

    if (window_samples > 1u && window_samples < window_n) {
        window_n = window_samples;
    }
    if (window_n <= 1u) {
        nout = window_n;
    } else {
        nout = (max_points >= 2u) ? max_points : 2u;
    }
    if (nout == 0u) {
        return -1;
    }

    const int is_baseband_trace = (max_points <= 500u);

    if (is_baseband_trace) {
        nout = window_n;
    }

    for (i = 0u; i < nout; ++i) {
        const Complex v = is_baseband_trace ? sig[i] : trace_sample_complex_linear(sig, window_n, i, nout);
        ymin = fmin(ymin, fmin(v.re, v.im));
        ymax = fmax(ymax, fmax(v.re, v.im));
    }
    if (overlay_sig && overlay_n > 1u) {
        for (i = 0u; i < nout; ++i) {
            const Complex v = is_baseband_trace ? overlay_sig[i] : trace_sample_complex_linear(overlay_sig, overlay_n, i, nout);
            ymin = fmin(ymin, fmin(v.re, v.im));
            ymax = fmax(ymax, fmax(v.re, v.im));
        }
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

    {
        double y_span = ymax - ymin;
        double x_scale = plot_w / (double)((nout > 1u) ? (nout - 1u) : 1u);
        double y_scale = plot_h / y_span;
    
        f = fopen(path, "w");
        if(!f) return -1;
    
        fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);
        fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");
        fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">%s</text>\n", ml, title ? title : "Signal Trace");
        fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%.2f\" height=\"%.2f\" fill=\"white\" stroke=\"#d1d5db\"/>\n", ml, mt, plot_w, plot_h);
    
        /* Horizontal grid lines with labels */
        for (i = 0u; i <= 6u; ++i) {
            char y_label[32];
            const double yv = ymin + y_span * (double)i / 6.0;
            const double y = (double)(height - mb) - (yv - ymin) * y_scale;
            format_axis_tick_value(yv, y_label, sizeof(y_label));
            fprintf(f, "<line x1=\"%d\" y1=\"%.2f\" x2=\"%d\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", ml, y, width - mr, y);
            fprintf(f, "<text x=\"%d\" y=\"%.2f\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%s</text>\n", ml - 8, y + 4.0, y_label);
        }
    
        /* Vertical grid lines with labels */
        for (i = 0u; i <= 6u; ++i) {
            const double xv = (double)((window_n > 1u) ? (window_n - 1u) : 0u) * (double)i / 6.0;
            const double x = (double)ml + plot_w * (double)i / 6.0;
            fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, (double)mt, x, (double)(height - mb));

            if (fs_hz > 0.0) {
                const double t_us = (xv / fs_hz) * 1e6;
                if (fabs(t_us) < 0.01) {
                    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.6f</text>\n", x, (double)(height - mb + 18), t_us);
                } else {
                    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.3f</text>\n", x, (double)(height - mb + 18), t_us);
                }
            } else {
                fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", x, (double)(height - mb + 18), xv);
            }
        }
    
        if (fs_hz > 0.0) {
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Time (us)</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
        } else {
            fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Sample index</text>\n", (double)(ml + plot_w * 0.5), (double)(height - 28));
        }
        fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Amplitude (V)</text>\n", (double)(mt + plot_h * 0.5));

        if (overlay_sig && overlay_n > 1u) {
            if (is_baseband_trace) {
                fprintf(f, "<polyline fill=\"none\" stroke=\"#94a3b8\" stroke-width=\"1.2\" stroke-opacity=\"0.85\" stroke-dasharray=\"5 4\" points=\"");
                for (i = 0u; i < nout; ++i) {
                    const Complex v = overlay_sig[i];
                    fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.re) * y_scale);
                }
                fprintf(f, "\"/>\n");
                fprintf(f, "<polyline fill=\"none\" stroke=\"#cbd5e1\" stroke-width=\"1.2\" stroke-opacity=\"0.85\" stroke-dasharray=\"5 4\" points=\"");
                for (i = 0u; i < nout; ++i) {
                    const Complex v = overlay_sig[i];
                    fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.im) * y_scale);
                }
                fprintf(f, "\"/>\n");
            } else {
                fprintf(f, "<polyline fill=\"none\" stroke=\"#94a3b8\" stroke-width=\"1.2\" stroke-opacity=\"0.85\" stroke-dasharray=\"5 4\" points=\"");
                for (i = 0u; i < nout; ++i) {
                    const Complex v = trace_sample_complex_linear(overlay_sig, overlay_n, i, nout);
                    fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.re) * y_scale);
                }
                fprintf(f, "\"/>\n");
                fprintf(f, "<polyline fill=\"none\" stroke=\"#cbd5e1\" stroke-width=\"1.2\" stroke-opacity=\"0.85\" stroke-dasharray=\"5 4\" points=\"");
                for (i = 0u; i < nout; ++i) {
                    const Complex v = trace_sample_complex_linear(overlay_sig, overlay_n, i, nout);
                    fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.im) * y_scale);
                }
                fprintf(f, "\"/>\n");
            }
        }

        if (is_baseband_trace) {
            fprintf(f, "<polyline fill=\"none\" stroke=\"#3b82f6\" stroke-width=\"1.5\" stroke-opacity=\"0.8\" points=\"");
            for (i = 0u; i < nout; ++i) {
                const Complex v = sig[i];
                fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.re) * y_scale);
            }
            fprintf(f, "\"/>\n");
            fprintf(f, "<polyline fill=\"none\" stroke=\"#ef4444\" stroke-width=\"1.5\" stroke-opacity=\"0.8\" points=\"");
            for (i = 0u; i < nout; ++i) {
                const Complex v = sig[i];
                fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.im) * y_scale);
            }
            fprintf(f, "\"/>\n");

            for (i = 0u; i < nout; ++i) {
                const double cx = (double)ml + (double)i * x_scale;
                const double iy = (double)mt + (ymax - sig[i].re) * y_scale;
                const double qy = (double)mt + (ymax - sig[i].im) * y_scale;
                fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"2.5\" fill=\"#3b82f6\" stroke=\"#1e40af\" stroke-width=\"0.8\"/>\n", cx, iy);
                fprintf(f, "<circle cx=\"%.2f\" cy=\"%.2f\" r=\"2.5\" fill=\"#ef4444\" stroke=\"#991b1b\" stroke-width=\"0.8\"/>\n", cx, qy);
            }

            {
                const size_t samples_per_symbol = (fs_hz > 0.0 && window_n > 1u) ? (size_t)((double)window_n / 20.0) : 1u;
                if (samples_per_symbol > 1u && samples_per_symbol < nout) {
                    size_t sym_idx;
                    for (sym_idx = 0u; sym_idx * samples_per_symbol < nout; ++sym_idx) {
                        const double sx = (double)ml + (double)(sym_idx * samples_per_symbol) * x_scale;
                        fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#64748b\" stroke-width=\"2.0\" stroke-dasharray=\"4 3\" opacity=\"0.8\"/>\n",
                                sx, (double)mt, sx, (double)(height - mb));
                    }
                }
            }
        } else {
            fprintf(f, "<polyline fill=\"none\" stroke=\"#3b82f6\" stroke-width=\"1.5\" stroke-opacity=\"0.8\" points=\"");
            for (i = 0u; i < nout; ++i) {
                const Complex v = trace_sample_complex_linear(sig, window_n, i, nout);
                fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.re) * y_scale);
            }
            fprintf(f, "\"/>\n");
            fprintf(f, "<polyline fill=\"none\" stroke=\"#ef4444\" stroke-width=\"1.5\" stroke-opacity=\"0.8\" points=\"");
            for (i = 0u; i < nout; ++i) {
                const Complex v = trace_sample_complex_linear(sig, window_n, i, nout);
                fprintf(f, "%.2f,%.2f ", (double)ml + (double)i * x_scale, (double)mt + (ymax - v.im) * y_scale);
            }
            fprintf(f, "\"/>\n");
        }

        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"14\" fill=\"#3b82f6\">In-Phase (I)</text>\n", ml + 10, mt + (int)plot_h + 25);
        fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"14\" fill=\"#ef4444\">Quadrature (Q)</text>\n", ml + 10, mt + (int)plot_h + 45);
        if (is_baseband_trace) {
            fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"monospace\" font-size=\"12\" fill=\"#64748b\">%zu samples | dots=actual symbols | dashed=symbol boundaries</text>\n", ml + 10, mt + (int)plot_h + 65, nout);
        }
        if (overlay_sig && overlay_n > 1u) {
            const int overlay_y = is_baseband_trace ? mt + (int)plot_h + 81 : mt + (int)plot_h + 65;
            const char* lbl = overlay_label ? overlay_label : "Previous stage";
            fprintf(f, "<text x=\"%d\" y=\"%d\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#64748b\">%s I/Q (dashed)</text>\n", ml + 10, overlay_y, lbl);
        }
        fprintf(f, "</svg>\n");
        fclose(f);
        return 0;
    }
}

/*
 * write_spectrum_svg — Generate a frequency-domain spectrum plot as SVG
 *
 * What it does:
 *   Creates an SVG image showing a frequency-domain magnitude plot (spectrum).
 *
 *   Used for visualizing the power spectral density or frequency response
 *   of a signal, with frequency on the X axis and magnitude in dB on the
 *   Y axis (range -120 dB to 0 dB).
 *
 * Layout:
 *   - 980×760 pixel canvas with margins matching write_trace_svg
 *   - X-axis: Frequency in Hz (0 to fs_hz/2)
 *   - Y-axis: Magnitude in dB (-120 to 0)
 *   - Grid lines at 10 dB intervals
 *   - Filled blue region under the curve
 *   - Noise floor region highlighted
 *   - Self-contained SVG (no external CSS)
 *
 * Parameters:
 *   path     — SVG file path to write
 *   freq_hz  — Array of frequency values in Hz (one per bin)
 *   mag_dB   — Array of magnitude values in dB (one per bin)
 *   n_bins   — Number of frequency bins
 *   fs_hz    — Sampling rate in Hz (used for axis labels)
 *   title    — Chart title
 *
 * Returns:
 *   0 on success, -1 if arguments invalid, -2 if file can't be opened
 */
int write_spectrum_svg(const char* path, const double* freq_hz, const double* mag_dB, size_t n_bins, double fs_hz, const char* title) {
    FILE* f;
    size_t i;

    const int width = 980;
    const int height = 760;
    const int ml = 80;    /* left margin */
    const int mr = 30;    /* right margin */
    const int mt = 50;    /* top margin */
    const int mb = 80;    /* bottom margin */
    const double plot_w = (double)(width - ml - mr);
    const double plot_h = (double)(height - mt - mb);

    if (!path || !freq_hz || !mag_dB || n_bins == 0u) {
        return -1;
    }

    /* Compute data range for Y-axis */
    double min_mag = mag_dB[0], max_mag = mag_dB[0];
    size_t mi;
    for (mi = 1u; mi < n_bins; mi++) {
        if (mag_dB[mi] < min_mag) min_mag = mag_dB[mi];
        if (mag_dB[mi] > max_mag) max_mag = mag_dB[mi];
    }
    /* Round to neat 10 dB ticks with headroom */
    double y_max = ceil(max_mag / 10.0) * 10.0 + 10.0;
    double y_min = floor(min_mag / 10.0) * 10.0 - 10.0;
    if (y_max - y_min < 80.0) y_min = y_max - 80.0;  /* ensure min 80 dB range */
    if (y_max - y_min > 100.0) y_min = y_max - 100.0;
    const double y_span = y_max - y_min;
    const double y_scale = plot_h / y_span;

    double f_max, x_scale;

    /* X axis range: use data range when zoomed (freq[0] > 0), else 0 to fs/2 */
    double f_min = 0.0;
    f_max = (fs_hz > 0.0) ? fs_hz / 2.0 : freq_hz[n_bins - 1u];
    if (fs_hz <= 0.0 && fabs(freq_hz[0]) > 0.0) {
        f_min = freq_hz[0];
    }
    if (f_max <= 0.0) f_max = 1.0;
    x_scale = plot_w / (f_max - f_min);

    f = fopen(path, "w");
    if (!f) {
        return -2;
    }

    /* ---- SVG header, background, title, plot area ---- */
    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);
    fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");
    fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">%s</text>\n", ml, title ? title : "Spectrum");
    fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%.2f\" height=\"%.2f\" fill=\"white\" stroke=\"#d1d5db\"/>\n", ml, mt, plot_w, plot_h);

    /* ---- Horizontal grid lines at 10 dB intervals ---- */
    {
        double dB_start = ceil(y_min / 10.0) * 10.0;
        double dB_end   = floor(y_max / 10.0) * 10.0;
        for (double dB_val = dB_start; dB_val <= dB_end + 0.01; dB_val += 10.0) {
            const double y = (double)(height - mb) - (dB_val - y_min) * y_scale;
            /* Major grid every 30 dB */
            int is_major = (fmod(dB_val, 30.0) < 0.001);
            fprintf(f, "<line x1=\"%d\" y1=\"%.2f\" x2=\"%d\" y2=\"%.2f\" stroke=\"%s\" stroke-width=\"%d\"/>\n",
                    ml, y, width - mr, y,
                    is_major ? "#cbd5e1" : "#e5e7eb", 1);
            fprintf(f, "<text x=\"%d\" y=\"%.2f\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n",
                    ml - 8, y + 4.0, dB_val);
        }
    }

    /* ---- Vertical grid lines (frequency) ---- */
    {
        /* Choose nice frequency step based on f_max */
        double freq_step;
        double nice_steps[] = {1e3, 2e3, 5e3, 10e3, 20e3, 50e3, 100e3, 200e3, 500e3, 1e6, 2e6, 5e6, 10e6, 20e6, 50e6};
        size_t n_nice = sizeof(nice_steps) / sizeof(nice_steps[0]);
        size_t si;

        /* Pick a step that gives roughly 5-8 grid lines based on span */
        freq_step = nice_steps[0];
        {
            double span = f_max - f_min;
            for (si = 0u; si < n_nice; ++si) {
                if (span / nice_steps[si] <= 8.0 && span / nice_steps[si] >= 3.0) {
                    freq_step = nice_steps[si];
                    break;
                }
            }
            if (span / freq_step > 8.0 || span / freq_step < 3.0) {
                freq_step = span / 6.0;
            }
        }

        /* Start from the first grid line >= f_min */
        double fv = floor(f_min / freq_step) * freq_step;
        if (fv < f_min) fv += freq_step;
        for (; fv <= f_max + freq_step * 0.01; fv += freq_step) {
            const double x = (double)ml + (fv - f_min) * x_scale;
            if (x > (double)(width - mr) + 0.5) break;

            fprintf(f, "<line x1=\"%.2f\" y1=\"%d\" x2=\"%.2f\" y2=\"%d\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, mt, x, height - mb);

            /* Format frequency label (handle negative frequencies) */
            {
                double abs_fv = (fv < 0.0) ? -fv : fv;
                if (abs_fv >= 1e6) {
                    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%s%.1fM</text>\n", x, (double)(height - mb + 18), fv < 0.0 ? "-" : "", abs_fv / 1e6);
                } else if (abs_fv >= 1e3) {
                    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%s%.0fk</text>\n", x, (double)(height - mb + 18), fv < 0.0 ? "-" : "", abs_fv / 1e3);
                } else {
                    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", x, (double)(height - mb + 18), fv);
                }
            }
        }
    }

    /* ---- Axis labels ---- */
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Frequency (Hz)</text>\n",
            (double)(ml + (int)plot_w / 2), (double)(height - 28));
    fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Magnitude (dB)</text>\n",
            (double)(mt + (int)plot_h / 2));

    /* ---- Filled blue region under the curve ---- */
    fprintf(f, "<polygon fill=\"#3b82f6\" fill-opacity=\"0.15\" stroke=\"none\" points=\"");
    /* Start at bottom-left of the curve */
    fprintf(f, "%.2f,%.2f ", (double)ml, (double)(height - mb));
    for (i = 0u; i < n_bins; ++i) {
        const double x = (double)ml + (freq_hz[i] - f_min) * x_scale;
        const double clamped_dB = (mag_dB[i] < y_min) ? y_min : ((mag_dB[i] > y_max) ? y_max : mag_dB[i]);
        const double y = (double)(height - mb) - (clamped_dB - y_min) * y_scale;
        if (x >= (double)ml && x <= (double)(width - mr)) {
            fprintf(f, "%.2f,%.2f ", x, y);
        }
    }
    /* Close polygon at bottom-right */
    {
        const double last_x = (double)ml + (freq_hz[n_bins - 1u] - f_min) * x_scale;
        const double clamped_last_x = (last_x > (double)(width - mr)) ? (double)(width - mr) : last_x;
        fprintf(f, "%.2f,%.2f ", clamped_last_x, (double)(height - mb));
    }
    fprintf(f, "\"/>\n");

    /* ---- Spectrum line (blue) ---- */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#3b82f6\" stroke-width=\"1.5\" points=\"");
    for (i = 0u; i < n_bins; ++i) {
        const double x = (double)ml + (freq_hz[i] - f_min) * x_scale;
        const double clamped_dB = (mag_dB[i] < y_min) ? y_min : ((mag_dB[i] > y_max) ? y_max : mag_dB[i]);
        const double y = (double)(height - mb) - (clamped_dB - y_min) * y_scale;
        if (x >= (double)ml && x <= (double)(width - mr)) {
            fprintf(f, "%.2f,%.2f ", x, y);
        }
    }
    fprintf(f, "\"/>\n");

    /* ---- Legend ---- */
    fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#3b82f6\" stroke-width=\"2\"/>\n", (double)ml + 18.0, (double)mt + 20.0, (double)ml + 38.0, (double)mt + 20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Spectrum</text>\n", (double)ml + 44.0, (double)mt + 24.0);

    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}

int write_dual_spectrum_svg(const char* path, const double* freq_hz,
                            const double* mag1_dB, const double* mag2_dB,
                            size_t n_bins, double fs_hz, const char* title) {
    FILE* f;
    size_t i;
    const int width = 980, height = 760, ml = 80, mr = 30, mt = 50, mb = 80;
    const double plot_w = (double)(width - ml - mr);
    const double plot_h = (double)(height - mt - mb);

    if (!path || !freq_hz || !mag1_dB || !mag2_dB || n_bins == 0u) return -1;

    double min_mag = mag1_dB[0], max_mag = mag1_dB[0];
    for (i = 1u; i < n_bins; i++) {
        if (mag1_dB[i] < min_mag) min_mag = mag1_dB[i];
        if (mag1_dB[i] > max_mag) max_mag = mag1_dB[i];
        if (mag2_dB[i] < min_mag) min_mag = mag2_dB[i];
        if (mag2_dB[i] > max_mag) max_mag = mag2_dB[i];
    }
    double y_max = ceil(max_mag / 10.0) * 10.0 + 10.0;
    double y_min = floor(min_mag / 10.0) * 10.0 - 10.0;
    if (y_max - y_min < 80.0) y_min = y_max - 80.0;
    const double y_span = y_max - y_min;
    const double y_scale = plot_h / y_span;
    double f_min = 0.0, f_max, x_scale;

    f_max = (fs_hz > 0.0) ? fs_hz / 2.0 : freq_hz[n_bins - 1u];
    if (fs_hz <= 0.0 && fabs(freq_hz[0]) > 0.0) f_min = freq_hz[0];
    if (f_max <= 0.0) f_max = 1.0;
    x_scale = plot_w / (f_max - f_min);

    f = fopen(path, "w");
    if (!f) return -2;

    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n", width, height, width, height);
    fprintf(f, "<rect width=\"100%%\" height=\"100%%\" fill=\"#f8fafc\"/>\n");
    fprintf(f, "<text x=\"%d\" y=\"30\" font-family=\"sans-serif\" font-size=\"24\" fill=\"#111827\">%s</text>\n", ml, title ? title : "Spectrum (I/Q)");
    fprintf(f, "<rect x=\"%d\" y=\"%d\" width=\"%.2f\" height=\"%.2f\" fill=\"white\" stroke=\"#d1d5db\"/>\n", ml, mt, plot_w, plot_h);

    /* Horizontal grid lines at 10 dB */
    {
        double ds = ceil(y_min / 10.0) * 10.0;
        double de = floor(y_max / 10.0) * 10.0;
        for (double dv = ds; dv <= de + 0.01; dv += 10.0) {
            double y = (double)(height - mb) - (dv - y_min) * y_scale;
            fprintf(f, "<line x1=\"%d\" y1=\"%.2f\" x2=\"%d\" y2=\"%.2f\" stroke=\"%s\" stroke-width=\"1\"/>\n", ml, y, width - mr, y, fmod(dv, 30.0) < 0.001 ? "#cbd5e1" : "#e5e7eb");
            fprintf(f, "<text x=\"%d\" y=\"%.2f\" text-anchor=\"end\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", ml - 8, y + 4.0, dv);
        }
    }

    /* Vertical grid lines */
    {
        double span = f_max - f_min, step;
        double nice[] = {1e3,2e3,5e3,10e3,20e3,50e3,100e3,200e3,500e3,1e6,2e6,5e6,10e6,20e6,50e6};
        size_t ns = sizeof(nice)/sizeof(nice[0]), si;
        for (step = nice[0], si = 0u; si < ns; si++) {
            if (span/nice[si] <= 8.0 && span/nice[si] >= 3.0) { step = nice[si]; break; }
        }
        if (span/step > 8.0 || span/step < 3.0) step = span/6.0;
        double fv = floor(f_min/step)*step;
        if (fv < f_min) fv += step;
        for (; fv <= f_max+step*0.01; fv += step) {
            double x = (double)ml + (fv-f_min)*x_scale;
            if (x > (double)(width-mr)+0.5) break;
            fprintf(f, "<line x1=\"%.2f\" y1=\"%d\" x2=\"%.2f\" y2=\"%d\" stroke=\"#e5e7eb\" stroke-width=\"1\"/>\n", x, mt, x, height-mb);
            double af = fv < 0 ? -fv : fv;
            const char *sign = fv < 0 ? "-" : "";
            if (af >= 1e6) fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%s%.1fM</text>\n", x, (double)(height-mb+18), sign, af/1e6);
            else if (af >= 1e3) fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%s%.0fk</text>\n", x, (double)(height-mb+18), sign, af/1e3);
            else fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#475569\">%.0f</text>\n", x, (double)(height-mb+18), fv);
        }
    }

    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Frequency (Hz)</text>\n", (double)(ml+(int)plot_w/2), (double)(height-28));
    fprintf(f, "<text transform=\"translate(24,%.2f) rotate(-90)\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">Magnitude (dB)</text>\n", (double)(mt+(int)plot_h/2));

    /* Filled region under I trace */
    fprintf(f, "<polygon fill=\"#3b82f6\" fill-opacity=\"0.10\" stroke=\"none\" points=\"");
    fprintf(f, "%.2f,%.2f ", (double)ml, (double)(height-mb));
    for (i = 0u; i < n_bins; i++) {
        double x = (double)ml + (freq_hz[i]-f_min)*x_scale;
        double c = mag1_dB[i] < y_min ? y_min : (mag1_dB[i] > y_max ? y_max : mag1_dB[i]);
        double y = (double)(height-mb) - (c-y_min)*y_scale;
        if (x >= ml && x <= width-mr) fprintf(f, "%.2f,%.2f ", x, y);
    }
    { double lx = (double)ml + (freq_hz[n_bins-1]-f_min)*x_scale; if (lx > width-mr) lx = width-mr; fprintf(f, "%.2f,%.2f ", lx, (double)(height-mb)); }
    fprintf(f, "\"/>\n");

    /* I trace line */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#3b82f6\" stroke-width=\"1.5\" points=\"");
    for (i = 0u; i < n_bins; i++) {
        double x = (double)ml + (freq_hz[i]-f_min)*x_scale;
        double c = mag1_dB[i] < y_min ? y_min : (mag1_dB[i] > y_max ? y_max : mag1_dB[i]);
        double y = (double)(height-mb) - (c-y_min)*y_scale;
        if (x >= ml && x <= width-mr) fprintf(f, "%.2f,%.2f ", x, y);
    }
    fprintf(f, "\"/>\n");

    /* Q trace line (orange) */
    fprintf(f, "<polyline fill=\"none\" stroke=\"#f97316\" stroke-width=\"1.5\" points=\"");
    for (i = 0u; i < n_bins; i++) {
        double x = (double)ml + (freq_hz[i]-f_min)*x_scale;
        double c = mag2_dB[i] < y_min ? y_min : (mag2_dB[i] > y_max ? y_max : mag2_dB[i]);
        double y = (double)(height-mb) - (c-y_min)*y_scale;
        if (x >= ml && x <= width-mr) fprintf(f, "%.2f,%.2f ", x, y);
    }
    fprintf(f, "\"/>\n");

    /* Legend */
    fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#3b82f6\" stroke-width=\"2\"/>\n", ml+18.0, mt+20.0, ml+38.0, mt+20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">I</text>\n", (double)ml+44.0, (double)mt+24.0);
    fprintf(f, "<line x1=\"%.2f\" y1=\"%.2f\" x2=\"%.2f\" y2=\"%.2f\" stroke=\"#f97316\" stroke-width=\"2\"/>\n", ml+76.0, mt+20.0, ml+96.0, mt+20.0);
    fprintf(f, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#111827\">Q</text>\n", (double)ml+102.0, (double)mt+24.0);

    fprintf(f, "</svg>\n");
    fclose(f);
    return 0;
}


void write_complex_trace_stage_artifacts(const char* svg_run_dir, const char* file_prefix, size_t stage_number, int is_input, const char* raw_stage_name, const StageMetric* metric, const Complex* sig, size_t nsym, double fs_hz, double signal_hz, double display_window_us) {
    static Complex* rf_prev_postmix_window = NULL;
    static size_t rf_prev_postmix_n = 0u;
    char svg_path[1024];
    char title[384];
    StageRenderInfo info;
    const int is_rf_postmix_stage = (!is_input && file_prefix && strcmp(file_prefix, "rf") == 0 && stage_number >= 6u);
    const int is_limiter_stage = (raw_stage_name && strstr(raw_stage_name, "limiter") != NULL);
    const size_t trace_window_samples = trace_zoom_window_samples(nsym, fs_hz, stage_number, is_input, signal_hz, display_window_us);
    const size_t window_n = (trace_window_samples > 1u && trace_window_samples < nsym) ? trace_window_samples : nsym;
    const Complex* overlay_sig = NULL;
    size_t overlay_n = 0u;
    if (!svg_run_dir || !file_prefix || !sig || nsym == 0) return;
    build_stage_render_info(raw_stage_name, metric, &info);
    if(is_input) {
        snprintf(svg_path, sizeof(svg_path), "%s/%s_input_trace.svg", svg_run_dir, file_prefix);
        snprintf(title, sizeof(title), "Input Signal Trace");
    } else {
        snprintf(svg_path, sizeof(svg_path), "%s/%s_stage_%02zu_%s_trace.svg", svg_run_dir, file_prefix, stage_number, info.stage_slug);
        snprintf(title, sizeof(title), "Stage %02zu - %.120s Trace", stage_number, info.stage_label);
    }

    if (is_rf_postmix_stage && !is_limiter_stage && stage_number >= 7u && rf_prev_postmix_window && rf_prev_postmix_n > 1u) {
        overlay_sig = rf_prev_postmix_window;
        overlay_n = rf_prev_postmix_n;
    }

    {
        const size_t trace_render_points = (stage_number >= 6u) ? 400u : 8000u;  /* 8000 render points for smooth interpolation on all stages */
        write_complex_trace_svg(svg_path, sig, nsym, trace_render_points, title, fs_hz, trace_window_samples, overlay_sig, overlay_n, "Previous stage");
    }

    if (is_rf_postmix_stage && window_n > 1u) {
        if (rf_prev_postmix_n != window_n) {
            Complex* resized = (Complex*)realloc(rf_prev_postmix_window, window_n * sizeof(Complex));
            if (resized) {
                rf_prev_postmix_window = resized;
                rf_prev_postmix_n = window_n;
            }
        }
        if (rf_prev_postmix_window && rf_prev_postmix_n == window_n) {
            memcpy(rf_prev_postmix_window, sig, window_n * sizeof(Complex));
        }
    }
}
