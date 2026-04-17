/*
 * ============================================================================
 * stage_models.c — CSV-Driven Receiver Stage Configuration Loader
 * ============================================================================
 *
 * PURPOSE:
 *   This file is responsible for reading the receiver's hardware design
 *   parameters from a CSV file and making them available to the simulator.
 *
 *   In a real satellite receiver, the signal passes through a chain of
 *   hardware components (stages): Band-Pass Filters (BPF), Low-Noise
 *   Amplifiers (LNA), Mixers, Low-Pass Filters (LPF), etc. Each stage
 *   has measurable properties:
 *     - Gain (dB): How much the stage amplifies (positive) or attenuates
 *       (negative) the signal. For example, an LNA might have +25 dB gain.
 *     - Noise Figure (NF, dB): How much extra noise the stage adds to the
 *       signal. A perfect stage has NF = 0 dB. Real stages always have NF > 0.
 *     - Filter Length: For stages that act as filters, this controls how
 *       many samples the moving-average filter spans.
 *
 *   Rather than hard-coding these values into the C source code (which would
 *   require recompilation every time you change a parameter), this module
 *   reads them from a CSV file at runtime. This lets users experiment with
 *   different receiver designs by simply editing a spreadsheet.
 *
 * DATA MODEL:
 *   The simulator supports THREE separate receiver chains, each modeling
 *   a different signal path:
 *
 *   Chain 0 — baseband_rx:
 *     The "analytical" complex baseband path. Processes I/Q symbols directly
 *     without RF translation. Fast but doesn't capture real RF effects.
 *
 *   Chain 1 — rf_frontend:
 *     The RF front-end path. Processes the real RF signal (at 24 GHz) before
 *     it is downconverted to baseband. Includes the bandpass filter and LNA
 *     that operate in the RF domain.
 *
 *   Chain 2 — rf_postmix_bb:
 *     The baseband path AFTER the mixer has downconverted the RF signal.
 *     Includes the low-pass filter and final amplifier that operate on the
 *     recovered baseband signal.
 *
 *   Together, chains 1 and 2 form the complete "brute-force RF" simulation
 *   path: signal → rf_frontend stages → mixer downconversion → rf_postmix_bb stages.
 *
 * CSV FORMAT SUPPORT:
 *   This loader supports two CSV formats:
 *
 *   1. CANONICAL FORMAT (modern, recommended):
 *      chain, name, gain_db, nf_db, [filter_len], [auto_gain_to_vpp], [target_vpp], [enabled]
 *      Example row: baseband_rx, lna, 25.0, 1.2, 1, 0, 0.0, 1
 *
 *   2. LEGACY FORMAT (backward-compatible):
 *      component, gain_db, nf_db
 *      Example row: LNA 1, 25.0, 1.2
 *      The "component" name is mapped to the canonical chain+stage via
 *      hard-coded rules in map_legacy_component().
 *
 *   The loader automatically detects which format is in use by checking
 *   whether a "chain" column exists in the CSV header.
 *
 * ============================================================================
 */

#include "stage_models.h"   /* Public types: StageModel, StageModelsConfig, StageChainId */

#include <ctype.h>     /* For isalnum(), tolower(), isspace() — character classification */
#include <errno.h>     /* For errno — system error code set by strtod/strtol */
#include <stdarg.h>    /* For va_list, va_start, va_end — variadic function support */
#include <stdio.h>     /* For FILE, fopen, fgets, fclose, vsnprintf, fprintf */
#include <stdlib.h>    /* For malloc, realloc, free, strtod, strtol */
#include <string.h>    /* For strlen, strcmp, strcspn, memset, memcpy */

/*
 * STAGE_CHAIN_COUNT is defined in stage_models.h as 3.
 * It represents the number of distinct receiver chains (baseband_rx,
 * rf_frontend, rf_postmix_bb).
 */

/*
 * MAX_FIELDS — Maximum number of comma-separated fields we can handle per CSV line.
 * If a CSV line has more than 32 fields, the extras are silently ignored.
 * 32 is extremely generous; typical lines have 4-8 fields.
 */
#define MAX_FIELDS   32

/*
 * MAX_LINE_LEN — Maximum length (in characters) of a single CSV line.
 * Lines longer than 2048 characters will be truncated, potentially causing
 * parse errors. This is more than enough for any reasonable stage definition.
 */
#define MAX_LINE_LEN 2048


/* ============================================================================
 * ERROR REPORTING HELPERS
 * ============================================================================ */

/*
 * set_error — Write a formatted error message into the caller's error buffer
 *
 * What it does:
 *   Works exactly like printf(), but writes into a fixed-size buffer provided
 *   by the caller instead of stdout. Uses vsnprintf() to prevent buffer overflow.
 *
 * Why it exists:
 *   When CSV loading fails, the user needs to know WHY — wrong column name,
 *   missing file, invalid number, etc. This function stores a human-readable
 *   error message that the caller can display.
 *
 * Parameters:
 *   errbuf      — Pointer to the caller's error message buffer
 *   errbuf_size — Size of that buffer in bytes (to prevent overflow)
 *   fmt         — printf-style format string (e.g., "Error at line %d: %s")
 *   ...         — Additional arguments matching the format placeholders
 *
 * Safety:
 *   - If errbuf is NULL or errbuf_size is 0, does nothing (no crash).
 *   - vsnprintf automatically truncates if the message is too long.
 */
static void set_error(char* errbuf, size_t errbuf_size, const char* fmt, ...) {
    va_list args;   /* Holds the variable-length argument list */

    /* Guard against NULL pointers or zero-size buffers */
    if (!errbuf || errbuf_size == 0u || !fmt) {
        return;
    }

    va_start(args, fmt);                          /* Initialize the argument list */
    vsnprintf(errbuf, errbuf_size, fmt, args);    /* Format the error message */
    va_end(args);                                 /* Clean up the argument list */
}


/*
 * clear_error — Reset the error buffer to empty (no error)
 *
 * What it does:
 *   Sets the first character of the error buffer to the null terminator '\0',
 *   effectively making it an empty string. Called at the start of loading
 *   to ensure no stale error messages from previous calls remain.
 *
 * Parameters:
 *   errbuf      — Pointer to the error buffer to clear
 *   errbuf_size — Size of the buffer
 */
static void clear_error(char* errbuf, size_t errbuf_size) {
    if (!errbuf || errbuf_size == 0u) {
        return;
    }
    errbuf[0] = '\0';   /* An empty string = "no error" */
}


/* ============================================================================
 * STRING MANIPULATION HELPERS
 * ============================================================================ */

/*
 * normalize_token — Convert a string to a "canonical" comparison key
 *
 * What it does:
 *   Takes a string like "Gain_dB", "gain-db", "GAIN DB", or "GainDb"
 *   and produces the normalized form "gaindb" — all lowercase, with all
 *   non-alphanumeric characters (spaces, underscores, hyphens) removed.
 *
 * Why it exists:
 *   CSV files from different sources use inconsistent formatting:
 *   - Some use "Gain_dB", others use "gain_db" or "GAIN(dB)"
 *   - Some chain names are "RF Frontend", others are "rf_frontend"
 *   Normalizing everything to a common form allows case-insensitive,
 *   punctuation-insensitive matching.
 *
 * Parameters:
 *   in       — The raw input string to normalize
 *   out      — Buffer to write the normalized result into
 *   out_size — Size of the output buffer (result will be truncated if too long)
 *
 * Example:
 *   Input:  "RF_Frontend"  →  Output: "rffrontend"
 *   Input:  "Noise Figure (dB)" → Output: "noisefiguredb"
 */
static void normalize_token(const char* in, char* out, size_t out_size) {
    size_t j = 0u;   /* Write index into the output buffer */
    size_t i;        /* Read index into the input string */

    /* Guard: ensure we have a valid output buffer */
    if (!out || out_size == 0u) {
        return;
    }
    out[0] = '\0';   /* Start with an empty string */

    if (!in) {
        return;      /* NULL input → empty output */
    }

    /* Scan each character of the input string */
    for (i = 0u; in[i] != '\0'; ++i) {
        const unsigned char c = (unsigned char)in[i];

        /*
         * Keep only alphanumeric characters (letters and digits).
         * Skip spaces, underscores, hyphens, parentheses, etc.
         * Convert uppercase letters to lowercase.
         */
        if (isalnum(c)) {
            if (j + 1u < out_size) {       /* Ensure room for this char + null terminator */
                out[j++] = (char)tolower(c);
            }
        }
    }
    out[j] = '\0';   /* Null-terminate the output string */
}


/*
 * trim_ws — Remove leading and trailing whitespace from a string (in-place)
 *
 * What it does:
 *   - Advances the string pointer past any leading spaces/tabs/newlines
 *   - Walks backward from the end and null-terminates at the last non-space character
 *
 * Why it exists:
 *   CSV fields often have accidental spaces: "  LNA  " should match "LNA".
 *   Trimming ensures clean comparison.
 *
 * Parameters:
 *   s — Pointer to the string to trim (MODIFIED in-place)
 *
 * Returns:
 *   Pointer to the start of the trimmed string. This may be different from
 *   the original `s` if leading whitespace was skipped.
 *
 * IMPORTANT: This function modifies the original string buffer by writing
 *   null bytes into trailing whitespace positions.
 */
static char* trim_ws(char* s) {
    char* end;

    if (!s) {
        return s;
    }

    /* Skip leading whitespace by advancing the pointer */
    while (*s != '\0' && isspace((unsigned char)*s)) {
        ++s;
    }

    /* If the entire string was whitespace, return empty */
    if (*s == '\0') {
        return s;
    }

    /* Find the end of the string, then walk backward to trim trailing whitespace */
    end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) {
        *end = '\0';   /* Overwrite trailing whitespace with null terminators */
        --end;
    }

    return s;   /* Return pointer to the first non-whitespace character */
}


/*
 * split_csv — Split a comma-separated line into an array of field pointers
 *
 * What it does:
 *   Scans through the string looking for comma characters. Each comma is
 *   replaced with a null byte '\0', effectively splitting the line into
 *   separate C strings. Pointers to the start of each field are stored
 *   in the `fields` array.
 *
 * Why it exists:
 *   To parse CSV lines without needing a full-featured CSV library.
 *
 * Limitations:
 *   - Does NOT handle quoted fields (e.g., "value,with,commas")
 *   - Does NOT handle escaped commas
 *   - This is acceptable because our CSV files contain only simple
 *     alphanumeric values and never need quoting.
 *
 * Parameters:
 *   line       — The CSV line to split (DESTROYED: commas become '\0')
 *   fields     — Output array of pointers to the start of each field
 *   max_fields — Maximum number of fields to extract (size of `fields` array)
 *
 * Returns:
 *   The number of fields found and stored in `fields`.
 *
 * Example:
 *   Input: "baseband_rx,lna,25.0,1.2"
 *   After: "baseband_rx\0lna\025.0\01.2"
 *   fields[0] → "baseband_rx", fields[1] → "lna", fields[2] → "25.0", fields[3] → "1.2"
 *   Returns: 4
 */
static size_t split_csv(char* line, char** fields, size_t max_fields) {
    size_t count = 0u;   /* Number of fields found so far */
    char* p = line;      /* Scanning pointer */

    if (!line || !fields || max_fields == 0u) {
        return 0u;
    }

    /* The first field starts at the beginning of the line */
    fields[count++] = p;

    /* Scan for commas; each comma marks the start of a new field */
    while (*p != '\0' && count < max_fields) {
        if (*p == ',') {
            *p = '\0';              /* Replace comma with null terminator */
            fields[count++] = p + 1; /* Next field starts right after the comma */
        }
        ++p;
    }

    return count;
}


/* ============================================================================
 * NUMBER PARSING HELPERS
 * ============================================================================ */

/*
 * parse_double_value — Safely convert a string to a double-precision float
 *
 * What it does:
 *   Uses strtod() (the C standard library's string-to-double function) to
 *   parse the input string `s` into a double value. Also checks for errors:
 *   - Invalid characters (like "abc" instead of "1.23")
 *   - Overflow/underflow
 *   - Trailing junk after the number (like "1.23xyz")
 *
 * Parameters:
 *   s   — The input string to parse (e.g., "25.3", "-1.66", "0.0")
 *   out — Pointer to where the parsed double value should be stored
 *
 * Returns:
 *   0  on success (the parsed value is written to *out)
 *   -1 if s or out is NULL (invalid arguments)
 *   -2 if the string could not be parsed as a valid number
 */
static int parse_double_value(const char* s, double* out) {
    char* end = NULL;   /* strtod will set this to the first unparsed character */
    double v;

    if (!s || !out) {
        return -1;
    }

    errno = 0;                          /* Clear errno before calling strtod */
    v = strtod(s, &end);               /* Attempt to parse */

    /*
     * Check for errors:
     *   errno != 0      : overflow, underflow, or other system error
     *   end == s         : no characters were consumed (not a number at all)
     *   *end != '\0'     : there are leftover characters after the number
     */
    if (errno != 0 || end == s || *end != '\0') {
        return -2;
    }

    *out = v;
    return 0;
}


/*
 * parse_int_value — Safely convert a string to an integer
 *
 * What it does:
 *   Same idea as parse_double_value, but for integers. Uses strtol()
 *   (string-to-long) and casts the result to int.
 *
 * Parameters:
 *   s   — The input string (e.g., "5", "1", "0")
 *   out — Pointer to where the parsed integer should be stored
 *
 * Returns:
 *   0  on success
 *   -1 if arguments are NULL
 *   -2 if the string is not a valid integer
 */
static int parse_int_value(const char* s, int* out) {
    char* end = NULL;
    long v;

    if (!s || !out) {
        return -1;
    }

    errno = 0;
    v = strtol(s, &end, 10);   /* Parse as base-10 (decimal) integer */

    if (errno != 0 || end == s || *end != '\0') {
        return -2;
    }

    *out = (int)v;   /* Narrow from long to int (safe for stage parameters) */
    return 0;
}


/*
 * dup_string — Duplicate a string by allocating new memory and copying
 *
 * What it does:
 *   This is our own implementation of POSIX strdup(), which is not part of
 *   the C standard (only POSIX). It allocates exactly enough memory for a
 *   copy of the string (including the null terminator) and copies the content.
 *
 * Why we need it:
 *   When we parse stage names from the CSV file, the name exists in a
 *   temporary line buffer that will be overwritten when the next line is read.
 *   We need to make a permanent copy of the name on the heap so it survives
 *   past the CSV parsing loop.
 *
 * Parameters:
 *   s — The string to duplicate
 *
 * Returns:
 *   A newly-allocated copy of the string (caller must free it), or NULL if
 *   allocation fails or s is NULL.
 *
 * Memory ownership:
 *   The caller (stage_models_free) is responsible for freeing the returned pointer.
 */
static char* dup_string(const char* s) {
    size_t len;
    char* out;

    if (!s) {
        return NULL;
    }

    len = strlen(s);                     /* Length WITHOUT the null terminator */
    out = (char*)malloc(len + 1u);       /* Allocate length + 1 for the '\0' */
    if (!out) {
        return NULL;                     /* malloc failed — out of memory */
    }

    memcpy(out, s, len + 1u);           /* Copy the string INCLUDING the '\0' */
    return out;
}


/* ============================================================================
 * CHAIN / STAGE MEMORY MANAGEMENT
 * ============================================================================ */

/*
 * clear_chain — Free all memory associated with a single stage chain
 *
 * What it does:
 *   1. Iterates through each stage in the chain and frees its heap-allocated
 *      name string (created by dup_string during CSV loading).
 *   2. Frees the chain array itself (the array of StageModel structs).
 *   3. Resets the chain pointer to NULL and count to 0.
 *
 * Why it exists:
 *   Proper cleanup prevents memory leaks. Each chain's stages were allocated
 *   with realloc() during loading, and each stage's name was allocated with
 *   malloc() — all of that memory must be returned to the system.
 *
 * Parameters:
 *   chain — Pointer to the chain array pointer (e.g., &cfg->chains[0])
 *   count — Pointer to the chain's element count (e.g., &cfg->counts[0])
 */
static void clear_chain(StageModel** chain, size_t* count) {
    size_t i;

    /* If the chain pointer or count pointer is NULL, nothing to do */
    if (!chain || !count || !*chain) {
        if (count) {
            *count = 0u;
        }
        return;
    }

    /* Free each stage's name string */
    for (i = 0u; i < *count; ++i) {
        free((*chain)[i].name);
        (*chain)[i].name = NULL;   /* Defensive: prevent use-after-free */
    }

    /* Free the array of StageModel structs itself */
    free(*chain);
    *chain = NULL;
    *count = 0u;
}


/*
 * stage_models_free — PUBLIC API: Release all memory owned by a StageModelsConfig
 *
 * What it does:
 *   Calls clear_chain() for all three chains (baseband_rx, rf_frontend,
 *   rf_postmix_bb), freeing every stage name and every stage array.
 *   After this call, the config is in a "clean zero" state and safe to reuse.
 *
 * Safety:
 *   - Safe to call with NULL cfg (does nothing)
 *   - Safe to call multiple times (idempotent — each call after the first is a no-op)
 *   - Safe to call on a zero-initialized config (all pointers are already NULL)
 *
 * Parameters:
 *   cfg — The configuration to free. NULL is allowed (no-op).
 */
void stage_models_free(StageModelsConfig* cfg) {
    int i;

    if (!cfg) {
        return;
    }

    /* Free each of the 3 chains (STAGE_CHAIN_COUNT = 3) */
    for (i = 0; i < STAGE_CHAIN_COUNT; ++i) {
        clear_chain(&cfg->chains[i], &cfg->counts[i]);
    }
}


/*
 * set_stage — Add a new stage to a chain, or update an existing stage by name
 *
 * What it does:
 *   1. Searches the specified chain for a stage with the same name.
 *   2. If found: updates that stage's parameters (gain, NF, filter, etc.)
 *   3. If NOT found: appends a new stage entry to the chain.
 *      - The chain array is grown by one element using realloc()
 *      - The stage name is duplicated onto the heap using dup_string()
 *
 * Why "update if exists"?
 *   In legacy mode, the same logical stage might be set multiple times
 *   (e.g., "LNA 1" maps to both baseband_rx and rf_frontend chains).
 *   The update logic prevents accidentally creating duplicate entries.
 *
 * Parameters:
 *   cfg              — The configuration to modify
 *   id               — Which chain to add to (0=baseband_rx, 1=rf_frontend, 2=rf_postmix_bb)
 *   name             — Human-readable stage name (e.g., "lna", "rf_bpf_eq")
 *   gain_db          — Stage gain in dB (positive=amplification, negative=loss)
 *   nf_db            — Noise Figure in dB (always ≥ 0)
 *   filter_len       — Moving-average filter length (1 = no filtering)
 *   auto_gain_to_vpp — If non-zero, gain is computed automatically to reach target_vpp
 *   target_vpp       — Target peak-to-peak voltage (only used if auto_gain_to_vpp != 0)
 *   errbuf           — Error message buffer (for diagnostics on failure)
 *   errbuf_size      — Size of errbuf
 *
 * Returns:
 *   0  on success
 *   -1 if arguments are invalid
 *   -2 if realloc() fails (out of memory for the stage array)
 *   -3 if dup_string() fails (out of memory for the name)
 */
static int set_stage(
    StageModelsConfig* cfg,
    StageChainId id,
    const char* name,
    double gain_db,
    double nf_db,
    int filter_len,
    int auto_gain_to_vpp,
    double target_vpp,
    char* errbuf,
    size_t errbuf_size) {

    StageModel* chain;   /* Pointer to the current chain's stage array */
    size_t count;         /* Number of stages currently in this chain */
    size_t i;

    /* Validate arguments */
    if (!cfg || !name || id < 0 || id >= STAGE_CHAIN_COUNT) {
        set_error(errbuf, errbuf_size, "Invalid stage assignment");
        return -1;
    }

    /* Clamp filter_len to minimum of 1 (1 = no filtering = pass-through) */
    if (filter_len < 1) {
        filter_len = 1;
    }

    chain = cfg->chains[id];
    count = cfg->counts[id];

    /* --- Check if a stage with this name already exists --- */
    for (i = 0u; i < count; ++i) {
        if (chain[i].name && strcmp(chain[i].name, name) == 0) {
            /* Found an existing stage with the same name — update its parameters */
            chain[i].gain_db = gain_db;
            chain[i].nf_db = nf_db;
            chain[i].filter_len = filter_len;
            chain[i].auto_gain_to_vpp = auto_gain_to_vpp ? 1 : 0;
            chain[i].target_vpp = target_vpp;
            return 0;   /* Success — updated in place */
        }
    }

    /* --- Stage not found — append a new entry --- */
    {
        /*
         * Grow the chain array by one element using realloc.
         * realloc(ptr, new_size) may move the entire array to a new location
         * in memory, which is why we need to update cfg->chains[id].
         *
         * If ptr is NULL (chain was empty), realloc behaves like malloc.
         */
        StageModel* grown = (StageModel*)realloc(chain, (count + 1u) * sizeof(StageModel));
        if (!grown) {
            set_error(errbuf, errbuf_size, "Memory allocation failed for stage list");
            return -2;
        }

        /* Update the chain pointer (may have changed due to realloc) */
        cfg->chains[id] = grown;
        cfg->counts[id] = count + 1u;

        /* Duplicate the stage name onto the heap */
        cfg->chains[id][count].name = dup_string(name);
        if (!cfg->chains[id][count].name) {
            set_error(errbuf, errbuf_size, "Memory allocation failed for stage name");
            return -3;
        }

        /* Fill in the new stage's parameters */
        cfg->chains[id][count].gain_db = gain_db;
        cfg->chains[id][count].nf_db = nf_db;
        cfg->chains[id][count].filter_len = filter_len;
        cfg->chains[id][count].auto_gain_to_vpp = auto_gain_to_vpp ? 1 : 0;
        cfg->chains[id][count].target_vpp = target_vpp;
    }

    return 0;   /* Success — new stage appended */
}


/* ============================================================================
 * CSV SCHEMA DETECTION & PARSING
 * ============================================================================ */

/*
 * parse_chain_id — Convert a flexible chain name string into a StageChainId enum value
 *
 * What it does:
 *   Normalizes the input string (lowercase, strip non-alphanumeric) and checks
 *   it against a list of known aliases for each chain. This allows the CSV file
 *   to use various spellings like "baseband_rx", "baseband", "Baseband RX",
 *   "chain0", etc.
 *
 * Recognized aliases:
 *   Chain 0 (BASEBAND_RX):   "basebandrx", "baseband", "chain0"
 *   Chain 1 (RF_FRONTEND):   "rffrontend", "rf", "chain1"
 *   Chain 2 (RF_POSTMIX_BB): "rfpostmixbb", "postmixbb", "chain2", "rftobb"
 *
 * Parameters:
 *   s      — The chain name string from the CSV
 *   out_id — Pointer to store the resulting enum value
 *
 * Returns:
 *   0  on success (chain ID stored in *out_id)
 *   -1 if arguments are NULL
 *   -2 if the chain name is not recognized
 */
static int parse_chain_id(const char* s, StageChainId* out_id) {
    char key[64];   /* Normalized version of the input */

    if (!s || !out_id) {
        return -1;
    }

    normalize_token(s, key, sizeof(key));

    /* Try matching against each chain's aliases */
    if (strcmp(key, "basebandrx") == 0 || strcmp(key, "baseband") == 0 || strcmp(key, "chain0") == 0) {
        *out_id = STAGE_CHAIN_BASEBAND_RX;
        return 0;
    }
    if (strcmp(key, "rffrontend") == 0 || strcmp(key, "rf") == 0 || strcmp(key, "chain1") == 0) {
        *out_id = STAGE_CHAIN_RF_FRONTEND;
        return 0;
    }
    if (strcmp(key, "rfpostmixbb") == 0 || strcmp(key, "postmixbb") == 0 || strcmp(key, "chain2") == 0 || strcmp(key, "rftobb") == 0) {
        *out_id = STAGE_CHAIN_RF_POSTMIX_BB;
        return 0;
    }

    return -2;   /* Unrecognized chain name */
}


/*
 * map_legacy_component — Convert old-format component names to modern stage definitions
 *
 * What it does:
 *   This is a backward-compatibility layer. Older CSV files used simple component
 *   names like "Filter 1", "LNA 1", "Mixer 1" without specifying which chain
 *   they belong to. This function contains hard-coded mappings that associate
 *   each legacy component name with the correct chain(s) and stage parameters.
 *
 * Why some components map to MULTIPLE chains:
 *   In a real receiver, some components appear in both the analytical (baseband)
 *   model and the RF model. For example, "LNA 1" affects both the baseband_rx
 *   chain and the rf_frontend chain because the LNA is shared hardware.
 *
 * Legacy component mappings:
 *   "Filter 1" (normalized: "filter1")
 *     → baseband_rx chain: stage "rf_bpf_eq" with filter_len=3
 *     → rf_frontend chain: stage "rf_bpf" with filter_len=5
 *     (Represents an RF Band-Pass Filter, modeled with different lengths)
 *
 *   "LNA 1" (normalized: "lna1")
 *     → baseband_rx chain: stage "lna" with filter_len=1 (pass-through)
 *     → rf_frontend chain: stage "lna" with filter_len=1
 *     (Low-Noise Amplifier — appears in both chains)
 *
 *   "Mixer 1" (normalized: "mixer1")
 *     → baseband_rx chain: stage "mixer_downconv" with filter_len=1
 *     (Frequency downconverter — only in the baseband analytical chain;
 *      in the RF path, mixing is handled explicitly by mix_down_and_lowpass())
 *
 *   "Filter 2" (normalized: "filter2")
 *     → baseband_rx chain: stage "bb_lpf" with filter_len=5
 *     → rf_postmix_bb chain: stage "bb_lpf" with filter_len=5
 *     (Baseband Low-Pass Filter — applied after downconversion)
 *
 *   "LNA 2" or "LNA 3" (normalized: "lna2" or "lna3")
 *     → baseband_rx chain: stage "bb_amp_1vpp" with auto_gain_to_vpp=1, target=1.0V
 *     → rf_postmix_bb chain: stage "bb_amp_1vpp" with auto_gain_to_vpp=1, target=1.0V
 *     (Baseband amplifier with automatic gain control to maintain 1V peak-to-peak)
 *
 * Unknown components:
 *   If a component name doesn't match any of the above, it is silently ignored
 *   (returns 0 = success). This is intentional: legacy CSV files may contain
 *   rows we don't care about, and the chain-completeness check at the end of
 *   stage_models_load_csv() will catch any genuinely missing stages.
 *
 * Parameters:
 *   cfg        — The configuration to populate
 *   component  — The raw component name from the CSV (e.g., "Filter 1")
 *   gain_db    — Gain value from the CSV
 *   nf_db      — Noise Figure value from the CSV
 *   errbuf     — Error buffer for diagnostics
 *   errbuf_size — Size of error buffer
 *
 * Returns:
 *   0  on success (component mapped or safely ignored)
 *   Negative on error (memory allocation failure in set_stage)
 */
static int map_legacy_component(
    StageModelsConfig* cfg,
    const char* component,
    double gain_db,
    double nf_db,
    char* errbuf,
    size_t errbuf_size) {

    char key[64];   /* Normalized component name */
    int rc;         /* Return code from set_stage */

    normalize_token(component, key, sizeof(key));

    /* --- "Filter 1" → RF Band-Pass Filter --- */
    if (strcmp(key, "filter1") == 0) {
        rc = set_stage(cfg, STAGE_CHAIN_BASEBAND_RX, "rf_bpf_eq", gain_db, nf_db, 3, 0, 0.0, errbuf, errbuf_size);
        if (rc != 0) return rc;
        return set_stage(cfg, STAGE_CHAIN_RF_FRONTEND, "rf_bpf", gain_db, nf_db, 5, 0, 0.0, errbuf, errbuf_size);
    }

    /* --- "LNA 1" → Low-Noise Amplifier --- */
    if (strcmp(key, "lna1") == 0) {
        rc = set_stage(cfg, STAGE_CHAIN_BASEBAND_RX, "lna", gain_db, nf_db, 1, 0, 0.0, errbuf, errbuf_size);
        if (rc != 0) return rc;
        return set_stage(cfg, STAGE_CHAIN_RF_FRONTEND, "lna", gain_db, nf_db, 1, 0, 0.0, errbuf, errbuf_size);
    }

    /* --- "Mixer 1" → Mixer Downconverter (baseband chain only) --- */
    if (strcmp(key, "mixer1") == 0) {
        return set_stage(cfg, STAGE_CHAIN_BASEBAND_RX, "mixer_downconv", gain_db, nf_db, 1, 0, 0.0, errbuf, errbuf_size);
    }

    /* --- "Filter 2" → Baseband Low-Pass Filter --- */
    if (strcmp(key, "filter2") == 0) {
        rc = set_stage(cfg, STAGE_CHAIN_BASEBAND_RX, "bb_lpf", gain_db, nf_db, 5, 0, 0.0, errbuf, errbuf_size);
        if (rc != 0) return rc;
        return set_stage(cfg, STAGE_CHAIN_RF_POSTMIX_BB, "bb_lpf", gain_db, nf_db, 5, 0, 0.0, errbuf, errbuf_size);
    }

    /* --- "LNA 2" or "LNA 3" → Baseband Amplifier with auto-gain (target 1 Vpp) --- */
    if (strcmp(key, "lna2") == 0 || strcmp(key, "lna3") == 0) {
        rc = set_stage(cfg, STAGE_CHAIN_BASEBAND_RX, "bb_amp_1vpp", gain_db, nf_db, 1, 1, 1.0, errbuf, errbuf_size);
        if (rc != 0) return rc;
        return set_stage(cfg, STAGE_CHAIN_RF_POSTMIX_BB, "bb_amp_1vpp", gain_db, nf_db, 1, 1, 1.0, errbuf, errbuf_size);
    }

    /*
     * If the component name doesn't match anything above, we silently
     * skip it. This is safe because:
     * 1. The CSV may contain comment rows or future components
     * 2. The completeness check at the end of stage_models_load_csv()
     *    will catch any critically missing stages
     */
    return 0;
}


/*
 * stage_models_load_csv — PUBLIC API: Load receiver stage configuration from a CSV file
 *
 * This is the main entry point for loading stage definitions. It performs:
 *
 * 1. ARGUMENT VALIDATION
 *    Checks that the CSV path and output config pointer are not NULL.
 *
 * 2. FILE OPENING
 *    Opens the CSV file for reading. Returns error -2 if the file doesn't exist.
 *
 * 3. FORMAT AUTO-DETECTION (First non-blank, non-comment line = header)
 *    The header row is parsed to determine:
 *    - Column indices for each known field (chain, name, gain_db, nf_db, etc.)
 *    - Whether this is "canonical" format (has a "chain" column) or
 *      "legacy" format (no "chain" column — uses component→chain mapping)
 *
 * 4. ROW-BY-ROW PARSING
 *    Each subsequent line is parsed according to the detected format:
 *    - Blank lines and lines starting with '#' are skipped
 *    - If an "enabled" column exists and the row's value is 0, it's skipped
 *    - In canonical mode: chain + name + gain + NF + optional fields are read
 *    - In legacy mode: component + gain + NF are read and mapped via map_legacy_component()
 *
 * 5. VALIDATION
 *    After all rows are processed:
 *    - Checks that the CSV had a header (errors if file was empty)
 *    - Checks that all 3 chains have at least one stage (errors if any chain is empty)
 *
 * 6. OUTPUT
 *    On success: frees any previous content in out_cfg, then assigns the newly loaded config.
 *    On failure: out_cfg is left unchanged (atomicity guarantee).
 *
 * Parameters:
 *   csv_path    — Path to the CSV file (absolute or relative)
 *   out_cfg     — Output configuration struct. Must be zero-initialized or previously freed.
 *   errbuf      — Buffer for a human-readable error message on failure
 *   errbuf_size — Size of errbuf in bytes
 *
 * Return codes:
 *    0  = Success
 *   -1  = NULL arguments
 *   -2  = File cannot be opened
 *   -3  = Header missing required columns (name/gain/nf)
 *   -4  = Legacy component mapping failed
 *   -5  = Canonical row has too few fields
 *   -6  = Unrecognized chain name in canonical row
 *   -7  = Invalid numeric value (gain or NF not a valid number)
 *   -8  = Memory allocation failed while adding a stage
 *   -9  = File contains no header row (completely empty/blank)
 *   -10 = One or more chains have zero stages after loading
 */
int stage_models_load_csv(const char* csv_path, StageModelsConfig* out_cfg, char* errbuf, size_t errbuf_size) {
    FILE* f;                         /* File handle for the CSV */
    char line[MAX_LINE_LEN];         /* Buffer for reading one line at a time */
    unsigned int line_no = 0u;       /* Current line number (for error messages) */
    int header_done = 0;              /* 1 once the header row has been parsed */
    int legacy_mode = 0;              /* 1 if no "chain" column was found (legacy format) */

    /* Column indices (set to -1 = "not found" initially) */
    int idx_chain = -1;               /* Index of the "chain" column */
    int idx_name = -1;                /* Index of the "name"/"stage"/"component" column */
    int idx_gain = -1;                /* Index of the "gain_db" column */
    int idx_nf = -1;                  /* Index of the "nf_db" column */
    int idx_filter_len = -1;          /* Index of the "filter_len" column (optional) */
    int idx_auto = -1;                /* Index of the "auto_gain_to_vpp" column (optional) */
    int idx_target = -1;              /* Index of the "target_vpp" column (optional) */
    int idx_enabled = -1;             /* Index of the "enabled" column (optional) */

    StageModelsConfig cfg;            /* Temporary config — only assigned to out_cfg on success */

    clear_error(errbuf, errbuf_size);

    /* --- Step 1: Validate arguments --- */
    if (!csv_path || !out_cfg) {
        set_error(errbuf, errbuf_size, "Invalid stage-models input arguments");
        return -1;
    }

    /* Zero-initialize the temporary config (all chains empty) */
    memset(&cfg, 0, sizeof(cfg));

    /* --- Step 2: Open the CSV file --- */
    f = fopen(csv_path, "r");
    if (!f) {
        set_error(errbuf, errbuf_size, "Could not open CSV file: %s", csv_path);
        return -2;
    }

    /* --- Step 3 & 4: Read and parse each line --- */
    while (fgets(line, (int)sizeof(line), f) != NULL) {
        char* fields[MAX_FIELDS];    /* Array of pointers to field starts */
        size_t count;                 /* Number of fields in this line */
        size_t i;
        char* p;

        ++line_no;

        /* Strip trailing newline/carriage-return characters */
        line[strcspn(line, "\r\n")] = '\0';

        /* Trim whitespace and skip blank lines or comment lines (starting with #) */
        p = trim_ws(line);
        if (*p == '\0' || *p == '#') {
            continue;
        }

        /* Split the line into comma-separated fields */
        count = split_csv(p, fields, MAX_FIELDS);

        /* Trim whitespace from each individual field */
        for (i = 0u; i < count; ++i) {
            fields[i] = trim_ws(fields[i]);
        }

        /* --- HEADER PARSING (first non-blank, non-comment line) --- */
        if (!header_done) {
            /*
             * Scan each field in the header to determine which column index
             * corresponds to which data field. We normalize each field name
             * so that "Gain_dB", "gain-db", "GAINDB" all match the same key.
             */
            for (i = 0u; i < count; ++i) {
                char norm[64];
                normalize_token(fields[i], norm, sizeof(norm));

                if (strcmp(norm, "chain") == 0 || strcmp(norm, "stagechain") == 0) {
                    idx_chain = (int)i;
                } else if (strcmp(norm, "name") == 0 || strcmp(norm, "stage") == 0 || strcmp(norm, "component") == 0) {
                    idx_name = (int)i;
                } else if (strcmp(norm, "gaindb") == 0 || strcmp(norm, "gainlossdb") == 0) {
                    idx_gain = (int)i;
                } else if (strcmp(norm, "nfdb") == 0 || strcmp(norm, "noisefiguredb") == 0) {
                    idx_nf = (int)i;
                } else if (strcmp(norm, "filterlen") == 0) {
                    idx_filter_len = (int)i;
                } else if (strcmp(norm, "autogaintovpp") == 0) {
                    idx_auto = (int)i;
                } else if (strcmp(norm, "targetvpp") == 0) {
                    idx_target = (int)i;
                } else if (strcmp(norm, "enabled") == 0) {
                    idx_enabled = (int)i;
                }
            }

            /* The CSV MUST have at least name, gain, and NF columns */
            if (idx_name < 0 || idx_gain < 0 || idx_nf < 0) {
                set_error(errbuf, errbuf_size, "CSV header must include stage/component, gain and noise figure columns");
                stage_models_free(&cfg);
                fclose(f);
                return -3;
            }

            /*
             * Detect format:
             * - If there's a "chain" column → canonical format
             * - If there's no "chain" column → legacy format (component names mapped to chains)
             */
            legacy_mode = (idx_chain < 0) ? 1 : 0;
            header_done = 1;
            continue;   /* Don't try to parse the header as data */
        }

        /* --- ENABLED CHECK (skip disabled rows) --- */
        if (idx_enabled >= 0 && idx_enabled < (int)count) {
            int enabled = 1;
            if (parse_int_value(fields[idx_enabled], &enabled) == 0 && enabled == 0) {
                continue;   /* Row is explicitly disabled — skip it */
            }
        }

        /* --- DATA ROW PARSING --- */
        if (legacy_mode) {
            /* LEGACY FORMAT: component, gain_db, nf_db */
            double gain_db;
            double nf_db;
            int rc;

            /* Skip rows where required columns are missing */
            if (idx_name >= (int)count || idx_gain >= (int)count || idx_nf >= (int)count) {
                continue;
            }

            /* Parse gain and NF values; skip row if parsing fails */
            if (parse_double_value(fields[idx_gain], &gain_db) != 0 || parse_double_value(fields[idx_nf], &nf_db) != 0) {
                continue;
            }

            /* Map the legacy component name to canonical chain/stage definitions */
            rc = map_legacy_component(&cfg, fields[idx_name], gain_db, nf_db, errbuf, errbuf_size);
            if (rc != 0) {
                set_error(errbuf, errbuf_size, "Legacy mapping failed at line %u: %s", line_no, fields[idx_name]);
                stage_models_free(&cfg);
                fclose(f);
                return -4;
            }
        } else {
            /* CANONICAL FORMAT: chain, name, gain_db, nf_db, [filter_len], [auto], [target], [enabled] */
            StageChainId chain_id;
            double gain_db;
            double nf_db;
            int filter_len = 1;         /* Default: no filtering */
            int auto_gain = 0;          /* Default: no auto-gain */
            double target_vpp = 0.0;    /* Default: no target voltage */

            /* Ensure all required columns are present in this row */
            if (idx_chain >= (int)count || idx_name >= (int)count || idx_gain >= (int)count || idx_nf >= (int)count) {
                set_error(errbuf, errbuf_size, "Malformed CSV row at line %u", line_no);
                stage_models_free(&cfg);
                fclose(f);
                return -5;
            }

            /* Parse the chain name (e.g., "baseband_rx") */
            if (parse_chain_id(fields[idx_chain], &chain_id) != 0) {
                set_error(errbuf, errbuf_size, "Unknown chain '%s' at line %u", fields[idx_chain], line_no);
                stage_models_free(&cfg);
                fclose(f);
                return -6;
            }

            /* Parse gain and NF numeric values */
            if (parse_double_value(fields[idx_gain], &gain_db) != 0 || parse_double_value(fields[idx_nf], &nf_db) != 0) {
                set_error(errbuf, errbuf_size, "Invalid gain/nf value at line %u", line_no);
                stage_models_free(&cfg);
                fclose(f);
                return -7;
            }

            /* Parse optional columns (only if present) */
            if (idx_filter_len >= 0 && idx_filter_len < (int)count) {
                (void)parse_int_value(fields[idx_filter_len], &filter_len);
            }
            if (idx_auto >= 0 && idx_auto < (int)count) {
                (void)parse_int_value(fields[idx_auto], &auto_gain);
            }
            if (idx_target >= 0 && idx_target < (int)count) {
                (void)parse_double_value(fields[idx_target], &target_vpp);
            }

            /* Add the stage to the appropriate chain */
            if (set_stage(&cfg, chain_id, fields[idx_name], gain_db, nf_db, filter_len, auto_gain, target_vpp, errbuf, errbuf_size) != 0) {
                set_error(errbuf, errbuf_size, "Failed to add stage at line %u", line_no);
                stage_models_free(&cfg);
                fclose(f);
                return -8;
            }
        }
    }

    fclose(f);   /* Done reading the file */

    /* --- Step 5: Post-load validation --- */

    /* Check that we actually found and parsed a header row */
    if (!header_done) {
        set_error(errbuf, errbuf_size, "CSV file has no header");
        stage_models_free(&cfg);
        return -9;
    }

    /*
     * Check that ALL THREE chains have at least one stage.
     * A receiver with a missing chain is physically impossible and would
     * cause the simulator to produce meaningless results.
     */
    if (cfg.counts[STAGE_CHAIN_BASEBAND_RX] == 0u || cfg.counts[STAGE_CHAIN_RF_FRONTEND] == 0u || cfg.counts[STAGE_CHAIN_RF_POSTMIX_BB] == 0u) {
        set_error(errbuf, errbuf_size, "CSV must define all chains: baseband_rx, rf_frontend, rf_postmix_bb");
        stage_models_free(&cfg);
        return -10;
    }

    /* --- Step 6: Commit the result --- */

    /*
     * Free any previous content in out_cfg (in case the caller didn't
     * free it themselves), then copy our temporary config into it.
     * This provides atomic update semantics: out_cfg is only modified
     * when the entire load succeeds.
     */
    stage_models_free(out_cfg);
    *out_cfg = cfg;
    return 0;   /* Success! */
}


/* ============================================================================
 * PUBLIC QUERY API
 * ============================================================================ */

/*
 * stage_models_get — Retrieve the stage array for a given receiver chain
 *
 * What it does:
 *   Returns a pointer to the internal array of StageModel structs for the
 *   specified chain ID, along with the number of stages in that chain.
 *   This is the primary way that main.c accesses the loaded stage data.
 *
 * Parameters:
 *   cfg        — A loaded StageModelsConfig (from stage_models_load_csv)
 *   id         — Which chain to retrieve:
 *                  STAGE_CHAIN_BASEBAND_RX (0) — complex baseband analytical path
 *                  STAGE_CHAIN_RF_FRONTEND (1) — real RF path before downconversion
 *                  STAGE_CHAIN_RF_POSTMIX_BB (2) — complex BB after downconversion
 *   out_models — Pointer that will be set to the internal stage array.
 *                DO NOT free this pointer — it's owned by the config.
 *
 * Returns:
 *   Number of stages in the chain (0 if error or empty chain).
 *   *out_models is set to the array pointer, or NULL on error.
 */
size_t stage_models_get(const StageModelsConfig* cfg, StageChainId id, const StageModel** out_models) {
    /* Validate all arguments */
    if (!cfg || !out_models || id < 0 || id >= STAGE_CHAIN_COUNT) {
        if (out_models) {
            *out_models = NULL;
        }
        return 0u;
    }

    *out_models = cfg->chains[id];   /* Set the output pointer to the internal array */
    return cfg->counts[id];           /* Return the number of stages */
}


/*
 * stage_chain_name — Get a human-readable name string for a chain ID
 *
 * What it does:
 *   Converts a numeric StageChainId enum value into a descriptive string.
 *   Used in status messages, CSV headers, and debugging output.
 *
 * Parameters:
 *   id — Chain identifier (0, 1, or 2)
 *
 * Returns:
 *   A pointer to a static string literal. Do NOT free this pointer.
 *   Returns "unknown" for invalid IDs.
 */
const char* stage_chain_name(StageChainId id) {
    switch (id) {
        case STAGE_CHAIN_BASEBAND_RX:
            return "baseband_rx";        /* Chain 0: complex baseband analytical path */
        case STAGE_CHAIN_RF_FRONTEND:
            return "rf_frontend";        /* Chain 1: real RF path before downconversion */
        case STAGE_CHAIN_RF_POSTMIX_BB:
            return "rf_postmix_bb";      /* Chain 2: complex BB after downconversion */
        default:
            return "unknown";            /* Invalid chain ID */
    }
}
