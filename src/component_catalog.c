/*
 * component_catalog.c — Load data/component_catalog.csv (datasheet values)
 *
 * Parses the CSV and provides lookup by component UID for P1dB/OIP3/IIP3
 * values from manufacturer datasheets.
 */

#include "component_catalog.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 1024
#define MAX_FIELDS 20

/* Parse a double from a CSV field — return NAN for empty/invalid */
static double parse_double_field(const char *s) {
    char *end = NULL;
    double val;
    if (!s || s[0] == '\0') return NAN;
    /* Strip whitespace */
    while (*s && (unsigned char)*s <= ' ') s++;
    if (s[0] == '\0') return NAN;
    val = strtod(s, &end);
    if (end == s) return NAN; /* no digits consumed */
    return val;
}

/* Split a CSV line into fields (mutates the input buffer) */
static int split_csv(char *line, const char **fields, int max_fields) {
    int count = 0;
    char *p = line;
    while (*p && count < max_fields) {
        /* Skip leading whitespace */
        while (*p && (unsigned char)*p <= ' ') p++;
        if (*p == '\0') break;

        /* Handle quoted fields */
        if (*p == '"') {
            p++; /* skip opening quote */
            fields[count++] = p;
            while (*p && !(*p == '"' && *(p+1) == ',')) {
                if (*p == '"' && *(p+1) == '\0') { *p = '\0'; p++; break; }
                p++;
            }
            if (*p == '"') { *p = '\0'; p++; }
        } else {
            fields[count++] = p;
            while (*p && *p != ',') p++;
        }
        if (*p == ',') { *p++ = '\0'; }
    }
    /* Pad remaining fields as empty */
    while (count < max_fields) {
        fields[count++] = "";
    }
    return count;
}

/*
 * Normalise a header string: lowercase, strip non-alphanumeric.
 * Used for case-insensitive column-name matching.
 */
static void normalise_header(const char *src, char *dst, size_t dst_len) {
    size_t j = 0;
    if (!src || !dst || dst_len == 0) return;
    while (*src && j < dst_len - 1) {
        char c = (char)tolower((unsigned char)*src);
        if (isalnum((unsigned char)c)) dst[j++] = c;
        src++;
    }
    dst[j] = '\0';
}

int component_catalog_load(const char *path, ComponentCatalog *cat) {
    FILE *f;
    char line[MAX_LINE];
    int line_no = 0;
    int entry_count = 0;
    const char *fields[MAX_FIELDS];
    int nf;
    int i;
    /* Column indices — found from header */
    int col_uid      = -1;
    int col_name     = -1;
    int col_part     = -1;
    int col_gain     = -1;
    int col_nf       = -1;
    int col_p1db     = -1;
    int col_oip3     = -1;
    int col_iip3     = -1;
    int headers_seen = 0;

    if (!path || !cat) return -1;
    memset(cat, 0, sizeof(*cat));
    snprintf(cat->filepath, sizeof(cat->filepath), "%s", path);

    f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "Error: cannot open component catalog '%s'\n", path);
        return -1;
    }

    while (fgets(line, sizeof(line), f) && entry_count < CATALOG_MAX_ENTRIES) {
        line_no++;
        /* Strip trailing newline */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';
        if (len == 0) continue;

        nf = split_csv(line, fields, MAX_FIELDS);
        if (nf < 2) continue;

        /* ---- First row: find columns by header name ---- */
        if (!headers_seen) {
            char norm[64];
            for (i = 0; i < nf; i++) {
                normalise_header(fields[i], norm, sizeof(norm));
                if (strcmp(norm, "componentuid") == 0)          col_uid  = i;
                else if (strcmp(norm, "name") == 0)              col_name = i;
                else if (strcmp(norm, "partnumber") == 0)        col_part = i;
                else if (strcmp(norm, "gaindb") == 0)            col_gain = i;
                else if (strcmp(norm, "noisefigured") == 0 ||
                         strcmp(norm, "nfdb") == 0)              col_nf   = i;
                else if (strcmp(norm, "p1dbdbm") == 0 ||
                         strcmp(norm, "p1db") == 0)              col_p1db = i;
                else if (strcmp(norm, "oip3db") == 0 ||
                         strcmp(norm, "oip3") == 0)              col_oip3 = i;
                else if (strcmp(norm, "iip3db") == 0 ||
                         strcmp(norm, "iip3") == 0)              col_iip3 = i;
            }
            headers_seen = 1;
            continue; /* header row done — move to data */
        }

        /* ---- Data rows: only record entries that have a UID ---- */
        if (col_uid < 0) break; /* no uid column found — wrong format */
        if (fields[col_uid][0] == '\0') continue; /* skip rows without catalog entry */

        CatalogEntry *e = &cat->entries[entry_count];

        snprintf(e->uid,  sizeof(e->uid),  "%s", fields[col_uid]);
        snprintf(e->name, sizeof(e->name), "%s", col_name >= 0 ? fields[col_name] : "");
        snprintf(e->part, sizeof(e->part), "%s", col_part >= 0 ? fields[col_part] : "");

        e->gain_db  = col_gain >= 0 ? parse_double_field(fields[col_gain]) : NAN;
        e->nf_db    = col_nf   >= 0 ? parse_double_field(fields[col_nf])   : NAN;
        e->p1db_dbm = col_p1db >= 0 ? parse_double_field(fields[col_p1db]) : NAN;
        e->oip3_dbm = col_oip3 >= 0 ? parse_double_field(fields[col_oip3]) : NAN;
        e->iip3_dbm = col_iip3 >= 0 ? parse_double_field(fields[col_iip3]) : NAN;

        entry_count++;
    }

    fclose(f);
    cat->count = entry_count;
    return 0;
}

const CatalogEntry *component_catalog_find(const ComponentCatalog *cat,
                                           const char *uid) {
    int i;
    if (!cat || !uid) return NULL;
    for (i = 0; i < cat->count; i++) {
        if (strcmp(cat->entries[i].uid, uid) == 0)
            return &cat->entries[i];
    }
    return NULL;
}

/*
 * Name-substring to catalog-UID mapping table.
 * Each entry pairs a stage-name substring (lowercase) with a catalog UID.
 * The first match wins — order matters: put more specific patterns first.
 */
static const char *STAGE_KEYWORDS[][2] = {
    {"lna3",     "LNA_3_01"},
    {"lna2",     "LNA_2_01"},
    {"lna1",     "LNA_1_01"},
    {"lna",      "LNA_3_01"},     /* generic fallback: last LNA in chain */
    {"mixer2",   "MIXER_2_01"},
    {"mixer1",   "MIXER_1_01"},
    {"mix",      "MIXER_2_01"},   /* generic fallback */
    {"bpf3",     "FILTER_3_01"},
    {"bpf2",     "FILTER_2_01"},
    {"preselector", "FILTER_1_01"},
    {"bpf",      "FILTER_1_01"},  /* generic fallback: first BPF */
    {"switch",   "SPST_SWITCH_01"},
    {NULL, NULL}
};

void component_catalog_override_stage(StageModel *stg,
                                      const ComponentCatalog *cat) {
    const CatalogEntry *ce = NULL;
    int i;
    double cat_iip3, cat_p1db;
    char name_lower[128];
    size_t j;

    if (!stg || !cat || !stg->name) return;

    /* Convert stage name to lowercase for case-insensitive matching */
    name_lower[0] = '\0';
    for (j = 0; j < sizeof(name_lower) - 1 && stg->name[j]; j++)
        name_lower[j] = (char)tolower((unsigned char)stg->name[j]);
    name_lower[j] = '\0';

    /* Find first keyword match */
    for (i = 0; STAGE_KEYWORDS[i][0]; i++) {
        if (strstr(name_lower, STAGE_KEYWORDS[i][0])) {
            ce = component_catalog_find(cat, STAGE_KEYWORDS[i][1]);
            if (ce) break;
        }
    }
    if (!ce) return;

    /* Determine IIP3: prefer direct iip3, else OIP3 − gain */
    if (!isnan(ce->iip3_dbm))
        cat_iip3 = ce->iip3_dbm;
    else if (!isnan(ce->oip3_dbm))
        cat_iip3 = ce->oip3_dbm - stg->gain_db;
    else
        cat_iip3 = stg->ip3_dbm;

    /* Determine input-referred P1dB: P1dB_out − gain */
    if (!isnan(ce->p1db_dbm))
        cat_p1db = ce->p1db_dbm - stg->gain_db;
    else
        cat_p1db = stg->p1db_dbm;

    /* Override stage model (only if finite and more aggressive than current) */
    if (isfinite(cat_iip3) && cat_iip3 < stg->ip3_dbm)
        stg->ip3_dbm = cat_iip3;
    if (isfinite(cat_p1db) && cat_p1db < stg->p1db_dbm)
        stg->p1db_dbm = cat_p1db;
}
