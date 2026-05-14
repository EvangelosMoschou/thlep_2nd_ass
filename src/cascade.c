/*
 * cascade.c — RF Receiver Cascade Analysis (Friis, IP3, Dynamic Range)
 *
 * Translates CascadeAnalyzer.m into C.  Computes system power requirements,
 * Friis noise cascade, IIP3 cascade, dynamic range (LDR, SFDR), and receiver
 * sensitivity.  Runs on the CSV-loaded stage chain with datasheet-correct
 * P1dB/IIP3 values from the component catalog when available.
 *
 * All functions are reentrant (no global state).
 */

#include "cascade.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "math_utils.h"
#include "physics.h"

#define K_BOLTZ     1.380649e-23   /* Boltzmann constant [J/K] — same as physics.h */

/* Large IIP3/P1dB value for stages where nonlinearity is not specified */
#define INFTY_DBM 100.0

/*
 * Stage-position to catalog-UID mapping for the baseband_rx chain.
 * This maps each of the 11 stages (positions 0-10) to its entry in
 * data/component_catalog.csv.  NULL means the stage has no catalog entry
 * (generic filter, limiter, etc.) and keeps the runtime CSV value.
 */
static const char *STAGE_UID_MAP[11] = {
    "SPST_SWITCH_01",   /* 0:  bb_00_switch */
    "FILTER_1_01",      /* 1:  bb_01_preselector_bpf */
    "LNA_1_01",         /* 2:  bb_02_lna1 */
    NULL,               /* 3:  bb_03_image_rejection_filter (generic) */
    "MIXER_1_01",       /* 4:  bb_04_mixer1 */
    "FILTER_2_01",      /* 5:  bb_05_bpf2 */
    "LNA_2_01",         /* 6:  bb_06_lna2 */
    "MIXER_2_01",       /* 7:  bb_07_mixer2 */
    "FILTER_3_01",      /* 8:  bb_08_bpf3 */
    "LNA_3_01",         /* 9:  bb_09_lna3 */
    NULL,               /* 10: bb_10_rlm43_5w (limiter, no catalog entry) */
};

/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================ */

/* Return dbm if finite and nonzero, else INFTY_DBM */
static double safe_iip3(double ip3_dbm) {
    if (isinf(ip3_dbm) || ip3_dbm == 0.0 || ip3_dbm > INFTY_DBM)
        return INFTY_DBM;
    if (ip3_dbm < -200.0)
        return INFTY_DBM;
    return ip3_dbm;
}

/* Clamp NF to minimum 1.0 in linear (0 dB) */
static double safe_nf_lin(double nf_db) {
    double f = db_to_lin_power(nf_db);
    return (f < 1.0) ? 1.0 : f;
}

/*
 * Look up the correct IIP3 and P1dB from the component catalog for a given
 * stage position in the baseband_rx chain.  If the catalog has an entry,
 * compute IIP3 from OIP3 (catalog gives OIP3, so IIP3 = OIP3 - gain).
 * Falls back to the stage_model values if no catalog entry is found.
 */
static void catalog_iip3_p1db(const ComponentCatalog *catalog,
                               int stage_pos, double gain_db,
                               double *iip3_dbm, double *p1db_dbm) {
    const CatalogEntry *ce = NULL;
    double cat_iip3, cat_p1db;

    if (!catalog || stage_pos < 0 ||
        (size_t)stage_pos >= sizeof(STAGE_UID_MAP)/sizeof(STAGE_UID_MAP[0]))
        return;

    if (!STAGE_UID_MAP[stage_pos])
        return; /* No catalog mapping for this position */

    ce = component_catalog_find(catalog, STAGE_UID_MAP[stage_pos]);
    if (!ce) return;

    /* Determine IIP3 from catalog */
    if (!isnan(ce->iip3_dbm)) {
        /* Catalog gives IIP3 directly */
        cat_iip3 = ce->iip3_dbm;
    } else if (!isnan(ce->oip3_dbm)) {
        /* Catalog gives OIP3 → IIP3 = OIP3 − gain */
        cat_iip3 = ce->oip3_dbm - gain_db;
    } else {
        cat_iip3 = *iip3_dbm; /* keep original */
    }

    /* Determine input-referred P1dB from catalog */
    if (!isnan(ce->p1db_dbm)) {
        /* Catalog P1dB is output-referred → IP1dB = P1dB − gain */
        cat_p1db = ce->p1db_dbm - gain_db;
    } else {
        cat_p1db = *p1db_dbm; /* keep original */
    }

    *iip3_dbm = cat_iip3;
    *p1db_dbm = cat_p1db;
}

/* ============================================================================
 * compute_cascade — Main cascade analysis
 * ============================================================================ */

int compute_cascade(const StageModelsConfig *stage_cfg,
                    const CascadeParams *params,
                    int chain_id,
                    const ComponentCatalog *catalog,
                    CascadeResult *result) {
    const StageModel *stages = NULL;
    size_t nstages;
    size_t i;
    int valid_stages;

    double Ni_W, Ni_dBm, Si_dBm;
    double Vrms, Pout_W, Pout_dBm;
    double k_B = K_BOLTZ;

    double g_lin, f_lin, iip3_mW;
    double F_total, IIP3_inv, G_roll;
    double F_lin, Te, G_lin, No_W, N_out_dBm;
    double OIP3, InP1dB, OutP1dB;
    double Pout_SFDR_max;
    double noise_floor_W, noise_floor_dBm;
    double sensitivity_dBm;

    if (!stage_cfg || !params || !result)
        return -1;

    memset(result, 0, sizeof(*result));

    stages = NULL;
    nstages = 0;
    nstages = stage_models_get(stage_cfg, chain_id, &stages);

    if (!stages || nstages == 0u)
        return -1;

    /* Clamp stage count to our array size */
    valid_stages = (int)nstages;
    if (valid_stages > 32) valid_stages = 32;
    result->num_stages = valid_stages;
    result->stage_count = valid_stages;

    /* -----------------------------------------------------------------------
     * 1. SYSTEM POWER REQUIREMENTS
     *    (CascadeAnalyzer.m §2)
     * ----------------------------------------------------------------------- */

    /* Input noise power at the antenna */
    Ni_W = k_B * params->antenna_temp_k * params->bw_hz;
    Ni_dBm = lin_to_db(Ni_W) + 30.0;

    /* Input signal power for the target SNR */
    Si_dBm = Ni_dBm + params->snr_target_db;

    /* Output power for 1 Vpp into 50 Ω (sine-wave equivalent) */
    Vrms = params->vpp_out / (2.0 * sqrt(2.0));
    Pout_W = (Vrms * Vrms) / R_LOAD_OHM;
    Pout_dBm = lin_to_db(Pout_W) + 30.0;

    result->ni_dbm = Ni_dBm;
    result->si_dbm = Si_dBm;
    result->pout_dbm = Pout_dBm;
    result->total_required_gain_db = Pout_dBm - Si_dBm;

    /* -----------------------------------------------------------------------
     * 2. FRIIS NOISE CASCADE + IIP3 CASCADE
     *    (CascadeAnalyzer.m §3)
     * ----------------------------------------------------------------------- */

    /* Seed from first stage */
    {
        const StageModel *s = &stages[0];
        double cat_iip3 = safe_iip3(s->ip3_dbm);
        double cat_p1db_dummy = safe_iip3(s->p1db_dbm);

        catalog_iip3_p1db(catalog, 0, s->gain_db, &cat_iip3, &cat_p1db_dummy);

        g_lin = db_to_lin_power(s->gain_db);
        f_lin = safe_nf_lin(s->nf_db);
        iip3_mW = pow(10.0, cat_iip3 / 10.0);

        result->stages[0].name = s->name;
        result->stages[0].gain_db = s->gain_db;
        result->stages[0].nf_db = s->nf_db;
        result->stages[0].iip3_dbm = cat_iip3;
        result->stages[0].cum_gain_db = s->gain_db;
        result->stages[0].cum_nf_db = s->nf_db;
        result->stages[0].cum_iip3_dbm = cat_iip3;

        F_total = f_lin;
        IIP3_inv = 1.0 / iip3_mW;
        G_roll = g_lin;
    }

    /* Cascade through remaining stages */
    for (i = 1; i < (size_t)valid_stages; ++i) {
        const StageModel *s = &stages[i];
        double cat_iip3 = safe_iip3(s->ip3_dbm);
        double cat_p1db = safe_iip3(s->p1db_dbm);

        /* Override with datasheet-correct catalog values */
        catalog_iip3_p1db(catalog, (int)i, s->gain_db, &cat_iip3, &cat_p1db);

        g_lin = db_to_lin_power(s->gain_db);
        f_lin = safe_nf_lin(s->nf_db);
        iip3_mW = pow(10.0, cat_iip3 / 10.0);

        /* Friis: F_total = F_prev + (F_i - 1) / G_prev */
        F_total = F_total + (f_lin - 1.0) / G_roll;

        /* IIP3 cascade: 1/IIP3 = 1/IIP3_prev + G_prev / IIP3_i */
        IIP3_inv = IIP3_inv + G_roll / iip3_mW;

        G_roll *= g_lin;

        /* Store per-stage cumulative values */
        result->stages[i].name = s->name;
        result->stages[i].gain_db = s->gain_db;
        result->stages[i].nf_db = s->nf_db;
        result->stages[i].iip3_dbm = cat_iip3;

        /* Cumulative gain in dB: sum of gains (log domain) */
        {
            double cum_g = 0.0;
            size_t j;
            for (j = 0; j <= i; ++j)
                cum_g += stages[j].gain_db;
            result->stages[i].cum_gain_db = cum_g;
        }

        result->stages[i].cum_nf_db = lin_to_db(F_total);
        result->stages[i].cum_iip3_dbm = 10.0 * log10(1.0 / IIP3_inv);
    }

    /* Extract cascade totals from the last stage */
    {
        int last = valid_stages - 1;
        result->total_gain_db  = result->stages[last].cum_gain_db;
        result->total_nf_db    = result->stages[last].cum_nf_db;
        result->total_iip3_dbm = result->stages[last].cum_iip3_dbm;
    }

    /* -----------------------------------------------------------------------
     * 3. DYNAMIC RANGE
     *    (CascadeAnalyzer.m §4)
     * ----------------------------------------------------------------------- */

    F_lin = db_to_lin_power(result->total_nf_db);
    Te = (F_lin - 1.0) * params->t0_k;
    G_lin = db_to_lin_power(result->total_gain_db);

    /* Output noise: No = k * (T_A + Te) * B * G */
    No_W = k_B * (params->antenna_temp_k + Te) * params->bw_hz * G_lin;
    N_out_dBm = lin_to_db(No_W) + 30.0;

    OIP3 = result->total_iip3_dbm + result->total_gain_db;
    InP1dB = result->total_iip3_dbm - 9.6;         /* standard approx */
    OutP1dB = InP1dB + result->total_gain_db - 1.0; /* 1 dB compression */

    /* SFDR: Pout where IM3 = noise floor */
    Pout_SFDR_max = N_out_dBm + (2.0 / 3.0) * (OIP3 - N_out_dBm);
    result->n_out_dbm  = N_out_dBm;
    result->oip3_dbm   = OIP3;
    result->input_p1db_dbm  = InP1dB;
    result->output_p1db_dbm = OutP1dB;
    result->ldr_db     = OutP1dB - N_out_dBm;
    result->sfdr_db    = Pout_SFDR_max - N_out_dBm;

    /* -----------------------------------------------------------------------
     * 4. RECEIVER SENSITIVITY
     * ----------------------------------------------------------------------- */

    /*
     * Standard sensitivity:
     *   S_min(dBm) = kT_0B + NF_total + SNR_required
     *
     * where kT_0B is the thermal noise floor at T_0 = 290 K.
     * Equivalent: -174 dBm/Hz + 10·log₁₀(B) + NF + SNR_req
     */
    noise_floor_W = k_B * params->t0_k * params->bw_hz;
    noise_floor_dBm = lin_to_db(noise_floor_W) + 30.0;

    sensitivity_dBm = noise_floor_dBm
                    + result->total_nf_db
                    + params->snr_required_db;

    result->noise_floor_dbm  = noise_floor_dBm;
    result->sensitivity_dbm   = sensitivity_dBm;

    return 0;
}

/* ============================================================================
 * print_cascade — Formatted output
 * ============================================================================ */

void print_cascade(const CascadeResult *r) {
    int i;

    if (!r) return;

    printf("\n");
    printf("================================================================================\n");
    printf("  CASCADE ANALYSIS — Part E (Receiver Front-End)\n");
    printf("================================================================================\n");

    /* --- Per-stage cascade table --- */
    printf("\n  %-3s  %-28s  %8s  %8s  %8s  %8s  %8s  %8s\n",
           "  #", "Stage", "Gain", "NF", "IIP3",
           "Cum G", "Cum NF", "Cum IIP3");
    printf("  ---  %-28s  %8s  %8s  %8s  %8s  %8s  %8s\n",
           "----", "------", "----", "------",
           "------", "------", "--------");
    for (i = 0; i < r->stage_count; ++i) {
        const StageCascadeEntry *e = &r->stages[i];
        printf("  %-3d  %-28s  %7.2f  %7.2f  %7.1f  %7.2f  %7.2f  %7.1f\n",
               i + 1,
               e->name ? e->name : "?",
               e->gain_db,
               e->nf_db,
               e->iip3_dbm,
               e->cum_gain_db,
               e->cum_nf_db,
               e->cum_iip3_dbm);
    }

    /* --- Cascade summary --- */
    printf("\n  --- Cascade Summary ---\n");
    printf("  Total Gain:          %9.2f  dB\n",  r->total_gain_db);
    printf("  Total Noise Figure:  %9.2f  dB\n",  r->total_nf_db);
    printf("  Total IIP3:          %9.2f  dBm\n", r->total_iip3_dbm);

    /* --- System power requirements --- */
    printf("\n  --- System Power Requirements ---\n");
    printf("  Input noise (Ni, T_ant=%.0f K):     %8.2f  dBm\n",
           (double)(150), r->ni_dbm);
    printf("  Input signal (Si, SNR=%.1f dB):     %8.2f  dBm\n",
           r->si_dbm - r->ni_dbm, r->si_dbm);
    printf("  Target output (%.2f Vpp, 50 Ω):     %8.2f  dBm\n",
           1.0, r->pout_dbm);
    printf("  Required gain:            %8.2f  dB\n",
           r->total_required_gain_db);

    /* --- Dynamic range --- */
    printf("\n  --- Dynamic Range ---\n");
    printf("  Output noise (No):      %9.2f  dBm\n", r->n_out_dbm);
    printf("  Output P1dB:            %9.2f  dBm\n", r->output_p1db_dbm);
    printf("  OIP3:                   %9.2f  dBm\n", r->oip3_dbm);
    printf("  LDR (P1dB − No):        %9.2f  dB\n",  r->ldr_db);
    printf("  SFDR:                   %9.2f  dB\n",  r->sfdr_db);

    /* --- Sensitivity --- */
    printf("\n  --- Receiver Sensitivity ---\n");
    printf("  Noise floor (kT₀B):     %9.2f  dBm\n", r->noise_floor_dbm);
    printf("  + NF total:             %9.2f  dB\n",  r->total_nf_db);
    printf("  + SNR req (64-APSK):    %9.2f  dB\n",
           r->sensitivity_dbm - r->noise_floor_dbm - r->total_nf_db);
    printf("  ───────────────────────────────────────────────────\n");
    printf("  RECEIVER SENSITIVITY:    %9.2f  dBm\n", r->sensitivity_dbm);
    printf("================================================================================\n");
}
