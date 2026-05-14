/*
 * propagation.c — ITU-R Propagation Models for 24 GHz Satellite Link Budget
 *
 * Implements Part D of the assignment: FSPL, rain (P.838-3), fog (P.840-9),
 * gas (P.676-13), and link margin computation.
 *
 * All functions are reentrant (no global state).
 *
 * Standards:
 *   ITU-R P.838-3  (09/2005):  Specific attenuation model for rain
 *   ITU-R P.840-9  (08/2023):  Attenuation due to clouds and fog
 *   ITU-R P.676-13 (08/2022):  Attenuation by atmospheric gases
 */

#include "propagation.h"

#include <math.h>
#include <stdio.h>

#include "math_utils.h"

/* ============================================================================
 * CONSTANTS
 * ============================================================================ */

#define SPEED_OF_LIGHT 2.99792458e8     /* c [m/s] */
#define R_EARTH_KM     6371.0            /* Mean Earth radius [km] */
#define RAIN_HEIGHT_KM 4.0               /* Effective rain height [km] (P.839) */
#define O2_EQUIV_KM    6.0               /* Oxygen equivalent height [km] at 24 GHz */
#define H2O_EQUIV_KM   2.0               /* Water-vapour equivalent height [km] at 24 GHz */

/* Sea-level reference specific attenuations at 24 GHz (from P.676-13 curves) */
#define O2_REF_DBKM      0.0080          /* Oxygen γ_o @ 1013.25 hPa, 288.15 K */
#define H2O_REF_DBKM     0.0850          /* Water-vapour γ_w @ ρ=7.5 g/m³, 288.15 K */
#define REF_PRESSURE_HPA 1013.25
#define REF_TEMP_K       288.15
#define REF_RHO_GM3      7.5

/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================ */

/* Degrees → radians */
static double deg2rad(double deg) {
    return deg * (3.14159265358979323846 / 180.0);
}

/* Safe sin(θ) for elevation  (clamp to avoid divide-by-zero near horizon) */
static double sin_elev(double elev_deg) {
    double s = sin(deg2rad(elev_deg));
    return (s < 0.05) ? 0.05 : s;
}

/* ============================================================================
 * ITU-R P.838-3 COEFFICIENT TABLES
 *
 * The specific rain attenuation is γ_R = k × R^α  [dB/km]
 *
 * log10(k_H) and log10(k_V) use 4-term Gaussian-exponent sums + linear term.
 * α_H and α_V use 5-term sums + linear term.
 *
 * Tables 1-4 from the standard.
 * ============================================================================ */

/*
 * Compute log10(k) using the P.838-3 curve-fit with 4-term sum.
 * Arguments:
 *   coeff[4][3] — a_j, b_j, c_j for j=0..3
 *   m, c        — linear-term constants: m·log₁₀f + c
 *   lf          — log₁₀(frequency_in_GHz)
 */
static double rain_log10k(const double coeff[4][3], double m, double c,
                          double lf) {
    double sum = 0.0;
    int j;
    for (j = 0; j < 4; ++j) {
        double t = (lf - coeff[j][1]) / coeff[j][2];
        sum += coeff[j][0] * exp(-t * t);
    }
    return sum + m * lf + c;
}

/*
 * Compute α using the P.838-3 curve-fit with 5-term sum.
 * Arguments:
 *   coeff[5][3] — a_j, b_j, c_j for j=0..4
 *   m, c        — linear-term constants: m·log₁₀f + c
 *   lf          — log₁₀(frequency_in_GHz)
 */
static double rain_alpha(const double coeff[5][3], double m, double c,
                         double lf) {
    double sum = 0.0;
    int j;
    for (j = 0; j < 5; ++j) {
        double t = (lf - coeff[j][1]) / coeff[j][2];
        sum += coeff[j][0] * exp(-t * t);
    }
    return sum + m * lf + c;
}

/*
 * Table 1 — Coefficients for k_H
 *   j:  a_j         b_j         c_j
 *   1: -5.33980   -0.10008    1.13098
 *   2: -0.35351    1.26970    0.45400
 *   3: -0.23789    0.86036    0.15354
 *   4: -0.94158    0.64552    0.16817
 *   linear: m_k = -0.18961, c_k = 0.71147
 */
static const double K_H_COEFF[4][3] = {
    {-5.33980, -0.10008, 1.13098},
    {-0.35351,  1.26970, 0.45400},
    {-0.23789,  0.86036, 0.15354},
    {-0.94158,  0.64552, 0.16817}
};
#define K_H_M  (-0.18961)
#define K_H_C  ( 0.71147)

/*
 * Table 2 — Coefficients for k_V
 *   j:  a_j         b_j         c_j
 *   1: -3.80595    0.56934    0.81061
 *   2: -3.44965   -0.22911    0.51059
 *   3: -0.39902    0.73042    0.11899
 *   4:  0.50167    1.07319    0.27195
 *   linear: m_k = -0.16398, c_k = 0.63297
 */
static const double K_V_COEFF[4][3] = {
    {-3.80595,  0.56934, 0.81061},
    {-3.44965, -0.22911, 0.51059},
    {-0.39902,  0.73042, 0.11899},
    { 0.50167,  1.07319, 0.27195}
};
#define K_V_M  (-0.16398)
#define K_V_C  ( 0.63297)

/*
 * Table 3 — Coefficients for α_H
 *   j:  a_j         b_j         c_j
 *   1: -0.14318    1.82442   -0.55187
 *   2:  0.29591    0.77564    0.19822
 *   3:  0.32177    0.63773    0.13164
 *   4: -5.37610   -0.96230    1.47828
 *   5: 16.1721    -3.29980    3.43990
 *   linear: m_α = 0.67849, c_α = -1.95537
 */
static const double A_H_COEFF[5][3] = {
    {-0.14318,  1.82442, -0.55187},
    { 0.29591,  0.77564,  0.19822},
    { 0.32177,  0.63773,  0.13164},
    {-5.37610, -0.96230,  1.47828},
    {16.1721,  -3.29980,  3.43990}
};
#define A_H_M  ( 0.67849)
#define A_H_C  (-1.95537)

/*
 * Table 4 — Coefficients for α_V
 *   j:  a_j         b_j         c_j
 *   1: -0.07771    2.33840   -0.76284
 *   2:  0.56727    0.95545    0.54039
 *   3: -0.20238    1.14520    0.26809
 *   4: -48.2991    0.791669   0.116226
 *   5:  48.5833    0.791459   0.116479
 *   linear: m_α = -0.053739, c_α = 0.83433
 */
static const double A_V_COEFF[5][3] = {
    {-0.07771,   2.33840,   -0.76284},
    { 0.56727,   0.95545,    0.54039},
    {-0.20238,   1.14520,    0.26809},
    {-48.2991,   0.791669,   0.116226},
    { 48.5833,   0.791459,   0.116479}
};
#define A_V_M  (-0.053739)
#define A_V_C  ( 0.83433)

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/* --------------------------------------------------------------------------
 * 1. FREE-SPACE PATH LOSS
 * -------------------------------------------------------------------------- */

double compute_fspl(double freq_hz, double dist_km) {
    /*
     * FSPL = (4π × d × f / c)²
     * In dB: 20·log10(4π) + 20·log10(d) + 20·log10(f) − 20·log10(c)
     *
     * Practical form: FSPL_dB = 92.45 + 20·log10(f_GHz) + 20·log10(d_km)
     */
    double f_ghz = freq_hz / 1.0e9;
    if (f_ghz <= 0.0 || dist_km <= 0.0) return INFINITY;
    return 92.45 + 20.0 * log10(f_ghz) + 20.0 * log10(dist_km);
}

/* --------------------------------------------------------------------------
 * 2. RAIN ATTENUATION  (ITU-R P.838-3)
 * -------------------------------------------------------------------------- */

double rain_specific_attenuation(double f_ghz, double R_mmh,
                                 double elev_deg, double tau_deg) {
    double lf;               /* log₁₀(f) */
    double k_H, k_V, k;      /* coefficients */
    double a_H, a_V, a;      /* exponents */
    double cos2e, cos2t;     /* cos²(θ), cos²(τ) */

    if (f_ghz <= 0.0 || R_mmh <= 0.0) return 0.0;

    lf = log10(f_ghz);

    /* Compute k_H, k_V from 4-term sums (Tables 1, 2) */
    k_H = pow(10.0, rain_log10k(K_H_COEFF, K_H_M, K_H_C, lf));
    k_V = pow(10.0, rain_log10k(K_V_COEFF, K_V_M, K_V_C, lf));

    /* Compute α_H, α_V from 5-term sums (Tables 3, 4) */
    a_H = rain_alpha(A_H_COEFF, A_H_M, A_H_C, lf);
    a_V = rain_alpha(A_V_COEFF, A_V_M, A_V_C, lf);

    /* Combine for given elevation and polarisation (eqs. 4, 5) */
    cos2e = cos(deg2rad(elev_deg));
    cos2e = cos2e * cos2e;
    cos2t = cos(deg2rad(tau_deg));
    cos2t = cos2t * cos2t;

    k = (k_H + k_V + (k_H - k_V) * cos2e * cos2t) / 2.0;

    /* α requires a weighted average (eq. 5) */
    if (k > 0.0) {
        a = (k_H * a_H + k_V * a_V + (k_H * a_H - k_V * a_V) * cos2e * cos2t)
            / (2.0 * k);
    } else {
        a = (a_H + a_V) / 2.0;
    }

    /* Specific attenuation */
    return k * pow(R_mmh, a);
}

double rain_slant_attenuation(double gamma_dbkm, double elev_deg) {
    if (gamma_dbkm <= 0.0) return 0.0;
    if (elev_deg <= 0.0)   return INFINITY;
    return gamma_dbkm * RAIN_HEIGHT_KM / sin_elev(elev_deg);
}

/* --------------------------------------------------------------------------
 * 3. FOG / CLOUD ATTENUATION  (ITU-R P.840-9, double-Debye model)
 * -------------------------------------------------------------------------- */

double fog_kl_coefficient(double f_ghz, double T_k) {
    double eps0, eps1, eps2;
    double fp, fs;               /* relaxation frequencies [GHz] */
    double ep_prime, ep_prime2;  /* ε'(f), ε''(f) */
    double eta, kl;
    double ff;                   /* f/fp, f/fs */

    if (f_ghz <= 0.0 || T_k <= 0.0) return 0.0;

    /*
     * Double-Debye model for dielectric permittivity of liquid water
     * eqs. (6)–(10) from P.840-9.
     */
    eps0 = 77.66 + 103.3 * (300.0 / T_k - 1.0);
    eps1 = 0.0671 * eps0;
    eps2 = 3.52;

    fp = 20.20 - 146.0 * (300.0 / T_k - 1.0)
          + 316.0 * (300.0 / T_k - 1.0) * (300.0 / T_k - 1.0);
    fs = 39.8 * fp;

    /* Avoid division by zero */
    if (fp <= 0.0 || fs <= 0.0) return 0.0;

    ff = f_ghz / fp;
    ep_prime2 = f_ghz * (eps0 - eps1) / (fp * (1.0 + ff * ff))
              + f_ghz * (eps1 - eps2) / (fs * (1.0 + f_ghz * f_ghz / (fs * fs)));

    ep_prime = (eps0 - eps1) / (1.0 + ff * ff)
             + (eps1 - eps2) / (1.0 + f_ghz * f_ghz / (fs * fs))
             + eps2;

    eta = (2.0 + ep_prime) / ep_prime2;
    if (eta < 0.0) eta = 0.0;

    /* Eq. (2): K_l = 0.819·f / (ε''·(1+η²)) */
    kl = 0.819 * f_ghz / (ep_prime2 * (1.0 + eta * eta));

    return kl;   /* (dB/km)/(g/m³) */
}

double fog_slant_attenuation(double f_ghz, double T_k, double M_gm3,
                             double elev_deg) {
    double kl, gamma_c;

    if (M_gm3 <= 0.0) return 0.0;

    kl = fog_kl_coefficient(f_ghz, T_k);
    gamma_c = kl * M_gm3;   /* specific attenuation [dB/km] */
    if (gamma_c <= 0.0) return 0.0;

    return gamma_c / sin_elev(elev_deg);
}

/* --------------------------------------------------------------------------
 * 4. GASEOUS ATTENUATION  (ITU-R P.676-13, simplified reference model)
 * -------------------------------------------------------------------------- */

void gas_specific_attenuation(double f_ghz, double P_hpa, double T_k,
                              double rho_gm3,
                              double *o2_dbkm, double *h2o_dbkm) {
    double po;       /* pressure ratio P / P_ref */
    double tr;       /* temperature ratio T_ref / T */
    double rr;       /* humidity ratio ρ / ρ_ref */

    if (f_ghz <= 0.0) {
        if (o2_dbkm)  *o2_dbkm  = 0.0;
        if (h2o_dbkm) *h2o_dbkm = 0.0;
        return;
    }

    po = (P_hpa > 0.0) ? (P_hpa / REF_PRESSURE_HPA) : 1.0;
    tr = (T_k   > 0.0) ? (REF_TEMP_K / T_k)         : 1.0;
    rr = (rho_gm3 > 0.0) ? (rho_gm3 / REF_RHO_GM3) : 0.0;

    /*
     * Oxygen attenuation (pressure-induced N₂ continuum + O₂ line tails):
     *   γ_o ∝ p² × T^(−3)   (pressure broadening)
     * Reference γ_o ≈ 0.008 dB/km at sea level, 24 GHz.
     */
    if (o2_dbkm) {
        *o2_dbkm = O2_REF_DBKM * po * po * tr * tr * tr;
    }

    /*
     * Water-vapour attenuation (22.235 GHz resonance line tail at 24 GHz):
     *   γ_w ∝ ρ × T^(−1.5)   (line strength ∝ density)
     * Reference γ_w ≈ 0.085 dB/km at ρ = 7.5 g/m³, 24 GHz.
     */
    if (h2o_dbkm) {
        *h2o_dbkm = H2O_REF_DBKM * rr * pow(tr, 1.5);
    }
}

double gas_slant_attenuation(double f_ghz, double P_hpa, double T_k,
                             double rho_gm3, double elev_deg) {
    double o2_dbkm, h2o_dbkm;
    double se;

    gas_specific_attenuation(f_ghz, P_hpa, T_k, rho_gm3, &o2_dbkm, &h2o_dbkm);

    se = sin_elev(elev_deg);

    /* Equivalent-height method: A = γ × h / sin(θ) */
    return (o2_dbkm * O2_EQUIV_KM + h2o_dbkm * H2O_EQUIV_KM) / se;
}

/* --------------------------------------------------------------------------
 * 5. LINK BUDGET ORCHESTRATOR
 * -------------------------------------------------------------------------- */

void compute_link_budget(const PropagationScenario *scenario,
                         LinkBudgetResult *budget) {
    double f_ghz;
    double gamma_rain, gamma_fog;
    double o2_dbkm, h2o_dbkm;

    if (!scenario || !budget) return;

    f_ghz = scenario->frequency_hz / 1.0e9;

    /* --- 1. Free-space path loss --- */
    budget->fspl_db = compute_fspl(scenario->frequency_hz, scenario->distance_km);

    /* --- 2. Rain attenuation --- */
    gamma_rain = rain_specific_attenuation(
        f_ghz, scenario->rain_rate_mmh,
        scenario->elevation_deg, scenario->polarization_deg);
    budget->rain_specific_dbkm = gamma_rain;
    budget->rain_slant_km = RAIN_HEIGHT_KM / sin_elev(scenario->elevation_deg);
    budget->rain_atten_db = rain_slant_attenuation(gamma_rain,
                                                   scenario->elevation_deg);

    /* --- 3. Fog/cloud attenuation --- */
    gamma_fog = fog_kl_coefficient(f_ghz, scenario->surface_temp_k)
                * scenario->liquid_water_gm3;
    budget->fog_specific_dbkm = gamma_fog;
    budget->fog_atten_db = fog_slant_attenuation(
        f_ghz, scenario->surface_temp_k,
        scenario->liquid_water_gm3, scenario->elevation_deg);

    /* --- 4. Gas attenuation --- */
    gas_specific_attenuation(f_ghz, scenario->surface_pressure_hpa,
                             scenario->surface_temp_k,
                             scenario->water_vapor_gm3,
                             &o2_dbkm, &h2o_dbkm);
    budget->gas_o2_dbkm = o2_dbkm;
    budget->gas_h2o_dbkm = h2o_dbkm;
    budget->gas_o2_km = O2_EQUIV_KM;
    budget->gas_h2o_km = H2O_EQUIV_KM;
    budget->gas_atten_db = gas_slant_attenuation(
        f_ghz, scenario->surface_pressure_hpa,
        scenario->surface_temp_k,
        scenario->water_vapor_gm3, scenario->elevation_deg);

    /* --- 5. Total attenuation --- */
    budget->total_atten_db = budget->fspl_db
                           + budget->rain_atten_db
                           + budget->fog_atten_db
                           + budget->gas_atten_db;

    /* --- 6. Link margin --- */
    budget->eirp_dbm = scenario->eirp_dbm;
    budget->rx_sensitivity_dbm = scenario->rx_sensitivity_dbm;
    budget->rx_power_dbm = budget->eirp_dbm
                         - budget->total_atten_db
                         + scenario->rx_gain_dbi;
    budget->link_margin_db = budget->rx_power_dbm
                           - budget->rx_sensitivity_dbm;
}

/* --------------------------------------------------------------------------
 * 6. PRINT LINK BUDGET  (formatted table to stdout)
 * -------------------------------------------------------------------------- */

void print_link_budget(const LinkBudgetResult *b) {
    if (!b) return;

    printf("\n");
    printf("================================================================================\n");
    printf("  LINK BUDGET ANALYSIS — Part D (Propagation)\n");
    printf("================================================================================\n");

    printf("\n  --- Transmitter ---\n");
    printf("  EIRP                                   %13.2f  dBm\n", b->eirp_dbm);

    printf("\n  --- Propagation Losses ---\n");
    printf("  Free-Space Path Loss (FSPL)            %13.2f  dB\n", b->fspl_db);

    if (b->rain_atten_db > 0.0) {
        printf("  Rain attenuation                       %13.2f  dB\n", b->rain_atten_db);
        printf("    Specific rain attenuation            %13.4f  dB/km\n", b->rain_specific_dbkm);
        printf("    Slant-path length (rain)             %13.2f  km\n", b->rain_slant_km);
    } else {
        printf("  Rain attenuation                         (none — R = 0 mm/h)\n");
    }

    if (b->fog_atten_db > 0.0) {
        printf("  Fog/cloud attenuation                  %13.2f  dB\n", b->fog_atten_db);
        printf("    Specific fog attenuation              %13.6f  dB/km\n", b->fog_specific_dbkm);
    } else {
        printf("  Fog/cloud attenuation                     (none — M = 0 g/m³)\n");
    }

    if (b->gas_atten_db > 0.0) {
        printf("  Gas attenuation (O₂ + H₂O)             %13.2f  dB\n", b->gas_atten_db);
        printf("    Oxygen (O₂)  γ_o = %7.4f dB/km × %4.1f km\n",
               b->gas_o2_dbkm, b->gas_o2_km);
        printf("    Water vapour (H₂O) γ_w = %7.4f dB/km × %4.1f km\n",
               b->gas_h2o_dbkm, b->gas_h2o_km);
    } else {
        printf("  Gas attenuation                           (none)\n");
    }

    printf("  ───────────────────────────────────────────────────\n");
    printf("  TOTAL PROPAGATION LOSS                  %13.2f  dB\n", b->total_atten_db);

    printf("\n  --- Receiver ---\n");
    printf("  Receiver antenna gain                   %13.2f  dBi\n",
           b->rx_power_dbm - b->eirp_dbm + b->total_atten_db);
    printf("  Received power                          %13.2f  dBm\n", b->rx_power_dbm);
    printf("  Receiver sensitivity                    %13.2f  dBm\n", b->rx_sensitivity_dbm);

    printf("\n  --- Link Margin ---\n");
    if (b->link_margin_db >= 0.0) {
        printf("  LINK MARGIN                             %13.2f  dB   ✓\n",
               b->link_margin_db);
    } else {
        printf("  LINK MARGIN                             %13.2f  dB   ✗ (INSUFFICIENT)\n",
               b->link_margin_db);
    }
    printf("================================================================================\n");
}
