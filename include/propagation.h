#ifndef PROPAGATION_H
#define PROPAGATION_H

#include <stddef.h>

/*
 * ============================================================================
 * propagation.h — ITU-R Propagation Models for Satellite Link Budget
 * ============================================================================
 *
 * PURPOSE:
 *   Implements Part D of the assignment: complete propagation analysis over
 *   a satellite-to-ground link at 24 GHz (K-band).  The module computes:
 *
 *     1. Free Space Path Loss (FSPL)
 *     2. Rain attenuation     (ITU-R P.838-3)
 *     3. Fog/cloud attenuation (ITU-R P.840-9, double-Debye model)
 *     4. Atmospheric gas/O₂   (ITU-R P.676-13, reference-value method)
 *     5. Link margin           (EIRP − total loss + G_rx − sensitivity)
 *
 * USAGE:
 *   @code
 *   PropagationScenario scenario = {
 *       .frequency_hz        = 24.0e9,
 *       .distance_km         = 36000.0,
 *       .elevation_deg       = 44.0,
 *       .polarization_deg    = 45.0,         // circular
 *       .rain_rate_mmh       = 10.0,          // 0.01% exceedance
 *       .surface_temp_k      = 288.15,
 *       .surface_pressure_hpa = 1013.25,
 *       .water_vapor_gm3     = 7.5,
 *       .liquid_water_gm3    = 0.05,          // medium fog
 *       .eirp_dbm            = 85.0,
 *       .rx_gain_dbi         = 40.0,          // typical ground-station antenna
 *       .rx_sensitivity_dbm  = -70.0          // placeholder from Part E
 *   };
 *
 *   LinkBudgetResult budget;
 *   compute_link_budget(&scenario, &budget);
 *   print_link_budget(&budget);
 *   @endcode
 *
 * STANDARDS:
 *   - ITU-R P.838-3  (09/2005): Specific rain attenuation model
 *   - ITU-R P.840-9  (08/2023): Attenuation due to clouds and fog
 *   - ITU-R P.676-13 (08/2022): Attenuation by atmospheric gases
 * ============================================================================
 */

/* ============================================================================
 * SCENARIO PARAMETERS
 * ============================================================================ */

typedef struct PropagationScenario {
    /* --- Link geometry --- */
    double frequency_hz;         /**< Carrier frequency [Hz].         (e.g. 24.0e9) */
    double distance_km;          /**< Slant path distance [km].       (GEO ~36000 km) */
    double elevation_deg;        /**< Elevation angle [deg].          (e.g. 44°) */

    /* --- Polarisation (for rain model) --- */
    double polarization_deg;     /**< Polarisation tilt angle [deg].
                                      τ=45° for circular, τ=0° for horizontal,
                                      τ=90° for vertical.             TBD by team. */

    /* --- Rain (ITU-R P.838-3) --- */
    double rain_rate_mmh;        /**< Point rain rate [mm/h].         (10 mm/h for 0.01% exceedance) */

    /* --- Surface meteorology (for gas/fog) --- */
    double surface_temp_k;       /**< Surface temperature [K].        (288.15 K = 15 °C) */
    double surface_pressure_hpa; /**< Surface barometric pressure [hPa]. (1013.25 hPa) */
    double water_vapor_gm3;      /**< Surface water-vapour density [g/m³]. (7.5 g/m³) */
    double liquid_water_gm3;     /**< Cloud/fog liquid-water density [g/m³].
                                      0.05 → medium fog (300 m visibility)
                                      0.5  → thick fog  (50 m visibility) */

    /* --- Transmitter / receiver (for link margin) --- */
    double eirp_dbm;             /**< Effective Isotropic Radiated Power [dBm].  (85 dBm for satellite) */
    double rx_gain_dbi;          /**< Receiver antenna gain [dBi].   (ground station, e.g. 40 dBi) */
    double rx_sensitivity_dbm;   /**< Receiver sensitivity [dBm].    (from Part E, placeholder) */
} PropagationScenario;

/* ============================================================================
 * LINK BUDGET RESULT
 * ============================================================================ */

typedef struct LinkBudgetResult {
    /* Individual loss components [dB] */
    double fspl_db;              /**< Free-space path loss */
    double rain_atten_db;        /**< Slant-path rain attenuation */
    double fog_atten_db;         /**< Slant-path fog/cloud attenuation */
    double gas_atten_db;         /**< Slant-path gaseous (O₂ + H₂O) attenuation */
    double total_atten_db;       /**< Sum of all propagation losses */

    /* Power levels [dBm] */
    double eirp_dbm;             /**< EIRP from scenario */
    double rx_power_dbm;         /**< Received power = EIRP − total_atten + G_rx */
    double rx_sensitivity_dbm;   /**< Sensitivity from scenario */
    double link_margin_db;       /**< rx_power − sensitivity (≥ 0 means link works) */

    /* Per-component details */
    double rain_specific_dbkm;   /**< Rain: specific attenuation γ_R [dB/km] */
    double rain_slant_km;        /**< Rain: effective slant path length [km] */
    double fog_specific_dbkm;    /**< Fog:  specific attenuation γ_c [dB/km] */
    double gas_o2_dbkm;          /**< Gas:  oxygen specific attenuation [dB/km] */
    double gas_h2o_dbkm;         /**< Gas:  water-vapour specific attenuation [dB/km] */
    double gas_o2_km;            /**< Gas:  oxygen equivalent height [km] */
    double gas_h2o_km;           /**< Gas:  water-vapour equivalent height [km] */
} LinkBudgetResult;

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/**
 * @brief Compute Free-Space Path Loss.
 *
 *   FSPL = (4π × d × f / c)²   [linear]
 *   FSPL_dB = 20·log10(4π) + 20·log10(d) + 20·log10(f) − 20·log10(c)
 *
 * Equivalent: FSPL_dB = 92.45 + 20·log10(f_GHz) + 20·log10(d_km)
 *
 * @param freq_hz   Carrier frequency [Hz]
 * @param dist_km   Slant-path distance [km]
 * @return FSPL in dB
 */
double compute_fspl(double freq_hz, double dist_km);

/**
 * @brief ITU-R P.838-3: specific rain attenuation γ_R = k × R^α [dB/km].
 *
 * Implements the full curve-fit model with 4-term (k) and 5-term (α)
 * Gaussian-exponent sums from Tables 1–4 of the recommendation.
 *
 * For circular polarisation use τ = 45°.  The result is combined from
 * horizontal (H) and vertical (V) coefficients via equations (4) and (5).
 *
 * @param f_ghz        Frequency [GHz]
 * @param R_mmh        Rain rate [mm/h]
 * @param elev_deg     Path elevation angle [deg]
 * @param tau_deg      Polarisation tilt angle [deg]  (45° = circular)
 * @return Specific rain attenuation [dB/km]
 */
double rain_specific_attenuation(double f_ghz, double R_mmh,
                                 double elev_deg, double tau_deg);

/**
 * @brief Compute slant-path rain attenuation.
 *
 *   A_rain = γ_R × h_R / sin(θ)
 *
 * where h_R ≈ 4.0 km is the effective rain height (ITU-R P.839).
 *
 * @param gamma_dbkm  Specific rain attenuation [dB/km]
 * @param elev_deg    Elevation angle [deg]
 * @return Slant-path rain attenuation [dB]
 */
double rain_slant_attenuation(double gamma_dbkm, double elev_deg);

/**
 * @brief ITU-R P.840-9: cloud/fog specific attenuation coefficient K_l.
 *
 * Uses the double-Debye model for the complex dielectric permittivity
 * of liquid water (Rayleigh scattering).  Result in (dB/km)/(g/m³).
 *
 * @param f_ghz  Frequency [GHz]
 * @param T_k    Cloud liquid-water temperature [K]  (typically 273–290 K)
 * @return Specific attenuation coefficient [(dB/km)/(g/m³)]
 */
double fog_kl_coefficient(double f_ghz, double T_k);

/**
 * @brief Compute slant-path fog/cloud attenuation.
 *
 *   γ_c = K_l × M           [dB/km]
 *   A_fog = γ_c / sin(θ)
 *
 * where M is the liquid-water density [g/m³] and θ is the elevation angle.
 *
 * @param f_ghz       Frequency [GHz]
 * @param T_k         Cloud temperature [K]
 * @param M_gm3       Liquid-water density [g/m³]
 * @param elev_deg    Elevation angle [deg]
 * @return Slant-path fog attenuation [dB]
 */
double fog_slant_attenuation(double f_ghz, double T_k, double M_gm3,
                             double elev_deg);

/**
 * @brief ITU-R P.676-13 (simplified): gas specific attenuation at 24 GHz.
 *
 * Implements a practical sea-level reference model based on the ITU
 * specific-attenuation curves.  Oxygen and water-vapour attenuations are
 * computed separately and scaled by surface pressure / water-vapour density.
 *
 * The oxygen specific attenuation at 24 GHz is dominated by the pressure-
 * induced nitrogen continuum and the tails of the 60 GHz O₂ complex:
 *   γ_o ≈ 0.008 dB/km  (at 1013.25 hPa, 288.15 K)
 *
 * The water-vapour specific attenuation at 24 GHz is influenced by the
 * 22.235 GHz resonance line:
 *   γ_w ≈ 0.085 dB/km   (at ρ = 7.5 g/m³, 288.15 K)
 *
 * @param f_ghz   Frequency [GHz]
 * @param P_hpa   Surface pressure [hPa]
 * @param T_k     Surface temperature [K]
 * @param rho_gm3 Surface water-vapour density [g/m³]
 * @param o2_dbkm [out] Oxygen specific attenuation [dB/km]
 * @param h2o_dbkm[out] Water-vapour specific attenuation [dB/km]
 */
void gas_specific_attenuation(double f_ghz, double P_hpa, double T_k,
                              double rho_gm3,
                              double *o2_dbkm, double *h2o_dbkm);

/**
 * @brief Compute slant-path gaseous (O₂ + H₂O) attenuation.
 *
 * Uses the equivalent-height method:
 *   A = γ_o × h_o / sin(θ) + γ_w × h_w / sin(θ)
 *
 * where h_o ≈ 6 km (O₂) and h_w ≈ 2 km (H₂O) at 24 GHz.
 *
 * @param f_ghz       Frequency [GHz]
 * @param P_hpa       Surface pressure [hPa]
 * @param T_k         Surface temperature [K]
 * @param rho_gm3     Water-vapour density [g/m³]
 * @param elev_deg    Elevation angle [deg]
 * @return Slant-path gas attenuation [dB]
 */
double gas_slant_attenuation(double f_ghz, double P_hpa, double T_k,
                             double rho_gm3, double elev_deg);

/**
 * @brief Compute the complete link budget.
 *
 * Runs all five propagation models and fills the LinkBudgetResult struct.
 * The result includes individual loss components, their sum, and the
 * resulting link margin.
 *
 * @param scenario  [in]  Scenario parameters (frequency, distance, weather…)
 * @param budget    [out] Computed link budget (all loss components + margin)
 */
void compute_link_budget(const PropagationScenario *scenario,
                         LinkBudgetResult *budget);

/**
 * @brief Print a formatted link-budget table to stdout.
 *
 * Matches the style of print_metrics() used elsewhere in the simulator.
 *
 * @param budget  Link-budget result to display
 */
void print_link_budget(const LinkBudgetResult *budget);

/**
 * @brief Print the Adaptive Coding & Modulation (ACM) table.
 *
 * Given the current SNR at the receiver input, shows which DVB-S2X
 * MODCODs are supported.  Thresholds are from the assignment table.
 *
 * @param snr_at_receiver_db  Actual SNR at receiver input [dB]
 */
void print_modcod_table(double snr_at_receiver_db);

#endif /* PROPAGATION_H */
