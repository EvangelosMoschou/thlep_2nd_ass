# Part D — Propagation Analysis (ITU-R Models)

## Overview

This module implements the complete propagation analysis for a 24 GHz K-band
satellite downlink (GEO orbit).  Five propagation mechanisms are modelled per
the requirements of **Μέρος Δ** of the final assignment:

1. **Free-Space Path Loss (FSPL)**
2. **Rain Attenuation** — ITU-R Recommendation P.838-3
3. **Fog / Cloud Attenuation** — ITU-R Recommendation P.840-9
4. **Atmospheric Gas Absorption (O₂ + H₂O)** — ITU-R Recommendation P.676-13
5. **Link Margin** — EIRP − total propagation loss + G_rx − sensitivity

---

## Files

| File | Description |
|------|-------------|
| `include/propagation.h` | Public API: scenario input struct, link budget result struct, function declarations |
| `src/propagation.c` | Implementation of all five models (ITU-R curve-fits, double-Debye, equivalent-height) |

---

## API Reference

### Scenario Parameters (`PropagationScenario`)

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `frequency_hz` | `double` | 24.0e9 | Carrier frequency [Hz] |
| `distance_km` | `double` | 36000.0 | Slant-path distance (GEO) [km] |
| `elevation_deg` | `double` | 44.0 | Elevation angle [°] |
| `polarization_deg` | `double` | 45.0 | Polarization tilt [°] (45 = circular, 0 = horiz, 90 = vert) |
| `rain_rate_mmh` | `double` | 10.0 | Point rain rate [mm/h] (0.01% exceedance) |
| `surface_temp_k` | `double` | 288.15 | Surface temperature [K] (15 °C) |
| `surface_pressure_hpa` | `double` | 1013.25 | Surface pressure [hPa] (sea level) |
| `water_vapor_gm3` | `double` | 7.5 | Surface water-vapour density [g/m³] |
| `liquid_water_gm3` | `double` | 0.05 | Cloud liquid-water density [g/m³] (0.05 = medium fog) |
| `eirp_dbm` | `double` | 85.0 | Satellite EIRP [dBm] |
| `rx_gain_dbi` | `double` | 68.7 | Ground-station antenna gain [dBi] |
| `rx_sensitivity_dbm` | `double` | −70.0 | Receiver sensitivity [dBm] (placeholder — from Part E) |

### Result Fields (`LinkBudgetResult`)

| Field | Unit | Description |
|-------|------|-------------|
| `fspl_db` | dB | Free-space path loss |
| `rain_atten_db` | dB | Slant-path rain attenuation |
| `fog_atten_db` | dB | Slant-path fog/cloud attenuation |
| `gas_atten_db` | dB | Slant-path gaseous attenuation (O₂ + H₂O) |
| `total_atten_db` | dB | Sum of all propagation losses |
| `eirp_dbm` | dBm | EIRP |
| `rx_power_dbm` | dBm | Received power (EIRP − losses + G_rx) |
| `rx_sensitivity_dbm` | dBm | Receiver sensitivity |
| `link_margin_db` | dB | Link margin (≥ 0 means link closes) |

---

## Model Details

### 1. Free-Space Path Loss

```
FSPL_dB = 92.45 + 20·log₁₀(f_GHz) + 20·log₁₀(d_km)
```

Derived from the Friis transmission equation:

```
FSPL = (4π·d·f / c)²   [linear]
```

### 2. Rain Attenuation — ITU-R P.838-3

The specific rain attenuation **γ_R** [dB/km] follows the power-law:

```
γ_R = k · R^α
```

where **k** and **α** are frequency- and polarization-dependent coefficients
computed from Gaussian-exponent curve-fits using Tables 1–4 of the
recommendation:

```
log₁₀(k_H)  = Σⱼ₌₁⁴ a_j · exp(−((log₁₀f − b_j)/c_j)²) + m_k·log₁₀f + c_k
log₁₀(k_V)  = Σⱼ₌₁⁴ a_j · exp(−((log₁₀f − b_j)/c_j)²) + m_k·log₁₀f + c_k
α_H          = Σⱼ₌₁⁵ a_j · exp(−((log₁₀f − b_j)/c_j)²) + m_α·log₁₀f + c_α
α_V          = Σⱼ₌₁⁵ a_j · exp(−((log₁₀f − b_j)/c_j)²) + m_α·log₁₀f + c_α
```

For circular polarisation (τ = 45°):

```
k = [k_H + k_V + (k_H − k_V)·cos²θ·cos²τ] / 2
α = [k_H·α_H + k_V·α_V + (k_H·α_H − k_V·α_V)·cos²θ·cos²τ] / (2k)
```

The slant-path rain attenuation uses the effective rain height **h_R = 4 km**
(ITU-R P.839):

```
A_rain = γ_R · h_R / sin(θ)
```

### 3. Fog / Cloud Attenuation — ITU-R P.840-9

The specific attenuation coefficient **K_l** [(dB/km)/(g/m³)] uses the
double-Debye model for the complex dielectric permittivity of liquid water:

```
ε₀  = 77.66 + 103.3·(300/T − 1)
ε₁  = 0.0671·ε₀
ε₂  = 3.52

f_p = 20.20 − 146·(300/T − 1) + 316·(300/T − 1)²    [GHz]
f_s = 39.8·f_p                                        [GHz]

ε''(f) = f·(ε₀−ε₁) / [f_p·(1+(f/f_p)²)]
       + f·(ε₁−ε₂) / [f_s·(1+(f/f_s)²)]

ε'(f)  = (ε₀−ε₁) / (1+(f/f_p)²) + (ε₁−ε₂) / (1+(f/f_s)²) + ε₂

η(f)   = (2 + ε'(f)) / ε''(f)

K_l    = 0.819·f / [ε''(f)·(1 + η²)]    [(dB/km)/(g/m³)]

γ_c    = K_l · M                         [dB/km]
```

where **M** is the liquid-water density (0.05 g/m³ for medium fog, 0.5 g/m³
for thick fog).  The slant path:

```
A_fog = γ_c / sin(θ)
```

### 4. Atmospheric Gas Absorption — ITU-R P.676-13

At 24 GHz, away from major absorption lines (60 GHz O₂ complex), the specific
attenuation is modelled with sea-level reference values scaled by pressure and
humidity:

| Component | Reference γ @ sea level | Scaling |
|-----------|------------------------|---------|
| Oxygen (O₂) | 0.0080 dB/km | γ_o = γ_o_ref · (P/1013.25)² · (288.15/T)³ |
| Water vapour (H₂O) | 0.0850 dB/km | γ_w = γ_w_ref · (ρ/7.5) · (288.15/T)^1.5 |

The slant path uses the equivalent-height method:

```
A_gas = γ_o · h_o / sin(θ) + γ_w · h_w / sin(θ)
```

where **h_o ≈ 6 km** (oxygen) and **h_w ≈ 2 km** (water vapour) at 24 GHz.

### 5. Link Margin

```
P_rx     = EIRP_dBm − A_total_dB + G_rx_dBi    [dBm]
Margin   = P_rx − sensitivity                    [dB]
```

A positive margin means the link closes (received power exceeds receiver
sensitivity by the margin amount).

---

## Default Scenario

**Satellite downlink** (GEO, K-band):

| Parameter | Value | Source |
|-----------|-------|--------|
| Frequency | 24.0 GHz | Receiver design |
| Orbit | GEO (36 000 km) | Assignment assumption |
| Elevation | 44° | Athens→SES-17 |
| Polarisation | Circular (45°) | TBD with team |
| Rain rate | 10 mm/h (0.01 %) | Generic ITU reference |
| Pressure | 1013.25 hPa | Sea level standard |
| Temperature | 288.15 K (15 °C) | Standard |
| Water vapour | 7.5 g/m³ | Reference |
| Fog density | 0.05 g/m³ | Medium fog (300 m vis.) |
| EIRP | 85 dBm | Assignment (satellite) |
| Antenna gain | 40 dBi | Typical ground station |
| Sensitivity | −70 dBm | Placeholder (Part E) |

---

## Sample Output

```
================================================================================
  LINK BUDGET ANALYSIS — Part D (Propagation)
================================================================================

  --- Transmitter ---
  EIRP                                           85.00  dBm

  --- Propagation Losses ---
  Free-Space Path Loss (FSPL)                   211.18  dB
  Rain attenuation                               11.18  dB
    Specific rain attenuation                   1.3971  dB/km
    Slant-path length (rain)                      8.00  km
  Fog/cloud attenuation                           0.03  dB
    Specific fog attenuation                   0.017013  dB/km
  Gas attenuation (O₂ + H₂O)                      0.44  dB
    Oxygen (O₂)  γ_o =  0.0080 dB/km ×  6.0 km
    Water vapour (H₂O) γ_w =  0.0850 dB/km ×  2.0 km
  ───────────────────────────────────────────────────
  TOTAL PROPAGATION LOSS                         222.83  dB

  --- Receiver ---
  Receiver antenna gain                           40.00  dBi
  Received power                                 -97.83  dBm
  Receiver sensitivity                           -70.00  dBm

  --- Link Margin ---
  LINK MARGIN                                    -27.83  dB   ✗ (INSUFFICIENT)
================================================================================
```

**Note:** The negative margin is expected with the placeholder sensitivity of
−70 dBm.  Once the receiver cascade analysis (Part E) provides the actual
sensitivity, the margin will be recomputed.  A realistic GEO K-band receiver
with a 40 dBi antenna typically achieves sensitivity around −95 to −105 dBm,
which would close this link with several dB of margin.

---

## Sensitivity Analysis

Users can vary these key parameters via the struct fields to explore different
scenarios:

| Parameter | Effect | Typical Range |
|-----------|--------|---------------|
| `rain_rate_mmh` | Rain attenuation increases with R | 0–150 mm/h |
| `elevation_deg` | Lower elevation → longer slant path → higher losses | 5°–90° |
| `liquid_water_gm3` | Fog density | 0 (clear) – 0.5 (thick fog) |
| `rx_sensitivity_dbm` | Link margin directly affected | −50 to −110 dBm |
| `distance_km` | FSPL scales with distance | 500 (LEO) – 36000 (GEO) |

---

## Integration

In `main.c`, the propagation analysis runs before the receiver simulation:

```c
PropagationScenario prop_scenario = {
    .frequency_hz         = cfg.carrier_hz,      /* 24.0e9 */
    .distance_km          = 36000.0,
    .elevation_deg        = 44.0,
    .polarization_deg     = 45.0,
    .rain_rate_mmh        = 10.0,
    .surface_temp_k       = 288.15,
    .surface_pressure_hpa = 1013.25,
    .water_vapor_gm3      = 7.5,
    .liquid_water_gm3     = 0.05,
    .eirp_dbm             = 85.0,
    .rx_gain_dbi          = 68.7,
    .rx_sensitivity_dbm   = -70.0
};
LinkBudgetResult prop_budget;
compute_link_budget(&prop_scenario, &prop_budget);
print_link_budget(&prop_budget);
```

The result struct `prop_budget` can also be accessed programmatically for
automated sensitivity sweeps or inclusion in reports.
