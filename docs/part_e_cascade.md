# Part E — Cascade Analysis (Receiver Front-End Design)

## Overview

This module translates `CascadeAnalyzer.m` (by teammate) into C.  It computes
the full RF receiver front-end cascade analysis for the 11-stage dual-conversion
superheterodyne receiver at 24 GHz:

1. **System power requirements** (Ni, Si, Pout for 1 Vpp)
2. **Friis noise cascade** (cumulative Gain, Noise Figure)
3. **IIP3 cascade** (cumulative third-order intercept)
4. **Dynamic range** (LDR, SFDR)
5. **Receiver sensitivity** (kTB + NF_total + SNR_required)

---

## Files

| File | Description |
|------|-------------|
| `include/cascade.h` | Public API: `CascadeParams`, `CascadeResult`, `StageCascadeEntry`, function declarations |
| `src/cascade.c` | Implementation of Friis cascade, IIP3 cascade, dynamic range, and sensitivity |
| `matlab/CascadeAnalyzer.m` | Original MATLAB reference (teammate) |

---

## Cascade Model

### Friis Noise Cascade

The total noise figure of N cascaded stages follows the Friis formula:

```
F_total = F_1 + (F_2 − 1)/G_1 + (F_3 − 1)/(G_1·G_2) + …
         + (F_N − 1)/(G_1·G_2·…·G_{N−1})
```

where **F_i** and **G_i** are the linear noise factor and linear gain of each
stage.  The first stage's gain dominates: a high-gain LNA as the first active
stage makes the noise figure of subsequent stages much less important.

### IIP3 Cascade

For cascaded third-order intercept:

```
1/IIP3_total = 1/IIP3_1 + G_1/IIP3_2 + G_1·G_2/IIP3_3 + …
```

The last high-gain stage before a nonlinear stage dominates the total IIP3.

---

## Receiver Chain (from `runtime_stage_models_target16.csv`, baseband_rx chain)

| # | Stage | Gain (dB) | NF (dB) | IIP3 (dBm) |
|---|-------|-----------|---------|------------|
| 1 | Switch | −0.3 | 0.3 | 20 |
| 2 | Preselector BPF | −0.4 | 0.4 | ∞ |
| 3 | **LNA 1** | **+29.0** | **1.8** | 0* |
| 4 | Image Rejection Filter | −3.5 | 3.5 | ∞ |
| 5 | Mixer 1 | −9.0 | 9.0 | 8 |
| 6 | BPF 2 | −0.6 | 0.6 | ∞ |
| 7 | **LNA 2** | **+23.7** | **0.27** | 20 |
| 8 | Mixer 2 | −6.5 | 6.5 | ∞ |
| 9 | BPF 3 | −3.0 | 3.0 | ∞ |
| 10 | **LNA 3** | **+50.0** | **3.5** | −7 |
| 11 | Limiter (RLM) | −0.04 | 0.04 | ∞ |

*\* CSV has IIP3=0 which is treated as unspecified (high IP3).*

---

## Computed Results

### Cascade Summary

| Parameter | Value |
|-----------|-------|
| **Total Gain** | **79.36 dB** |
| **Total Noise Figure** | **2.58 dB** |
| **Total IIP3** | **−37.10 dBm** |

The NF is excellent (2.58 dB) because LNA 1 (+29 dB gain, 1.8 dB NF)
dominates the Friis cascade.  The IIP3 is degraded by LNA 3 (−7 dBm IIP3)
after ~29 dB of preceding gain.

### System Power Requirements

| Parameter | Value | Formula |
|-----------|-------|---------|
| Input noise (Ni) | −93.83 dBm | k · T_ant · B |
| Input signal (Si) | −73.01 dBm | Ni + SNR_target |
| Target output (1 Vpp, 50 Ω) | +3.98 dBm | Vpp²/(8·R) in dBm |
| Required gain | 76.99 dB | Pout − Si |

### Dynamic Range

| Parameter | Value |
|-----------|-------|
| Output noise floor (No) | −10.37 dBm |
| Output P1dB | +31.66 dBm |
| OIP3 | +42.26 dBm |
| **LDR** (Linear Dynamic Range) | **42.03 dB** |
| **SFDR** (Spurious-Free Dynamic Range) | **35.09 dB** |

Where:
- LDR = P1dB_out − No  (range from noise floor to 1 dB compression)
- SFDR = (2/3)·(OIP3 − No)  (range where IM3 products stay below noise floor)

### Receiver Sensitivity

```
Sensitivity = kT₀B + NF_total + SNR_required
           = −90.96 + 2.58 + 26.5
           = −61.88 dBm
```

| Component | Value |
|-----------|-------|
| kT₀B @ 200 MHz | −90.96 dBm |
| Cascade NF | +2.58 dB |
| SNR required (64-APSK) | +26.50 dB (from assignment table) |
| **Sensitivity** | **−61.88 dBm** |

---

## Integration with Part D (Link Budget)

The cascade sensitivity feeds automatically into the propagation link budget:

```
P_rx   = EIRP − Total_Atten + G_rx
       = 85 − 222.83 + 40
       = −97.83 dBm

Margin = P_rx − Sensitivity
       = −97.83 − (−61.88)
       = −35.94 dB
```

The link does not close with the current parameters.  The following levers can
be adjusted to close it:

| Lever | Current | Target | Effect |
|-------|---------|--------|--------|
| Ground antenna gain | 40 dBi | 50–55 dBi | +10–15 dB margin |
| Rain rate | 10 mm/h | 5 mm/h (dry region) | −~6 dB loss |
| Elevation angle | 30° | 60° (higher) | shorter slant path |
| Bandwidth | 200 MHz | 100 MHz | −3 dB noise floor |
| Modulation | 64-APSK | 16-APSK | −6 dB SNR required |
| Added LNA gain | 29 dB | 35 dB | reduces cascade NF |

---

## Component Catalog (`data/component_catalog.csv`)

The cascade analysis reads datasheet-correct P1dB and IIP3 values from the
component catalog, NOT from the runtime stage-models CSV (which has wrong
values).  The mapping from stage position to catalog UID is hardcoded in a
lookup table in `cascade.c`:

| Pos | Stage name | Catalog UID | Datasheet |
|-----|-----------|-------------|-----------|
| 0 | bb_00_switch | `SPST_SWITCH_01` | MA4AGSW1 |
| 1 | bb_01_preselector_bpf | `FILTER_1_01` | ZVBP-K27G+ |
| 2 | bb_02_lna1 | `LNA_1_01` | ADL8142S |
| 3 | bb_03_image_rejection_filter | — | (generic) |
| 4 | bb_04_mixer1 | `MIXER_1_01` | HMC264LC3B |
| 5 | bb_05_bpf2 | `FILTER_2_01` | SXBP-1430+ |
| 6 | bb_06_lna2 | `LNA_2_01` | SAV-541-DG+ |
| 7 | bb_07_mixer2 | `MIXER_2_01` | SYM-25DHW+ |
| 8 | bb_08_bpf3 | `FILTER_3_01` | BFTC-500+ |
| 9 | bb_09_lna3 | `LNA_3_01` | ZHL-20W-13SWX+ |
| 10 | bb_10_rlm43_5w | — | (limiter, no catalog entry) |

For components with OIP3 in the catalog, IIP3 is computed as:
```
IIP3 = OIP3 − Gain   [dBm]
```

For components with IIP3 directly (mixers), the catalog value is used as-is.

This ensures the cascade analysis uses the same datasheet values as the
MATLAB `CascadeAnalyzer.m`.

### Verification

The following IIP3 values now match exactly between C and MATLAB:

| Stage | C (from catalog) | MATLAB | Match |
|-------|------------------|--------|-------|
| LNA1 (ADL8142S) | **−11.5 dBm** | −11.5 | ✅ |
| Mixer1 (HMC264LC3B) | **14.0 dBm** | 14.0 | ✅ |
| LNA2 (SAV-541-DG+) | **4.1 dBm** | 4.1 | ✅ |
| Mixer2 (SYM-25DHW+) | **30.0 dBm** | 30.0 | ✅ |
| LNA3 (ZHL-20W-13SWX+) | **−7.0 dBm** | −7.0 | ✅ |

## Usage

In `main.c`, the component catalog is loaded after the stage models:

```c
ComponentCatalog catalog;
component_catalog_load("data/component_catalog.csv", &catalog);

CascadeParams params = {
    .antenna_temp_k    = 150.0,
    .t0_k              = 290.0,
    .bw_hz             = 200.0e6,
    .vpp_out           = 1.0,
    .snr_target_db     = 20.82,
    .snr_required_db   = 26.5
};
CascadeResult result;
compute_cascade(&stage_cfg, &params, STAGE_CHAIN_BASEBAND_RX,
                &catalog, &result);
double sensitivity = result.sensitivity_dbm;  // used by propagation
```

```c
// Example output
// RECEIVER SENSITIVITY:       -61.88  dBm
```
