# Receiver Dual Simulation — Complete Explanation

## Table of Contents

1. [What This Program Does](#1-what-this-program-does)
2. [Background Concepts](#2-background-concepts)
3. [Project Structure](#3-project-structure)
4. [Architecture Overview](#4-architecture-overview)
5. [Signal Flow — End to End](#5-signal-flow--end-to-end)
6. [File-by-File Breakdown](#6-file-by-file-breakdown)
7. [Key Data Structures](#7-key-data-structures)
8. [Key Algorithms Explained](#8-key-algorithms-explained)
9. [CSV Configuration System](#9-csv-configuration-system)
10. [Output Files](#10-output-files)
11. [How to Build and Run](#11-how-to-build-and-run)
12. [Glossary](#12-glossary)

---

## 1. What This Program Does

This is a **satellite receiver simulator** written in C. It models how a digital communication signal (specifically **64-APSK**, a modulation scheme used in the **DVB-S2X** satellite TV standard) is degraded as it passes through the stages of a receiver.

In simple terms: imagine you're watching satellite TV. The satellite transmits a signal, and your dish receives it. Between the dish and the screen, the signal passes through amplifiers, filters, and mixers — each of which adds noise and distorts the signal slightly. This program simulates that entire journey and measures exactly how much the signal quality degrades at each step.

### The Two Simulation Modes

The simulator runs the **same signal** through **two independent paths**, then compares them:

| Path | Name | Speed | Fidelity |
|------|------|-------|----------|
| **Path 1** | Complex Baseband | Very fast | Analytical approximation |
| **Path 2** | Brute-Force RF | Slow | Physically realistic |

- **Complex Baseband**: Processes the signal as abstract I/Q (In-phase/Quadrature) numbers. Think of it as doing the math on paper — fast and accurate, but it doesn't account for real radio effects.

- **Brute-Force RF**: Actually creates a simulated 24 GHz radio wave, samples it at 96 GHz, passes it through the receiver stages as if it were a real signal, then converts it back. This captures effects that the baseband model misses (like aliasing or mixer artifacts).

---

## 2. Background Concepts

### 2.1 What is 64-APSK?

**APSK** = Amplitude and Phase Shift Keying. It's a way of encoding digital data into a radio signal.

Each "symbol" the transmitter sends is one of **64 predefined complex numbers** (points on a 2D plane). These 64 points are arranged in **4 concentric rings**:

```
Ring 0 (innermost):  radius = 1.00    (4 points per quadrant → 16 total)
Ring 1:              radius = 1.88    (4 points per quadrant → 16 total)
Ring 2:              radius = 2.72    (4 points per quadrant → 16 total)
Ring 3 (outermost):  radius = 3.95    (4 points per quadrant → 16 total)
```

Since log₂(64) = 6, each symbol carries **6 bits** of information.

### 2.2 What is I/Q?

A complex signal has two components:
- **I (In-phase)**: The "horizontal" part (real component)
- **Q (Quadrature)**: The "vertical" part (imaginary component)

Together, they define a point on a 2D plane. The signal can be written as `I + jQ` where `j = √(-1)`.

### 2.3 What is SNR?

**SNR** = Signal-to-Noise Ratio. Measured in **dB (decibels)**.

`SNR_dB = 10 × log₁₀(P_signal / P_noise)`

Higher SNR = cleaner signal. For 64-APSK to work reliably, you typically need SNR > 18 dB.

### 2.4 What is EVM?

**EVM** = Error Vector Magnitude. Measured as a **percentage**.

`EVM% = √(P_noise / P_signal) × 100`

It measures how far received symbols deviate from their ideal positions. Lower EVM = better. An EVM of 5% means the average error is 5% of the signal amplitude.

### 2.5 What is Noise Figure (NF)?

Every real hardware component (amplifier, filter, mixer) adds some noise. **Noise Figure** measures how much noise a component adds, in dB.

- NF = 0 dB → Perfect (adds no noise)
- NF = 3 dB → Doubles the noise power
- NF = 10 dB → Multiplies noise power by 10

### 2.6 What is AWGN?

**AWGN** = Additive White Gaussian Noise. The standard noise model for communications.

- **Additive**: Noise is added to the signal (not multiplied)
- **White**: Equal noise power at all frequencies
- **Gaussian**: Each noise sample follows a bell-curve distribution

### 2.7 What are dB (Decibels)?

A logarithmic scale for expressing ratios:

| dB | Linear Factor | Meaning |
|----|--------------|---------|
| +20 dB | ×100 | 100× amplification |
| +10 dB | ×10 | 10× amplification |
| +3 dB | ×2 | Doubled |
| 0 dB | ×1 | No change |
| -3 dB | ×0.5 | Halved |
| -10 dB | ×0.1 | 10× attenuation |
| -20 dB | ×0.01 | 100× attenuation |

---

## 3. Project Structure

```
receiver_dual_sim/
├── Makefile                 — Build system (compile, run, clean)
│
├── include/                 — Header files (type and function declarations)
│   ├── sim_types.h          — Complex, StageMetric, SimConfig structs
│   ├── prng.h               — PRNG function declarations
│   ├── stage_models.h       — Stage configuration types + CSV loader API
│   └── stage_artifacts.h    — Output writer function declarations
│
├── src/                     — Source code (implementation)
│   ├── main.c               — Main simulation engine (~1600 lines)
│   ├── prng.c               — Random number generator (~300 lines)
│   ├── stage_models.c       — CSV parser for stage definitions (~690 lines)
│   └── stage_artifacts.c    — CSV/SVG output generation (~840 lines)
│
├── stage_models/            — Receiver configuration CSV files
│   ├── stage_models.csv                    — Design reference
│   └── runtime_stage_models_target16.csv   — Active simulator config
│
├── data/                    — Input data (if needed)
│
├── out/                     — Simulation output (created at runtime)
│   ├── topology_sim_1/
│   │   ├── csv/             — CSV data files
│   │   └── svg/             — SVG chart images
│   ├── topology_sim_2/         — (same structure)
│   ├── topology_sim_3/
│   ├── topology_sim_4/
│   └── topology_sim_5/
│
└── EXPLANATION.md           — This file
```

---

## 4. Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                           MAIN.C                                    │
│                                                                     │
│  ┌──────────┐   ┌──────────────┐   ┌──────────────────────────┐    │
│  │ CLI Parse │→ │ Build 64-APSK │→ │ Generate Random Symbols   │    │
│  │  --snr    │   │ Constellation │   │ (256 symbols by default) │    │
│  │  --seed   │   └──────────────┘   └──────────────────────────┘    │
│  │  --stage  │                              │                       │
│  └──────────┘                              ▼                       │
│                                 ┌──────────────────────┐           │
│                                 │ Calculate Link Budget │           │
│                                 │ P_noise = kTB         │           │
│                                 └──────────┬───────────┘           │
│                                            │                       │
│                             ┌──────────────┼──────────────┐        │
│                             ▼              ▼              │        │
│                 ┌───────────────┐  ┌────────────────┐     │        │
│                 │  Path 1:      │  │ Path 2:        │     │        │
│                 │  Complex      │  │ Brute-Force    │     │        │
│                 │  Baseband     │  │ RF             │     │        │
│                 │               │  │                │     │        │
│                 │ Add AWGN →    │  │ Upsample →     │     │        │
│                 │ Stage chain → │  │ Modulate →     │     │        │
│                 │ Measure SNR   │  │ Add AWGN →     │     │        │
│                 │               │  │ RF stages →    │     │        │
│                 │               │  │ Downconvert →  │     │        │
│                 │               │  │ BB stages →    │     │        │
│                 │               │  │ Measure SNR    │     │        │
│                 └──────┬────────┘  └───────┬────────┘     │        │
│                        │                   │              │        │
│                        ▼                   ▼              │        │
│                 ┌──────────────────────────────────┐      │        │
│                 │        Write Outputs             │      │        │
│                 │  CSV data + SVG visualizations   │      │        │
│                 │  Console summary table            │      │        │
│                 └──────────────────────────────────┘      │        │
└─────────────────────────────────────────────────────────────────────┘

External modules:
┌──────────┐  ┌───────────────┐  ┌──────────────────┐
│ PRNG.C   │  │ STAGE_MODELS.C│  │ STAGE_ARTIFACTS.C│
│          │  │               │  │                  │
│ Seed →   │  │ CSV file →    │  │ Data arrays →    │
│ Uniform  │  │ Parse header  │  │ CSV files        │
│ Gaussian │  │ Parse rows    │  │ SVG charts       │
│ uint32   │  │ Build chains  │  │ Metric tables    │
└──────────┘  └───────────────┘  └──────────────────┘
```

### The Three Receiver Chains

The CSV file defines three separate chains of stages:

| Chain | ID | Purpose |
|-------|----|---------|
| `baseband_rx` | 0 | Fast analytical path (I/Q processing only) |
| `rf_frontend` | 1 | RF stages before downconversion (real signal) |
| `rf_postmix_bb` | 2 | Baseband stages after downconversion (complex I/Q) |

**Path 1** uses only `baseband_rx`.  
**Path 2** uses `rf_frontend` → mixer → `rf_postmix_bb`.

---

## 5. Signal Flow — End to End

### Phase 1: Transmitter (common to both paths)

```
1. Build 64-APSK constellation (64 fixed I/Q points, normalized to Es=1)
2. Randomly pick 256 symbols from the constellation (using PRNG)
3. Record the transmitted sequence as both "reference" (clean) and "signal" (will degrade)
```

### Phase 2a: Complex Baseband Path

```
4. Add AWGN to the "signal" copy at the configured SNR level
5. For each stage in baseband_rx chain:
   a. Apply moving-average filter (if filter_len > 1)
   b. Apply gain (amplify or attenuate)
   c. Inject stage noise (based on Noise Figure)
   d. Measure SNR & EVM
   e. Write CSV data + SVG constellation plot
```

### Phase 2b: Brute-Force RF Path

```
4. Upsample: repeat each symbol SPS times (rectangular pulses)
5. IQ modulate onto 24 GHz carrier:
   RF[t] = I[t]·cos(2πf_c·t) − Q[t]·sin(2πf_c·t)
6. Add AWGN to the RF waveform
7. For each stage in rf_frontend chain:
   a. Filter → Gain → Noise (same as baseband, but real signal)
8. Downconvert RF → baseband:
   I_raw = 2·RF·cos(2πf_c·t)    Q_raw = −2·RF·sin(2πf_c·t)
   Apply low-pass filter to remove 2×f_c image
9. Downsample: pick center sample of each symbol period
10. For each stage in rf_postmix_bb chain:
    a. Filter → Gain → Noise (complex I/Q signal)
```

### Phase 3: Output

```
11. Write stage metrics CSV and SVG for both paths
12. Print console summary with SNR/EVM at each stage
```

---

## 6. File-by-File Breakdown

### 6.1 `src/main.c` — The Simulation Engine

This is the largest file (~1600 lines) and the heart of the simulator. It contains:

| Function | Purpose |
|----------|---------|
| `main()` | Entry point — CLI parsing, orchestration, cleanup |
| `build_64apsk_constellation_dvbs2x()` | Construct the 64-APSK constellation from DVB-S2X spec |
| `normalize_constellation()` | Scale constellation to unit average power |
| `generate_symbols()` | Randomly select transmitted symbols |
| `add_awgn_complex()` / `add_awgn_real()` | Inject Gaussian noise |
| `apply_stage_complex()` / `apply_stage_real()` | Process signal through one stage (filter → gain → noise) |
| `simulate_complex_baseband()` | Run the fast analytical simulation |
| `simulate_bruteforce_rf()` | Run the full RF simulation |
| `env_to_rf_real()` | IQ upconversion to RF carrier |
| `mix_down_and_lowpass()` | RF downconversion with anti-aliasing filter |
| `moving_average_complex()` / `_real()` | Rectangular FIR low-pass filter |
| `compute_metric_complex()` / `_real()` | Calculate SNR and EVM |
| `db_to_lin()` / `lin_to_db()` | Unit conversion helpers |

### 6.2 `src/prng.c` — Random Number Generator

Implements the **xoshiro256\*\*** PRNG algorithm:

| Function | Purpose |
|----------|---------|
| `prng_seed(uint32_t)` | Initialize with a 32-bit seed (uses SplitMix64 to expand) |
| `prng_uniform()` | Random double in [0, 1) |
| `prng_gauss()` | Standard normal N(0,1) via Box-Muller transform |
| `prng_uint32()` | Random 32-bit unsigned integer |

**Why xoshiro256\*\*?**
- Extremely fast (~10 CPU cycles per number)
- Excellent statistical quality (passes BigCrush, PractRand)
- Long period: 2²⁵⁶ − 1 states before repeating
- Deterministic: same seed → same sequence every time

### 6.3 `src/stage_models.c` — CSV Configuration Loader

Reads receiver stage definitions from CSV files:

| Function | Purpose |
|----------|---------|
| `stage_models_load_csv()` | Main entry: open file, detect format, parse, validate |
| `stage_models_free()` | Free all allocated memory |
| `stage_models_get()` | Retrieve stage array for a given chain |
| `stage_chain_name()` | Convert chain ID to string name |
| `parse_chain_id()` | Flexible chain name matching |
| `map_legacy_component()` | Backward-compatible mapping for old CSV format |

**CSV format auto-detection:** The loader checks the header row for a "chain" column. If present → canonical format. If absent → legacy format (maps component names like "LNA 1" to the appropriate chain and stage).

### 6.4 `src/stage_artifacts.c` — Output Generation

Generates all CSV data files and SVG visualizations:

| Function | Purpose |
|----------|---------|
| `write_constellation_csv()` | Save I/Q coordinates to CSV |
| `write_constellation_svg()` | Draw scatter plot of I/Q constellation |
| `write_real_trace_csv()` | Save time-domain RF waveform to CSV |
| `write_trace_svg()` | Draw time-domain waveform overlay |
| `write_metrics_csv()` / `_svg()` | Stage metrics summary table |
| `write_input_budget_csv()` / `write_budget_svg()` | Link budget parameter card |
| `humanize_stage_name()` | Convert "rf_bpf_eq" → "RF BPF EQ" |
| `slugify_text()` | Convert "RF BPF EQ" → "rf_bpf_eq" (filesystem-safe) |

---

## 7. Key Data Structures

### `Complex` (sim_types.h)

```c
typedef struct { double re, im; } Complex;
```
Represents a complex number with real (In-phase) and imaginary (Quadrature) parts.

### `SimConfig` (sim_types.h)

```c
typedef struct {
    double carrier_hz;         // RF carrier frequency (default: 24 GHz)
    double symbol_rate_hz;     // Symbol rate (default: 10 Msym/s)
    double rolloff;            // Root-raised-cosine roll-off (default: 0.2)
    double input_snr_db;       // Input SNR in dB (default: 20)
    double antenna_temp_k;     // Antenna temperature in Kelvin (default: 150 K)
    double t0_k;               // Reference temperature (always 290 K)
    double rf_sample_rate_hz;  // RF sampling rate (default: 96 GHz)
    int symbols;               // Number of symbols to simulate (default: 256)
    unsigned int seed;         // PRNG seed (default: current time)
} SimConfig;
```

### `StageModel` (stage_models.h)

```c
typedef struct {
    char* name;               // Human-readable name (e.g., "lna", "rf_bpf")
    double gain_db;           // Gain in dB (+25 = amplify, -3 = attenuate)
    double nf_db;             // Noise Figure in dB (higher = noisier)
    int filter_len;           // Moving-average filter window (1 = no filter)
    int auto_gain_to_vpp;     // If 1, calculate gain to target Vpp automatically
    double target_vpp;        // Target peak-to-peak voltage
} StageModel;
```

### `StageMetric` (sim_types.h)

```c
typedef struct {
    const char* stage;        // Stage name
    const char* domain;       // Signal domain (e.g., "complex_baseband")
    double signal_power;      // Mean signal power
    double noise_power;       // Mean noise power (MSE vs. reference)
    double snr_db;            // Signal-to-Noise Ratio in dB
    double evm_pct;           // Error Vector Magnitude in %
} StageMetric;
```

### `StageModelsConfig` (stage_models.h)

```c
typedef struct {
    StageModel* chains[3];    // Three arrays of stages (one per chain)
    size_t counts[3];         // Number of stages in each chain
} StageModelsConfig;
```

---

## 8. Key Algorithms Explained

### 8.1 DVB-S2X 64-APSK Constellation Construction

Each of the 64 symbols is identified by a 6-bit label. The bits are decoded as:

```
Bit layout: [a, b, p, q, u, v]

Bits [5:4] = ab → Ring selection via Gray-code LUT
                   ab=00→Ring0, ab=01→Ring1, ab=11→Ring3, ab=10→Ring2

Bit [3] = q  ┐
Bit [2] = p  ┘→ Quadrant selection (4 quadrants via p,q combinations)

Bits [1:0] = uv → Phase position via Gray-code LUT
                   uv=00→1, uv=01→3, uv=11→7, uv=10→5

Phase angle = phi_number × π/16 (in first quadrant)
Full angle  = dvbs2x_quadrant_angle(phi, p, q)
I = radius × cos(angle)
Q = radius × sin(angle)
```

After construction, the constellation is **normalized** so that the average symbol energy `Es = 1`.

### 8.2 Friis Noise Cascade Model (T0 Reference Tracker)

The `pn_t0_track` variable tracks the reference noise power as it propagates through stages. This models the **Friis noise temperature formula**:

```
F_total = F₁ + (F₂ - 1)/G₁ + (F₃ - 1)/(G₁·G₂) + ...
```

Where `F_i` is the noise factor and `G_i` is the gain of stage `i`. The tracker ensures that noise added by later stages is appropriately scaled by the cumulative gain of all preceding stages.

### 8.3 Box-Muller Transform (Gaussian Noise Generation)

The `prng_gauss()` function uses the **Box-Muller transform** to convert two uniform random numbers into two independent Gaussian samples:

```
u₁, u₂ ~ Uniform(0, 1)
magnitude = √(-2 · ln(u₁))
angle = 2π · u₂
sample₁ = magnitude · cos(angle)    → returned immediately
sample₂ = magnitude · sin(angle)    → cached for next call
```

### 8.4 IQ Modulation / Demodulation

**Upconversion** (baseband → RF):
```
RF[t] = I[t] · cos(2πf_c·t) − Q[t] · sin(2πf_c·t)
```

**Downconversion** (RF → baseband):
```
I_raw = 2 · RF · cos(2πf_c·t)    // Multiply by cosine
Q_raw = −2 · RF · sin(2πf_c·t)   // Multiply by −sine
                                   // Factor of 2 compensates for half-power loss
Low-pass filter I_raw and Q_raw to remove 2×f_c image
```

Both use a **Numerically Controlled Oscillator (NCO)** instead of computing `cos()` and `sin()` for every sample. The NCO multiplies by a fixed rotation phasor each step, which is much faster.

### 8.5 Moving Average Filter

A simple but effective rectangular low-pass filter:

```
output[i] = (1/W) × Σ input[i-W+1 ... i]
```

Implemented efficiently with a running sum (O(n) total, not O(n×W)):
- Add `input[i]` to the running sum
- If window is full, subtract `input[i-W]`
- Output = running_sum / window_count

---

## 9. CSV Configuration System

### Canonical Format (Recommended)

```csv
chain, name, gain_db, nf_db, filter_len, auto_gain_to_vpp, target_vpp, enabled
baseband_rx, rf_bpf_eq, -2.5, 2.5, 3, 0, 0.0, 1
baseband_rx, lna, 25.0, 1.2, 1, 0, 0.0, 1
baseband_rx, mixer_downconv, -7.0, 8.0, 1, 0, 0.0, 1
baseband_rx, bb_lpf, -1.5, 1.5, 5, 0, 0.0, 1
baseband_rx, bb_amp_1vpp, 0.0, 3.0, 1, 1, 1.0, 1
rf_frontend, rf_bpf, -2.5, 2.5, 5, 0, 0.0, 1
rf_frontend, lna, 25.0, 1.2, 1, 0, 0.0, 1
rf_postmix_bb, bb_lpf, -1.5, 1.5, 5, 0, 0.0, 1
rf_postmix_bb, bb_amp_1vpp, 0.0, 3.0, 1, 1, 1.0, 1
```

### Legacy Format (Backward-Compatible)

```csv
component, gain_db, nf_db
Filter 1, -2.5, 2.5
LNA 1, 25.0, 1.2
Mixer 1, -7.0, 8.0
Filter 2, -1.5, 1.5
LNA 2, 15.0, 3.0
```

Legacy component names are automatically mapped to canonical chains via hard-coded rules in `map_legacy_component()`.

### Column Name Flexibility

Headers are normalized (case-insensitive, punctuation-stripped), so all of these are equivalent:
- `gain_db`, `Gain_dB`, `GAIN-DB`, `Gain(dB)`, `gaindb`

---

## 10. Output Files

### 10.1 Directory Layout

```
out/topology_sim_1/
├── csv/
│   ├── input_budget.csv                              — Simulation parameters
│   ├── baseband_input.csv                            — Baseband input constellation
│   ├── baseband_stage_01_rf_bpf_eq.csv              — After RF BPF EQ stage
│   ├── baseband_stage_02_lna.csv                     — After LNA stage
│   ├── ...                                           — (one per stage)
│   ├── rf_input.csv                                  — RF input waveform
│   ├── rf_stage_01_rf_bpf.csv                       — After RF BPF stage
│   ├── ...
│   ├── stage_metrics_baseband.csv                    — All baseband stage metrics
│   └── stage_metrics_rf.csv                          — All RF stage metrics
│
└── svg/
    ├── input_budget.svg                              — Link budget card
    ├── baseband_input_snr_20p00db_evm_9p50pct.svg   — Input constellation plot
    ├── baseband_stage_01_rf_bpf_eq_snr_19p85db_....svg
    ├── ...
    ├── rf_input_snr_20p00db.svg                     — RF waveform plot
    ├── rf_stage_01_rf_bpf_snr_19p85db.svg
    ├── ...
    ├── stage_metrics_baseband.svg                    — Metrics summary table
    └── stage_metrics_rf.svg
```

### 10.2 CSV File Formats

**Constellation CSV** — I/Q coordinates:
```csv
idx,ref_i,ref_q,sig_i,sig_q
0,0.707106781187,0.707106781187,0.715234521983,0.698765432101
```

**Trace CSV** — Time-domain waveform:
```csv
idx,ref,sig
0,0.234567890123,0.239876543210
```

**Metrics CSV** — Per-stage quality summary:
```csv
stage,domain,signal_power,noise_power,snr_db,evm_pct
input,complex_baseband,1.000000e+00,1.234567e-02,19.085,11.11
```

### 10.3 SVG Visualizations

All SVGs are self-contained vector graphics viewable in any web browser:
- **Constellation plots**: 980×760 px scatter plots with blue (reference) and orange (received) dots
- **Trace plots**: 1100×600 px line charts with blue (reference) and orange (received) waveforms
- **Metrics tables**: Formatted data tables with alternating row colors
- **Budget cards**: Parameter summary cards

---

## 11. How to Build and Run

### Prerequisites

- GCC or any C11-compatible compiler
- Standard C math library (`-lm`)
- No external dependencies

### Build

```bash
cd receiver_dual_sim
make          # Build the simulator
```

### Run with defaults

```bash
make run      # Run with default parameters
# or
./bin/dual_receiver_sim
```

### Run with custom parameters

```bash
./bin/dual_receiver_sim \
    --seed 42 \
    --symbols 512 \
    --snr 25 \
    --carrier 24e9 \
    --symbol-rate 10e6 \
    --stage-csv stage_models/runtime_stage_models_target16.csv \
    --stage-sim 1
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--seed <int>` | current time | PRNG seed for reproducibility |
| `--symbols <int>` | 256 | Number of symbols to simulate |
| `--symbol-rate <Hz>` | 10e6 | Symbol rate (10 Msym/s) |
| `--rf-fs <Hz>` | 96e9 | RF sampling rate (96 GHz) |
| `--carrier <Hz>` | 24e9 | Carrier frequency (24 GHz) |
| `--snr <dB>` | 20 | Input Signal-to-Noise Ratio |
| `--stage-csv <path>` | stage_models/runtime_...csv | Stage configuration file |
| `--topology-sim <1-4>` | 1 | Output slot (topology_sim_N) |

### View outputs

Open any SVG file in a web browser:
```bash
firefox out/topology_sim_1/svg/baseband_input_snr_20p00db_evm_9p50pct.svg
```

### Clean

```bash
make clean    # Remove binaries and output
```

---

## 12. Glossary

| Term | Full Name | Description |
|------|-----------|-------------|
| **64-APSK** | 64-Amplitude Phase Shift Keying | Modulation with 64 symbols in 4 rings |
| **ADC** | Analog-to-Digital Converter | Converts analog signal to digital samples |
| **AWGN** | Additive White Gaussian Noise | Standard noise model |
| **BPF** | Band-Pass Filter | Passes frequencies in a specific band, rejects others |
| **dB** | Decibel | Logarithmic ratio unit (10·log₁₀) |
| **dBm** | dB relative to 1 milliwatt | Absolute power level |
| **DVB-S2X** | Digital Video Broadcasting - Satellite 2nd gen Extended | Satellite TV standard |
| **EVM** | Error Vector Magnitude | RMS error relative to signal amplitude |
| **FIR** | Finite Impulse Response | Filter type with finite-duration response |
| **IIR** | Infinite Impulse Response | Filter type with feedback (recursive) |
| **I/Q** | In-phase / Quadrature | Two orthogonal signal components |
| **LNA** | Low-Noise Amplifier | First receiver amplifier (critical for noise performance) |
| **LO** | Local Oscillator | Signal source for mixer frequency translation |
| **LPF** | Low-Pass Filter | Passes low frequencies, rejects high frequencies |
| **MSE** | Mean Squared Error | Average squared difference between two signals |
| **NCO** | Numerically Controlled Oscillator | Digital sine/cosine generator |
| **NF** | Noise Figure | Measure of noise added by a component (dB) |
| **PRNG** | Pseudo-Random Number Generator | Deterministic algorithm producing random-looking numbers |
| **RF** | Radio Frequency | The actual radio signal domain (e.g., 24 GHz) |
| **SER** | Symbol Error Rate | Fraction of incorrectly decoded symbols |
| **SNR** | Signal-to-Noise Ratio | Ratio of signal power to noise power (dB) |
| **SPS** | Samples Per Symbol | Oversampling factor in the RF path |
| **SVG** | Scalable Vector Graphics | XML-based vector image format |
| **Vpp** | Volts peak-to-peak | Full voltage swing of a signal |
