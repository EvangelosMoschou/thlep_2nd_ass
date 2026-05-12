# Realistic Receiver Simulation Enhancement — Full Realism Suite

## TL;DR

> **Quick Summary**: Add 8 realistic RF impairment models (LO phase noise, I/Q imbalance, flicker noise, real filters, AM-to-PM, ADC quantization+jitter, LO leakage, clock jitter) as a new parallel simulation path alongside the existing brute-force RF path. The existing baseband analytical path is disabled but code preserved.
>
> **Deliverables**:
> - 5 new C modules: `phase_noise.c/h`, `iq_imbalance.c/h`, `flicker_noise.c/h`, `biquad_filter.c/h`, `adc_model.c/h`
> - Extended `StageModel` with new CSV fields for impairment parameters
> - New `simulate_realistic_rf()` function in `main.c`
> - Extended `apply_stage_real_fused()` with AM-to-PM conversion
> - Updated `main()` to launch both RF and realistic paths, compare results
> - Updated CSV config with impairment parameters
>
> **Estimated Effort**: Large (8 impairment models + integration + validation)
> **Parallel Execution**: YES — 5 waves
> **Critical Path**: T1 (new modules) → T2 (extend stage processing) → T4 (realistic path integration) → T5 (main integration)

---

## Context

### Original Request
User asked: "How can we make this C implementation be more close to what a realistic system simulation looks like (eg more real noise calculation etc)"

### Interview Summary
**Key Discussions**:
- User chose **full realism suite** (all 8 identified impairments)
- User wants to **keep existing RF path** as baseline, **add new realistic path** running in parallel
- User wants to **disable baseband analytical path** but keep code for speed scalability
- Target: 64-APSK DVB-S2X at 24 GHz carrier, 10 Msym/s symbol rate

**Research Findings**:
- Codebase uses xoshiro256** PRNG with ziggurat Gaussian generation
- Stage processing order: filter → nonlinearity (P1dB/IP3) → gain → noise (Friis) → limiter
- Friis noise tracking via N_current, Gain_total, P_sig_in_W
- CSV format: chain,name,gain_db,nf_db,filter_len,is_limiter,p1db_dbm,ip3_dbm,ref,enabled
- Output: CSV data + SVG constellation/metrics diagrams
- SoA (Structure of Arrays) layout used for performance-critical paths

### Metis Review
**Identified Gaps** (addressed):
- **Individual impairment toggles**: Added per-impairment enable flags in SimConfig
- **Determinism requirement**: All new modules use existing PRNG for reproducibility
- **Output compatibility**: Realistic path produces identical CSV/SVG format as RF path
- **No adaptive compensation**: Explicitly excluded — this is impairment simulation only
- **No new visualization**: Reuse existing artifact generation
- **No channel models**: Beyond AWGN already present

---

## Work Objectives

### Core Objective
Transform the simulator from a simple thermal-noise-only model to a comprehensive system-level simulation that captures all major RF receiver impairments, enabling realistic EVM/SNR predictions for 64-APSK signals.

### Concrete Deliverables
- 5 new C source files + headers for impairment models
- Extended CSV config with new parameter columns
- `simulate_realistic_rf()` function in `main.c`
- Updated `main()` with 3-path comparison (disabled baseband, RF baseline, realistic)
- All existing visualization outputs work unchanged for both paths

### Definition of Done
- [ ] `make` succeeds with no warnings
- [ ] `./bin/dual_receiver_sim --seed 42 --symbols 1000` runs both RF and realistic paths
- [ ] Realistic path shows measurably worse EVM than RF baseline (due to added impairments)
- [ ] All constellation SVGs generated for both paths
- [ ] CSV metrics files contain impairment-specific columns
- [ ] Same seed produces identical output (determinism check)

### Must Have
- All 8 impairment models implemented and enabled in realistic path
- Per-impairment enable/disable flags in SimConfig
- Existing RF path unchanged (bit-exact with current behavior)
- Baseband path code preserved but not executed
- Deterministic: same seed → same output

### Must NOT Have (Guardrails)
- No adaptive impairment compensation (no phase noise cancellation, no I/Q correction)
- No new impairment types beyond the 8 specified
- No real-time visualization or GUI
- No different modulation schemes (stay on 64-APSK)
- No hardware-in-the-loop
- No channel models (fading, multipath) beyond existing AWGN
- No LDPC/BCH decoding (stay at physical layer)
- No changes to existing SVG generation code
- No changes to existing CSV output format (only additive columns)

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (Makefile, existing test patterns)
- **Automated tests**: Tests-after (add validation after implementation)
- **Framework**: Existing `make` build + manual validation via CLI
- **Agent-Executed QA**: ALWAYS (mandatory for all tasks)

### QA Policy
Every task includes agent-executed QA scenarios:
- **Build verification**: `make clean && make` succeeds
- **Runtime verification**: CLI execution with specific flags, check output files exist
- **Numerical verification**: Compare metrics between RF baseline and realistic path
- **Determinism verification**: Run twice with same seed, diff output

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — new impairment modules, all independent):
├── Task 1: Phase noise module (phase_noise.c/h) [deep]
├── Task 2: I/Q imbalance module (iq_imbalance.c/h) [quick]
├── Task 3: Flicker noise module (flicker_noise.c/h) [quick]
├── Task 4: Biquad filter module (biquad_filter.c/h) [unspecified-high]
├── Task 5: ADC model module (adc_model.c/h) [quick]

Wave 2 (After Wave 1 — extend existing infrastructure):
├── Task 6: Extend StageModel + CSV loader with new parameters [quick]
├── Task 7: Extend apply_stage_real_fused with AM-to-PM conversion [quick]

Wave 3 (After Wave 2 — realistic path core):
├── Task 8: simulate_realistic_rf() — main realistic path function [deep]

Wave 4 (After Wave 3 — integration):
├── Task 9: Update main() — 3-path integration + CLI flags [unspecified-high]
├── Task 10: Update Makefile + build system [quick]

Wave 5 (After Wave 4 — validation):
├── Task 11: End-to-end validation + determinism check [unspecified-high]

Wave FINAL (After ALL tasks — 4 parallel reviews, then user okay):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Real manual QA (unspecified-high)
└── Task F4: Scope fidelity check (deep)
```

### Dependency Matrix
- **1-5**: None — all independent
- **6**: None — extends existing structs
- **7**: 6 — needs extended StageModel
- **8**: 1,2,3,4,5,7 — needs all modules + extended stage processing
- **9**: 8 — needs realistic path function
- **10**: 9 — needs main.c changes
- **11**: 9,10 — needs full build + execution

### Agent Dispatch Summary
- **1**: **5** — T1 → `deep`, T2-T5 → `quick` (T4 → `unspecified-high`)
- **2**: **2** — T6 → `quick`, T7 → `quick`
- **3**: **1** — T8 → `deep`
- **4**: **2** — T9 → `unspecified-high`, T10 → `quick`
- **5**: **1** — T11 → `unspecified-high`
- **FINAL**: **4** — F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`, F4 → `deep`

---

## TODOs

- [ ] 1. **Phase Noise Module** (`include/phase_noise.h` + `src/phase_noise.c`)

  **What to do**:
  - Create `phase_noise.h` with `PhaseNoiseConfig` struct and public API
  - `PhaseNoiseConfig` fields: `white_floor_dbc_hz`, `f2_corner_hz`, `f3_corner_hz`, `f2_slope_dbc_hz`, `f3_slope_dbc_hz`
  - Implement `phase_noise_init(PhaseNoiseConfig*, uint32_t seed)` — precompute IIR filter coefficients from PSD
  - Implement `phase_noise_generate(double* output, size_t n, double fs_hz)` — generate phase noise time series
  - PSD model: L(f) = white_floor + K2/f² + K3/f³ (three-region model)
  - Use IIR filter cascade: white noise → 1/f² filter → 1/f³ filter → integrate → phase
  - Apply as complex rotation: `sig[i] *= cexp(I * phi[i])` for complex, `cos(phi[i])` for real
  - Default values for 24 GHz VCO: white_floor=-155 dBc/Hz, f2_corner=100 kHz, f3_corner=10 kHz
  - Must use existing PRNG (`prng_gauss()`) for determinism

  **Must NOT do**:
  - No closed-loop phase noise cancellation
  - No frequency-domain PSD synthesis (must be time-domain IIR)
  - No changes to existing PRNG

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Requires understanding of phase noise PSD models, IIR filter design, and integration into existing RF pipeline
  - **Skills**: None needed
  - **Skills Evaluated but Omitted**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 2, 3, 4, 5)
  - **Blocks**: Task 8 (realistic path needs this module)
  - **Blocked By**: None

  **References**:
  - `src/prng.c:prng_gauss()` — Gaussian noise source for phase noise generation
  - `src/main.c:env_to_rf_soa()` — RF upconversion where phase noise is applied
  - `src/main.c:mix_down_soa()` — Downconversion where LO phase noise also applies
  - Analog Devices MT-008: "The Relationship of Phase Noise to Jitter"
  - Mini-Circuits VCO phase noise datasheets for 24 GHz typical values

  **Acceptance Criteria**:
  - [ ] `phase_noise.h` compiles with existing codebase
  - [ ] `phase_noise.c` builds without warnings
  - [ ] `phase_noise_generate()` produces output with correct PSD shape (verify via FFT)
  - [ ] Same seed produces identical phase noise sequence

  **QA Scenarios**:
  ```
  Scenario: Phase noise generation produces correct PSD shape
    Tool: Bash (compile + run test program)
    Preconditions: Build phase_noise.c with test harness
    Steps:
      1. Compile test program that generates 1M samples of phase noise
      2. Write samples to CSV
      3. Verify PSD has -20 dB/decade slope in 1/f² region, -30 dB/decade in 1/f³ region
    Expected Result: PSD plot shows three distinct regions matching config
    Evidence: .sisyphus/evidence/task-1-psd-shape.csv

  Scenario: Determinism check — same seed produces identical output
    Tool: Bash
    Preconditions: Built phase_noise module
    Steps:
      1. Run with seed=42, save output to /tmp/pn_1.csv
      2. Run with seed=42, save output to /tmp/pn_2.csv
      3. diff /tmp/pn_1.csv /tmp/pn_2.csv
    Expected Result: diff shows no differences
    Evidence: .sisyphus/evidence/task-1-determinism.txt
  ```

  **Commit**: YES (groups with 1-5)
  - Message: `feat(sim): add phase noise module with PSD-based IIR model`
  - Files: `include/phase_noise.h`, `src/phase_noise.c`
  - Pre-commit: `make`

- [ ] 2. **I/Q Imbalance Module** (`include/iq_imbalance.h` + `src/iq_imbalance.c`)

  **What to do**:
  - Create `iq_imbalance.h` with `IQImbalanceConfig` struct
  - Fields: `gain_error_db` (typical 0.3 dB), `phase_error_deg` (typical 1.5°)
  - Implement `iq_imbalance_apply(double* re, double* im, size_t n, const IQImbalanceConfig*)`
  - Model: `I_out = I_in`, `Q_out = Q_in * 10^(-gain_error/20) * cos(phase_error) - I_in * sin(phase_error)`
  - Also implement real-signal version for RF path: separate I/Q channel gain mismatch
  - Default values from typical 24 GHz mixer datasheets

  **Must NOT do**:
  - No I/Q calibration or correction
  - No frequency-dependent imbalance (assume flat across bandwidth)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple mathematical model, straightforward implementation
  - **Skills**: None

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 3, 4, 5)
  - **Blocks**: Task 8
  - **Blocked By**: None

  **References**:
  - `src/main.c:mix_down_soa()` — Where I/Q separation happens, natural injection point
  - `src/main.c:apply_stage_soa()` — SoA processing pattern to follow

  **Acceptance Criteria**:
  - [ ] Module compiles and links
  - [ ] Applied to perfect constellation produces measurable EVM degradation
  - [ ] Gain error of 0 dB + phase error of 0° produces no change

  **QA Scenarios**:
  ```
  Scenario: Zero imbalance produces no change
    Tool: Bash
    Steps:
      1. Apply IQ imbalance with gain_error=0, phase_error=0 to known signal
      2. Compare input and output arrays
    Expected Result: Input and output are identical (diff = 0)
    Evidence: .sisyphus/evidence/task-2-zero-imbalance.txt

  Scenario: Non-zero imbalance produces constellation distortion
    Tool: Bash
    Steps:
      1. Apply gain_error=0.5dB, phase_error=2° to 64-APSK constellation
      2. Measure EVM before and after
    Expected Result: EVM increases by measurable amount (>0.5%)
    Evidence: .sisyphus/evidence/task-2-distortion.csv
  ```

  **Commit**: YES (groups with 1-5)
  - Message: `feat(sim): add I/Q imbalance impairment model`
  - Files: `include/iq_imbalance.h`, `src/iq_imbalance.c`

- [ ] 3. **Flicker Noise Module** (`include/flicker_noise.h` + `src/flicker_noise.c`)

  **What to do**:
  - Create `flicker_noise.h` with `FlickerNoiseConfig` struct
  - Fields: `corner_freq_hz` (where 1/f = white noise), `white_noise_power`
  - Implement Voss-McCartney algorithm for 1/f noise generation
  - `flicker_noise_generate(double* output, size_t n, double fs_hz, const FlickerNoiseConfig*)`
  - Alternative: IIR approximation with multiple pole-zero pairs
  - Must use existing PRNG for determinism
  - Default corner frequency: 1 kHz (typical for LNA baseband)

  **Must NOT do**:
  - No frequency-domain generation (must be time-domain)
  - No changes to white noise generation

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Well-known algorithm, straightforward implementation

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 8
  - **Blocked By**: None

  **References**:
  - `src/prng.c:prng_gauss()` — Random source for Voss-McCartney
  - `src/main.c:add_awgn_complex()` — Pattern for noise injection

  **Acceptance Criteria**:
  - [ ] Module compiles and links
  - [ ] Generated noise has 1/f PSD characteristic (verify via FFT)
  - [ ] Deterministic with same seed

  **QA Scenarios**:
  ```
  Scenario: 1/f PSD verification
    Tool: Bash
    Steps:
      1. Generate 1M samples of flicker noise
      2. Compute PSD via FFT
      3. Verify -10 dB/decade slope in 1/f region
    Expected Result: PSD shows -10 dB/decade slope below corner frequency
    Evidence: .sisyphus/evidence/task-3-psd.csv
  ```

  **Commit**: YES (groups with 1-5)
  - Message: `feat(sim): add flicker (1/f) noise module`
  - Files: `include/flicker_noise.h`, `src/flicker_noise.c`

- [ ] 4. **Biquad Filter Module** (`include/biquad_filter.h` + `src/biquad_filter.c`)

  **What to do**:
  - Create `biquad_filter.h` with `BiquadConfig` struct and `BiquadState` for filter state
  - Implement Butterworth low-pass biquad: `y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]`
  - Implement coefficient calculation from cutoff frequency and filter order
  - Support cascaded biquad sections for higher-order filters (2nd, 4th, 6th order)
  - `biquad_init(BiquadConfig*, cutoff_hz, fs_hz, order)` — compute coefficients
  - `biquad_process(BiquadState*, double* input, double* output, size_t n)` — process samples
  - Must handle both real and complex (SoA) signals
  - Replace existing moving-average filter in stage processing

  **Must NOT do**:
  - No FIR filter design (IIR only)
  - No adaptive filter coefficient changes

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Requires understanding of IIR filter design, coefficient calculation, numerical stability

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 8
  - **Blocked By**: None

  **References**:
  - `src/main.c:apply_moving_average_complex()` — Existing filter to understand replacement target
  - `src/main.c:apply_stage_real_fused()` — Where filter is called in stage processing
  - RBJ Audio EQ Cookbook (biquad coefficient formulas)

  **Acceptance Criteria**:
  - [ ] Module compiles and links
  - [ ] Butterworth LPF has -20*N dB/decade roll-off (N = order)
  - [ ] Filter is stable (no oscillation for any valid input)
  - [ ] State preservation works correctly across chunked processing

  **QA Scenarios**:
  ```
  Scenario: Butterworth roll-off verification
    Tool: Bash
    Steps:
      1. Apply 4th-order Butterworth LPF (cutoff=5 MHz) to white noise
      2. Compute output PSD
      3. Verify -80 dB/decade roll-off above cutoff
    Expected Result: PSD shows correct roll-off slope
    Evidence: .sisyphus/evidence/task-4-rolloff.csv

  Scenario: Filter stability with impulse input
    Tool: Bash
    Steps:
      1. Feed single impulse (1.0 followed by zeros) through filter
      2. Verify output decays to zero without oscillation
    Expected Result: Output decays monotonically to < 1e-10
    Evidence: .sisyphus/evidence/task-4-impulse.csv
  ```

  **Commit**: YES (groups with 1-5)
  - Message: `feat(sim): add biquad IIR filter module for realistic filter responses`
  - Files: `include/biquad_filter.h`, `src/biquad_filter.c`

- [ ] 5. **ADC Model Module** (`include/adc_model.h` + `src/adc_model.c`)

  **What to do**:
  - Create `adc_model.h` with `ADCModelConfig` struct
  - Fields: `bit_depth` (typical 12-16 bits), `full_scale_vpp` (1.0 V per assignment), `jitter_ps` (typical 0.5-2 ps)
  - Implement quantization: `y = round(x * (2^N - 1) / V_fs) * V_fs / (2^N - 1)`
  - Implement jitter-induced noise: `SNR_jitter = -20*log10(2*pi*f_in*sigma_jitter)`
  - `adc_model_apply(double* signal, size_t n, double fs_hz, double f_in_hz, const ADCModelConfig*)`
  - Quantization noise: uniform distribution in [-LSB/2, +LSB/2]
  - Jitter noise: add Gaussian noise with sigma from jitter formula
  - Default: 12-bit ADC, 1.0 Vpp full scale, 1 ps jitter

  **Must NOT do**:
  - No non-ideal ADC effects (DNL, INL, missing codes)
  - No oversampling or decimation

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple mathematical model

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 8
  - **Blocked By**: None

  **References**:
  - `src/main.c:auto_gain_to_vpp()` — Target Vpp calculation (1.0 V per assignment)
  - `src/main.c:apply_stage_real_fused()` — Where ADC model would be final stage

  **Acceptance Criteria**:
  - [ ] Module compiles and links
  - [ ] Quantization SNR matches 6.02*N + 1.76 dB formula
  - [ ] Jitter SNR matches -20*log10(2*pi*f_in*sigma) formula

  **QA Scenarios**:
  ```
  Scenario: Quantization SNR verification for 12-bit ADC
    Tool: Bash
    Steps:
      1. Feed full-scale sine wave through 12-bit ADC model
      2. Measure output SNR
    Expected Result: SNR ≈ 74 dB (6.02*12 + 1.76)
    Evidence: .sisyphus/evidence/task-5-quant-snr.txt
  ```

  **Commit**: YES (groups with 1-5)
  - Message: `feat(sim): add ADC quantization and jitter model`
  - Files: `include/adc_model.h`, `src/adc_model.c`

- [ ] 6. **Extend StageModel + CSV Loader** with new impairment parameters

  **What to do**:
  - Add new fields to `StageModel` in `include/stage_models.h`:
    - `double lo_phase_noise_dbc_hz` — LO phase noise floor (dBc/Hz)
    - `double iq_gain_error_db` — I/Q gain mismatch
    - `double iq_phase_error_deg` — I/Q phase error
    - `double am_pm_coeff` — AM-to-PM conversion coefficient (degrees per dB)
    - `int filter_type` — 0=moving-average (legacy), 1=Butterworth
    - `int filter_order` — Butterworth filter order (2, 4, 6)
  - Extend CSV format: add columns `lo_pn_dbc_hz,iq_gain_err_db,iq_phase_err_deg,am_pm_deg_per_db,filter_type,filter_order`
  - Update `stage_models_load_csv()` in `src/stage_models.c` to parse new columns
  - Maintain backward compatibility: new columns default to 0/disabled if absent
  - Add `RealisticPathConfig` struct in `sim_types.h` with global impairment toggles

  **Must NOT do**:
  - No breaking changes to existing CSV parsing
  - No removal of existing fields

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Struct extension and CSV parsing addition

  **Parallelization**:
  - **Can Run In Parallel**: YES (with Task 7)
  - **Parallel Group**: Wave 2
  - **Blocks**: Task 7, Task 8
  - **Blocked By**: None (extends existing code, no dependency on Wave 1 modules)

  **References**:
  - `include/stage_models.h:StageModel` — Struct to extend
  - `src/stage_models.c:stage_models_load_csv()` — CSV parser to extend
  - `stage_models/runtime_stage_models_target16.csv` — CSV to extend

  **Acceptance Criteria**:
  - [ ] Existing CSV files parse correctly (backward compatibility)
  - [ ] New CSV files with extended columns parse correctly
  - [ ] Default values applied when new columns absent

  **QA Scenarios**:
  ```
  Scenario: Backward compatibility — old CSV still works
    Tool: Bash
    Steps:
      1. Run simulator with existing runtime_stage_models_target16.csv
      2. Verify no parse errors, defaults applied for new fields
    Expected Result: Simulator runs successfully, new fields have default values
    Evidence: .sisyphus/evidence/task-6-backward-compat.txt
  ```

  **Commit**: YES (groups with 7)
  - Message: `feat(sim): extend StageModel and CSV loader with impairment parameters`
  - Files: `include/stage_models.h`, `src/stage_models.c`, `include/sim_types.h`

- [ ] 7. **Extend apply_stage_real_fused with AM-to-PM Conversion**

  **What to do**:
  - In `apply_stage_real_fused()` in `src/main.c`, add AM-to-PM conversion in the nonlinearity section
  - Model: `phase_shift_deg = am_pm_coeff * (P_in_dBm - P1dB_dBm)` for P_in > P1dB threshold
  - Apply phase rotation to complex signal: multiply by `cos(phi) + j*sin(phi)`
  - For real signal path: AM-to-PM manifests as cross-coupling between I/Q channels
  - Only apply when `stg->am_pm_coeff != 0`
  - Default coefficient: 5°/dB (typical for RF amplifiers near compression)

  **Must NOT do**:
  - No changes to existing P1dB/IP3 nonlinearity math
  - No changes to gain, noise, or limiter sections

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Small additive change to existing function

  **Parallelization**:
  - **Can Run In Parallel**: NO (depends on Task 6 for extended StageModel)
  - **Parallel Group**: Wave 2
  - **Blocks**: Task 8
  - **Blocked By**: Task 6

  **References**:
  - `src/main.c:apply_stage_real_fused()` — Function to modify (line ~1026)
  - `src/main.c:apply_stage_complex()` — Complex path nonlinearity section (line ~920)

  **Acceptance Criteria**:
  - [ ] AM-to-PM applies only when coefficient is non-zero
  - [ ] Phase shift increases with input power above threshold
  - [ ] No change to existing behavior when am_pm_coeff = 0

  **QA Scenarios**:
  ```
  Scenario: AM-PM disabled produces no change
    Tool: Bash
    Steps:
      1. Run stage with am_pm_coeff=0
      2. Compare output to current (pre-change) behavior
    Expected Result: Identical output
    Evidence: .sisyphus/evidence/task-7-am-pm-disabled.txt
  ```

  **Commit**: YES (groups with 6-7)
  - Message: `feat(sim): add AM-to-PM conversion to stage processing`
  - Files: `src/main.c`

- [ ] 8. **simulate_realistic_rf()** — Main realistic path function

  **What to do**:
  - Create new function in `src/main.c` with signature matching `simulate_bruteforce_rf()`
  - Signal flow:
    1. Generate symbols (same as RF path)
    2. Add antenna thermal noise (same as RF path)
    3. Pulse shape with RRC (same as RF path)
    4. **Apply LO phase noise** before upconversion (new)
    5. Upconvert to RF (same as RF path)
    6. Process through RF frontend stages with **AM-to-PM** (extended from Task 7)
    7. **Apply LO phase noise at mixer** (new — LO noise affects downconversion)
    8. Downconvert to baseband (same as RF path)
    9. **Apply I/Q imbalance** after downconversion (new)
    10. Process through post-mix BB stages with **biquad filters** (new)
    11. **Add flicker noise** to baseband stages (new)
    12. **Apply ADC model** at final stage (new)
    13. **Add LO leakage DC offset** (new)
  - Collect metrics at each stage (same format as RF path)
  - Return metrics array for comparison with RF baseline
  - Use same buffer allocation strategy (SoA layout)

  **Must NOT do**:
  - No modification of existing `simulate_bruteforce_rf()`
  - No new output formats
  - No impairment compensation or correction

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Complex integration of all 8 impairments into a coherent signal flow. Requires deep understanding of the existing RF pipeline.

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 3 (sequential after Wave 2)
  - **Blocks**: Task 9
  - **Blocked By**: Tasks 1-7

  **References**:
  - `src/main.c:simulate_bruteforce_rf()` — Template function to mirror
  - `src/main.c:apply_stage_real_fused()` — Stage processing to call
  - `src/main.c:env_to_rf_soa()` — Upconversion function
  - `src/main.c:mix_down_soa()` — Downconversion function
  - All Wave 1 modules: `phase_noise.h`, `iq_imbalance.h`, `flicker_noise.h`, `biquad_filter.h`, `adc_model.h`

  **Acceptance Criteria**:
  - [ ] Function compiles and links
  - [ ] Runs end-to-end without crashes
  - [ ] Produces metrics at each stage
  - [ ] Realistic path EVM > RF baseline EVM (impairments add degradation)
  - [ ] Deterministic with same seed

  **QA Scenarios**:
  ```
  Scenario: Realistic path runs end-to-end
    Tool: Bash
    Steps:
      1. Build simulator with all modules
      2. Run with --seed 42 --symbols 500
      3. Check output files exist in out/
    Expected Result: All CSV and SVG files generated for realistic path
    Evidence: .sisyphus/evidence/task-8-e2e-files.txt

  Scenario: Realistic path shows worse EVM than RF baseline
    Tool: Bash
    Steps:
      1. Run simulator with both paths enabled
      2. Parse final EVM from both paths' metrics CSV
      3. Compare: EVM_realistic > EVM_rf_baseline
    Expected Result: Realistic path EVM is measurably higher
    Evidence: .sisyphus/evidence/task-8-evm-comparison.txt
  ```

  **Commit**: YES
  - Message: `feat(sim): add simulate_realistic_rf() with all 8 impairments`
  - Files: `src/main.c`

- [ ] 9. **Update main() — 3-Path Integration + CLI Flags**

  **What to do**:
  - Add CLI flags: `--disable-bb` (default: true), `--enable-realistic` (default: true), `--enable-rf` (default: true)
  - In `main()`, conditionally launch paths based on flags:
    - Baseband path: disabled by default, code preserved
    - RF path: enabled by default (baseline)
    - Realistic path: enabled by default (new)
  - Add comparison output: print side-by-side metrics table
  - Generate separate output directories: `out/rf_baseline/` and `out/realistic/`
  - Add summary section: "RF Baseline vs Realistic Path Comparison"
  - Update `SimConfig` with `run_bb`, `run_rf`, `run_realistic` flags
  - Add `RealisticPathConfig` initialization with default impairment values

  **Must NOT do**:
  - No deletion of baseband path code
  - No changes to existing CLI argument parsing logic (only additions)

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Requires careful integration of multiple code paths without breaking existing behavior

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 4
  - **Blocks**: Task 10, Task 11
  - **Blocked By**: Task 8

  **References**:
  - `src/main.c:main()` — Entry point to modify (line ~2807)
  - `src/main.c:simulate_bruteforce_rf()` — RF path to call alongside realistic
  - `src/main.c:parse_cli_args()` — CLI parsing to extend

  **Acceptance Criteria**:
  - [ ] `--disable-bb` flag works (baseband path skipped)
  - [ ] `--enable-realistic` flag works
  - [ ] Both RF and realistic paths run by default
  - [ ] Output directories are separate
  - [ ] Comparison table printed to stdout

  **QA Scenarios**:
  ```
  Scenario: Default run executes both RF and realistic paths
    Tool: Bash
    Steps:
      1. Run: ./bin/dual_receiver_sim --seed 42 --symbols 500
      2. Check stdout for both path summaries
      3. Check out/rf_baseline/ and out/realistic/ directories exist
    Expected Result: Both paths complete, both output directories populated
    Evidence: .sisyphus/evidence/task-9-default-run.txt

  Scenario: Individual path flags work
    Tool: Bash
    Steps:
      1. Run: ./bin/dual_receiver_sim --seed 42 --symbols 500 --enable-rf --disable-realistic
      2. Verify only RF path runs
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 500 --disable-rf --enable-realistic
      4. Verify only realistic path runs
    Expected Result: Correct paths executed based on flags
    Evidence: .sisyphus/evidence/task-9-flags.txt
  ```

  **Commit**: YES (groups with 10)
  - Message: `feat(sim): integrate 3-path comparison in main() with CLI flags`
  - Files: `src/main.c`

- [ ] 10. **Update Makefile + Build System**

  **What to do**:
  - Add new source files to `SRCS` in `Makefile`:
    - `src/phase_noise.c`
    - `src/iq_imbalance.c`
    - `src/flicker_noise.c`
    - `src/biquad_filter.c`
    - `src/adc_model.c`
  - Add new header files to `HDRS` in `Makefile`
  - Verify `make clean && make` builds successfully
  - Add optional `make test` target for validation

  **Must NOT do**:
  - No changes to compiler flags
  - No changes to existing source file list

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple Makefile edit

  **Parallelization**:
  - **Can Run In Parallel**: NO (depends on Task 9 for complete source list)
  - **Parallel Group**: Wave 4
  - **Blocks**: Task 11
  - **Blocked By**: Task 9

  **References**:
  - `Makefile` — Build system to update

  **Acceptance Criteria**:
  - [ ] `make clean && make` succeeds
  - [ ] All new .c files compiled
  - [ ] Binary runs without missing symbol errors

  **QA Scenarios**:
  ```
  Scenario: Clean build succeeds
    Tool: Bash
    Steps:
      1. make clean
      2. make
      3. Verify bin/dual_receiver_sim exists
    Expected Result: Build succeeds, binary created
    Evidence: .sisyphus/evidence/task-10-build.txt
  ```

  **Commit**: YES (groups with 9-10)
  - Message: `build: add new impairment modules to Makefile`
  - Files: `Makefile`

- [ ] 11. **End-to-End Validation + Determinism Check**

  **What to do**:
  - Create validation script `scripts/validate_realistic.sh`
  - Run simulator with `--seed 42 --symbols 1000`
  - Capture metrics from both RF and realistic paths
  - Verify:
    1. All 8 impairments contribute measurable degradation
    2. Realistic path EVM > RF baseline EVM
    3. Determinism: run twice, diff output files
    4. No memory leaks (use valgrind if available)
    5. Performance: realistic path completes in < 5 minutes for 1000 symbols
  - Generate validation report as markdown

  **Must NOT do**:
  - No changes to simulator code
  - No new dependencies

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Requires running full simulation, parsing outputs, comparing results

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 5 (final implementation wave)
  - **Blocks**: Final Verification Wave
  - **Blocked By**: Tasks 9, 10

  **References**:
  - `out/rf_baseline/csv/` — RF baseline metrics
  - `out/realistic/csv/` — Realistic path metrics
  - `scripts/` — Existing scripts directory

  **Acceptance Criteria**:
  - [ ] Validation script runs successfully
  - [ ] All 8 impairments verified active
  - [ ] Determinism confirmed
  - [ ] Performance within bounds

  **QA Scenarios**:
  ```
  Scenario: Full validation suite passes
    Tool: Bash
    Steps:
      1. Run: bash scripts/validate_realistic.sh
      2. Check all assertions pass
    Expected Result: All validation checks pass
    Evidence: .sisyphus/evidence/task-11-validation.md

  Scenario: Determinism check across two runs
    Tool: Bash
    Steps:
      1. Run simulator with seed=42, save outputs to /tmp/run1/
      2. Run simulator with seed=42, save outputs to /tmp/run2/
      3. diff -r /tmp/run1/ /tmp/run2/
    Expected Result: No differences
    Evidence: .sisyphus/evidence/task-11-determinism.txt
  ```

  **Commit**: YES
  - Message: `test(sim): add end-to-end validation script for realistic path`
  - Files: `scripts/validate_realistic.sh`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.

- [ ] F1. **Plan Compliance Audit** — `oracle`
- [ ] F2. **Code Quality Review** — `unspecified-high`
- [ ] F3. **Real Manual QA** — `unspecified-high`
- [ ] F4. **Scope Fidelity Check** — `deep`

---

## Commit Strategy

- **Wave 1**: `feat(sim): add phase noise, IQ imbalance, flicker noise, biquad filter, ADC modules` — 10 files
- **Wave 2**: `feat(sim): extend StageModel CSV loader and add AM-PM conversion` — 3 files
- **Wave 3**: `feat(sim): add simulate_realistic_rf() path` — 1 file
- **Wave 4**: `feat(sim): integrate 3-path comparison in main()` — 2 files
- **Wave 5**: `test(sim): add end-to-end validation` — 1 file

---

## Success Criteria

### Verification Commands
```bash
make clean && make                          # Expected: clean build, no warnings
./bin/dual_receiver_sim --seed 42 --symbols 1000  # Expected: both paths run, output in out/
./bin/dual_receiver_sim --seed 42 --symbols 1000  # Run again — diff should show no changes
```

### Final Checklist
- [ ] All 8 impairments implemented in realistic path
- [ ] RF baseline unchanged (bit-exact)
- [ ] Baseband path disabled but code preserved
- [ ] Deterministic output (same seed = same results)
- [ ] All tests pass
- [ ] No new warnings from compiler
