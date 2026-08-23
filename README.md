# receiver_dual_sim

20 GHz optimized satellite receiver simulator for DVB-S2X 64-APSK.
Implements full RF front-end cascade analysis (Part E), ITU-R propagation
models (Part D), and dual-path stage-by-stage signal simulation (C core with
MATLAB reference).

## Quick Start

```bash
make
./run.sh
```

Output goes to `out/rf_baseline/` and `out/realistic/` with CSV metrics
and constellation, trace, and spectrum SVGs per receiver stage.

## Folder Structure

```
receiver_dual_sim/
├── data_input/
│   ├── 20ghz/
│   │   ├── receiver.csv          ← Receiver chain: topology + datasheet specs
│   │   └── transmitter.csv       ← Transmitter chain configuration
│   ├── component_cache/          ← Digi-Key API cache
│   └── component_cache.db        ← Component database
├── src/
│   ├── main.c                    ← Orchestrator: CLI, cascade, propagation, simulation
│   ├── propagation.c             ← Part D: FSPL, rain (P.838-3), fog (P.840-9), gas (P.676-13)
│   ├── cascade.c                 ← Part E: Friis NF, IIP3, dynamic range, sensitivity
│   ├── component_catalog.c       ← Loads datasheet values from receiver.csv
│   ├── signal_chain.c            ← Per-stage signal processing (gain, noise, nonlinearity)
│   ├── sim_baseband.c            ← Complex baseband analytical path
│   ├── stage_models.c            ← CSV-driven stage chain loader
│   ├── stage_artifacts.c         ← SVG/CSV artifact generation
│   ├── phase_noise.c, iq_imbalance.c, flicker_noise.c, adc_model.c, biquad_filter.c
│   ├── constellation.c, metrics.c, cli_args.c, output_mgr.c, prng.c
├── include/                      ← Headers for all modules
├── matlab/
│   ├── sim_receiver_matlab.m     ← MATLAB reference simulation
│   ├── CascadeAnalyzer.m         ← Cascade analysis (teammate)
│   ├── propagation_losses.m      ← Propagation sweep (teammate)
│   ├── build_simulink_model.m    ← Simulink model generator
│   ├── measure_i_vpp.m, export_figure_png_svg.m, plot_trace_like_c.m
├── docs/
│   ├── part_d_propagation.md     ← Part D documentation
│   └── part_e_cascade.md         ← Part E documentation
├── scripts/
│   └── run_component_sweep.py    ← Batch sweep automation
├── Makefile, CMakeLists.txt
└── run.sh                        ← Launcher (works from anywhere, even double-click)
```

## Parts Implemented

| Part | Description | Module |
|------|-------------|--------|
| **D** | Propagation: FSPL, rain (P.838-3), fog (P.840-9), gas (P.676-13), link margin | `propagation.c` |
| **E** | Receiver cascade: Friis NF, IIP3, P1dB, dynamic range, sensitivity | `cascade.c` |
| **F** | Full receiver simulation with stage-by-stage metrics | `main.c` + `signal_chain.c` |

### Part D — Link Budget

```
EIRP (85.00 dBm)
  − FSPL (209.60 dB @ 20 GHz, 36000 km)
  − Rain attenuation (5.66 dB; specific 0.9827 dB/km, slant-path 5.76 km,
    h_R = 4 km, P.838-3 with cos(2τ) polarization term)
  − Fog/cloud attenuation (0.02 dB, P.840-9 double-Debye)
  − Gas attenuation (0.31 dB, O₂+H₂O, P.676-13 reference model,
    h_o = 6 km, h_w = 2 km)
= TOTAL propagation loss (215.59 dB)
  + Rx antenna gain (68.70 dBi, Viasat 13.5 m, aperture efficiency ~65%)
= Received power (−61.89 dBm)
  − Sensitivity (−61.80 dBm from cascade)
= LINK MARGIN (−0.09 dB) ✗ INSUFFICIENT
```

All ITU-R models implemented from the original recommendation formulas:
P.838-3 curve-fit, P.840-9 double-Debye, P.676-13 reference model.

### Part E — Cascade Analysis

```
Total Gain:      151.57 dB   (includes 67.20 dB receive-antenna row as final
                              chain stage; electronics-only ≈ 84.37 dB)
Total NF:          2.67 dB
Total IIP3:      −41.39 dBm
Output P1dB:      99.58 dBm   (reported by tool; output-referred, inflated by
                              the antenna-stage convention)
Output noise No:  61.26 dBm   (reported by tool; output-referred, inflated by
                              the antenna-stage convention)
LDR:              38.32 dB
SFDR:             32.61 dB
Sensitivity:     −61.80 dBm   = kTB (−90.96 dBm @ 290 K, 200 MHz)
                              + NF (2.67 dB) + SNR_req (26.5 dB)
                              (64-APSK, 200 MHz)
```

Component IIP3/P1dB values come from **datasheet values** in
`data_input/20ghz/receiver.csv` (LNA1 ADL8142S: OIP3=17.5 dBm,
Mixer1 HMC264LC3B: IIP3=14 dBm, etc.), overriding the runtime CSV.

## Simulation Architecture

- Front end is TRUE PASSBAND ("brute-force"): real-valued samples at
  rf_sample_rate 96 GS/s (~4.8 samples/carrier-cycle at 20 GHz); mixers
  multiply by literal cos(ωt)
- After Mix2 downconversion the chain continues as complex baseband at
  80 MS/s (decimation 1200×), matching a real superheterodyne where
  post-mixer stages sit near zero IF
- Per-stage outputs: CSV metrics plus constellation, trace, and spectrum
  SVGs under `out/rf_baseline/` and `out/realistic/`
  (layout `out/<path>/{Rx,Tx}/{csv,constellations,traces,spectrum}`)
- Gaussian noise via exact Box-Muller over xoshiro256** (the former
  "ziggurat" tables were invalid and were replaced)
- Reproducibility: pass `--seed <int>` for deterministic runs (default seed
  is time-based). OpenMP thread teams are pinned (`omp_set_dynamic(0)`, 8
  threads unless `OMP_NUM_THREADS` is set) so the per-thread PRNG stream
  partitioning is stable across machines and system load.

Defaults: carrier 20 GHz, rf_sample_rate 96 GHz, symbol rate 10 MHz,
rolloff 0.2, T_ant 91 K, T0 290 K, B 200 MHz.

## Build

```bash
make          # Uses GCC + OpenMP
make clean    # Remove binaries and output
```

Or via CMake:
```bash
mkdir build && cd build && cmake .. && make
```

## Run

```bash
./run.sh                                    # Default: RF + realistic paths
./run.sh --symbols 100 --disable-rf         # Fewer symbols, no RF
./run.sh --seed 42 --snr 25                 # Custom seed and SNR
```

Flags: `--symbols`, `--snr`, `--seed`, `--carrier`, `--symbol-rate`,
`--stage-csv`, `--enable-bb`, `--disable-rf`, `--disable-realistic`.

## Output

```
out/
├── rf_baseline/
│   └── {Rx,Tx}/
│       ├── csv/              ← Per-stage metrics (SNR, EVM, gain, noise power)
│       ├── constellations/   ← I/Q scatter plots per stage (SVG)
│       ├── traces/           ← Time-domain waveform overlays (SVG)
│       └── spectrum/         ← Spectrum plots per stage (SVG)
└── realistic/
    └── {Rx,Tx}/
        ├── csv/
        ├── constellations/
        ├── traces/
        └── spectrum/
```

Generated when MATLAB runs:
```
out/matlab/               ← MATLAB reference figures
```

## Configuration

Per-frequency folders under `data_input/<freq>/` drive everything:
- `receiver.csv` — chain topology (3 chains: baseband_rx, rf_frontend, rf_postmix_bb),
  component parameters (gain, NF, filter length), and datasheet specs (OIP3, IIP3, P1dB)
- `transmitter.csv` — transmitter chain configuration
Use `--stage-csv data_input/<freq>/receiver.csv` to select a different frequency band.

## MATLAB Reference

```bash
cd matlab && matlab -batch "sim_receiver_matlab"
```

Requires Communications Toolbox and Phased Array System Toolbox.
