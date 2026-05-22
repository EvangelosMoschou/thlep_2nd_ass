# receiver_dual_sim

24 GHz K-band satellite receiver simulator for DVB-S2X 64-APSK.
Implements full RF front-end cascade analysis (Part E), ITU-R propagation
models (Part D), and dual-path stage-by-stage signal simulation.

## Quick Start

```bash
make
./run.sh
```

Output goes to `out/rf_baseline/` and `out/realistic/` with CSV metrics,
constellation SVGs, and time-domain trace SVGs per receiver stage.

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
EIRP (85 dBm)
  − FSPL (211.18 dB @ 24 GHz, GEO)
  − Rain attenuation (11.18 dB @ 10 mm/h)
  − Fog attenuation (0.03 dB)
  − Gas attenuation (0.44 dB)
  + Rx antenna gain (40 dBi)
= Received power (−97.83 dBm)
  − Sensitivity (−61.88 dBm from cascade)
= Margin (−35.94 dB)
```

All ITU-R models implemented from the original recommendation formulas:
P.838-3 curve-fit, P.840-9 double-Debye, P.676-13 reference model.

### Part E — Cascade Analysis

```
Total Gain:       79.36 dB
Total NF:          2.58 dB
Total IIP3:      −36.44 dBm
Sensitivity:     −61.88 dBm  (64-APSK, 200 MHz)
LDR:              42.69 dB
SFDR:             35.52 dB
```

Component IIP3/P1dB values come from **datasheet values** in
`data_input/20ghz/receiver.csv` (LNA1 ADL8142S: OIP3=17.5 dBm,
Mixer1 HMC264LC3B: IIP3=14 dBm, etc.), overriding the runtime CSV.

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
│   ├── csv/              ← Per-stage metrics (SNR, EVM, gain, noise power)
│   ├── constellations/   ← I/Q scatter plots per stage (SVG)
│   └── traces/           ← Time-domain waveform overlays (SVG)
└── realistic/
    ├── csv/
    ├── constellations/
    └── traces/
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
