# receiver_dual_sim

Dual-path receiver simulation workspace for DVB-S2X 64-APSK analysis.

This repository combines:
- A C simulation engine for stage-by-stage RF/baseband processing and trace artifacts.
- A MATLAB reference script for constellation-domain validation and comparison.

## Folder Structure

```text
receiver_dual_sim/
|-- Makefile
|-- README.md
|-- EXPLANATION.md
|-- bin/
|-- data/
|-- include/
|-- matlab/
|-- out/
|   |-- baseband/
|   |-- rf/
|   `-- matlab/
|-- out_sweep/
|-- scripts/
|-- src/
`-- stage_models/
```

## What Each Folder Contains

- `bin/`
  - Compiled executables (for example `dual_receiver_sim`).

- `data/`
  - Optional input datasets used by helper tools or experiments.

- `include/`
  - Public headers for simulation types and module interfaces.

- `matlab/`
  - MATLAB reference implementation and plotting scripts.
  - Main script: `sim_receiver_matlab.m`.

- `out/`
  - Generated runtime artifacts.
  - `out/baseband/`: C baseband-stage CSV/SVG outputs.
  - `out/rf/`: C RF-stage CSV/SVG outputs.
  - `out/matlab/`: MATLAB-generated figures.

- `out_sweep/`
  - Generated artifacts from design-space sweeps (batch exploration).

- `scripts/`
  - Utility scripts (automation, sweeps, helpers).

- `src/`
  - C source code for the core simulation pipeline and artifact rendering.

- `stage_models/`
  - Stage-chain CSV definitions used by C runtime.
  - Active profile: `runtime_stage_models_target16.csv`.

## Visualization Policy (Current)

- Signal/time traces are taken from the C simulation outputs because the C RF path is the runtime-faithful waveform path.
- Constellation comparisons are taken from MATLAB outputs because the MATLAB reference path applies symbol-domain alignment steps intended for clean constellation evaluation.

See `EXPLANATION.md` for full rationale and technical details.

## Build And Run (C)

From `receiver_dual_sim/`:

```bash
make
./bin/dual_receiver_sim --stage-csv stage_models/runtime_stage_models_target16.csv
```

## Run MATLAB Reference Plots

From `receiver_dual_sim/matlab/`:

```bash
matlab -batch "sim_receiver_matlab"
```

Generated figures are saved under `out/matlab/`.
