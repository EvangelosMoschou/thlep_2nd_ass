# Learnings — ADC Model (Wave 1, T1)

## Date: 2026-05-11

### Code Conventions Observed
- Header guard style: `#ifndef MODULE_H / #define MODULE_H`
- Doxygen-style docstrings with @file, @brief, @param, @return (matching phase_noise.h, stage_models.h)
- Header block comment: `/* === ... === */` with PURPOSE, DESIGN, DEFAULT sections (matching phase_noise.c, prng.c)
- Config struct pattern: public fields + internal state section (filled by init)
- M_PI guard: `#ifndef M_PI / #define M_PI ...`
- Include order: module header first, then dependencies (prng.h), then stdlib (math.h, stdio.h)
- Makefile: SRC variable lists all .c files, LDFLAGS = -lm -fopenmp

### PRNG Usage
- `prng_gauss()` returns standard normal (mean=0, stddev=1)
- All randomness must use existing PRNG for determinism
- PRNG is NOT thread-safe but simulator runs single-threaded

### Build Notes
- Clean build required to pick up new source files in Makefile
- Pre-existing warning: `apply_stage_real` unused in main.c (not from our changes)
- LSP (clangd) not installed; verification via `make` only

### ADC Model Design Decisions
- Input voltage range assumed [-V_fs/2, +V_fs/2] (centered around 0)
- Quantization uses `(2^N - 1)` levels (not `2^N`) per task spec
- Jitter noise sigma: `2*pi*f_in * (V_fs/2) * sigma_jitter` — uses full-scale amplitude
- Jitter skipped when `jitter_ps=0` or `f_in=0` (no-op)
- `adc_model_free()` is no-op but provided for API symmetry

## Flicker Noise Implementation (Wave 1)

### Files Created
- `include/flicker_noise.h` — FlickerNoiseConfig struct + API declarations
- `src/flicker_noise.c` — Voss-McCartney 1/f noise algorithm

### Design Decisions
- **Algorithm**: Voss-McCartney (time-domain, stochastic summation of octave-spaced random walks)
- **PRNG**: Uses existing `prng_gauss()` for full determinism
- **Default corner_freq**: 1 kHz (typical for LNA baseband)
- **Max octaves**: 32 (covers fs/corner ratios up to ~2 billion)
- **No dynamic allocation**: All state embedded in struct (like phase_noise)

### API Pattern
Follows same init/generate/reset/free pattern as `phase_noise.h`:
- `flicker_noise_init(&cfg)` — computes octaves, sets up intervals
- `flicker_noise_generate(&cfg)` — returns one 1/f noise sample
- `flicker_noise_reset(&cfg)` — clears state for re-simulation
- `flicker_noise_free(&cfg)` — no-op (no dynamic alloc), API consistency

### Makefile Updated
Added `src/flicker_noise.c` to SRC list. Build passes clean.

### Code Style
- Matches existing C file conventions (header banners, include order, 4-space indent)
- `#ifndef M_PI` / `#ifndef M_LN2` guards (same as prng.c, phase_noise.c)
- Input validation with fprintf(stderr, ...) on error (same pattern)
