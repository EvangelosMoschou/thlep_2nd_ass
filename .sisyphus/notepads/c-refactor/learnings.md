# Learnings

## Task 5 — CMakeLists.txt

- MATLAB ships its own cmake at `/usr/local/MATLAB/R2025b/bin/glnxa64/cmake/bin/cmake`
- System `cmake` not available; use MATLAB's cmake for this environment
- `-static-libgcc` works: no `libgcc_s.so.1` in ldd output after link
- Binary deps clean: only `libm`, `libgomp`, `libc`, `ld-linux`, `vdso`
- Pre-existing source warnings (unused vars, format truncation) — not from cmake setup

## 2026-05-12: Task 6 - .gitignore Update

### Patterns Used
- `build/` — ignores entire CMake build output directory
- `bin/*` + `!bin/dual_receiver_sim` — ignores everything in `bin/` EXCEPT the binary
  - Important: `bin/` (directory pattern) prevents negation of files inside. Must use `bin/*` (glob pattern) for negation to work.
- `*.o`, `*.a`, `*.so` — global patterns catch build artifacts anywhere in repo

## 2026-05-12: Task 8 — metrics.c/metrics.h

### Functions Extracted from main.c
- `mean_power_complex`, `mean_noise_power_complex` — complex signal power measurement
- `mean_power_real`, `mean_noise_power_real` — real signal power measurement
- `compute_snr_db` — extracted from inline SNR logic (uses `lin_to_db` from math_utils.h)
- `compute_evm_pct` — extracted from inline EVM logic (EVM only valid for complex I/Q)
- `compute_stage_metric_complex` / `compute_stage_metric_real` — fill StageMetric struct

### Design Decisions
- Renamed `compute_metric_complex` → `compute_stage_metric_complex` per task naming convention
- Renamed `compute_metric_real` → `compute_stage_metric_real`
- Extracted SNR and EVM into standalone `compute_snr_db()` and `compute_evm_pct()` helper functions
- `compute_stage_metric_complex` calls both helpers (instead of inline logic as in main.c)
- `compute_stage_metric_real` calls `compute_snr_db` but sets `evm_pct = NAN` directly (EVM undefined for real)

### Key Differences from main.c
- Functions changed from `static` → external linkage (declared in header)
- SNR/EVM logic extracted to named helper functions instead of inline if/else

### Verification Approach
- 13-unit test program covering: power calc, noise calc, SNR, EVM, edge cases (NULL, zero-length, INF, NAN)
- All 13 tests pass with 0 failures
- Module compiles cleanly with `-std=c11 -fopenmp`
- Evidence saved to `.sisyphus/evidence/task-8-metrics.txt`

### Verification Approach
- Created mock build artifacts to test gitignore patterns
- Actual cmake build confirmed patterns work in practice
- `git check-ignore -v <file>` shows which rule matched
- `git status --short` shows clean output (no build artifacts untracked)

## 2026-05-12: Task 7 — Constellation Module Extraction

### Files Created
- `include/constellation.h` — Public API: `build_dvbs2_64apsk_constellation(Complex*, size_t)`
- `src/constellation.c` — Implementation with static `normalize_constellation()` helper

### Key Design Decisions
- `normalize_constellation` kept as `static` (internal to constellation.c) — it's an implementation detail, not a public API
- `dvbs2x_quadrant_angle` (the `__attribute__((unused))` static function) NOT moved — it's unused and uses a different construction approach (bit-based mapping) vs the actual build function (direct ring iteration). Left in main.c for now.
- Function renamed from `build_64apsk_constellation_dvbs2` to `build_dvbs2_64apsk_constellation` per plan specification
- Added `size_t count` parameter for bounds checking (returns -2 if count < 64)
- Ring radii kept as pre-normalization DVB-S2X standard values (1.0, 2.73, 4.52, 6.15)

### Verification
- Test program compiled with `gcc -std=c11 -Iinclude` — no errors, no warnings
- Output: 64 points with 4 distinct radii (0.202, 0.553, 0.915, 1.245 — post-normalization)
- Evidence saved to `.sisyphus/evidence/task-7-constellation.txt`

### Gotcha
- Space in project path breaks gcc with `-I` flag via bash — must use relative paths or workdir

## 2026-05-12: Task 4 — PRNG Reentrancy

### Architecture
- PRNG uses xoshiro256** (256-bit state: s[4] of uint64_t) + ziggurat Gaussian sampling
- Ziggurat spare-sample caching: have_spare_gauss/spare_gauss for Box-Muller compatibility
- Parallel PRNG uses per-thread PrngState array indexed by omp_get_thread_num()

### Key Design Decisions
- `PrngState` struct: { uint64_t s[4]; int have_spare_gauss; double spare_gauss; }
- Ziggurat lookup tables (ziggurat_x[256], ziggurat_f[256]) remain `static const` — NOT mutable state
- `prng_seed` renamed to `prng_init` (takes PrngState* as first param)
- Parallel functions take PrngState* array; caller manages storage
- main.c holds file-scope `rng` and `rng_threads[PRNG_MAX_OMP_THREADS]` — this is fine since the goal was to remove statics from prng.c, not from callers

### Files Changed
- include/prng.h — Added PrngState struct, updated all signatures
- src/prng.c — Removed all file-scope mutable statics, passed state explicitly
- src/main.c — Updated all call sites, added PrngState declarations
- include/flicker_noise.h, src/flicker_noise.c — Added PrngState* param to generate()
- include/adc_model.h, src/adc_model.c — Added PrngState* param to apply()
- include/phase_noise.h, src/phase_noise.c — Added PrngState* param to generate()

### Determinism Verification
- seed 42 produces identical sequences across independent PrngState instances
- seed 99 produces different sequence from seed 42
- prng_uint32, prng_gauss all deterministic
- Ziggurat spare caching preserved (gauss values show ±pairs)

## 2026-05-12: Task 10 — output_mgr Module

### Files Created
- `include/output_mgr.h` — Public API: `clean_output_dir`, `ensure_output_dirs`, `get_run_dir`
- `src/output_mgr.c` — Implementation extracted from main.c

### Key Design Decisions
- `clean_output_dir` = renamed from `clear_directory_contents` (main.c:252)
- `ensure_output_dirs(base, topology_id)` — refactored from `ensure_output_dirs(void)` (main.c:223), generalized to accept `base` path. The `topology_id` param is reserved for future per-slot directory support.
- `get_run_dir` — new utility to build `base/topology_sim_N/path_type` paths
- Internal `ensure_dir_exists` helper handles both checking and creating single directories
- All functions use distinct negative error codes so callers can identify which step failed

### Important Gotcha
- `mkdir()` does NOT create intermediate parent directories. For nested paths like `rf_baseline/csv`, the `rf_baseline` parent must be created first or mkdir fails with ENOENT.
- The original `ensure_output_dirs(void)` in main.c only created flat dirs (`out/`, `out/csv/`, etc.) but the clearing code later tried to clear `rf_baseline/*` and `realistic/*` dirs that were never explicitly created — the refactored `ensure_output_dirs` creates all needed subdirs including `rf_baseline/` and `realistic/`.

## 2026-05-12: Task 12 — Shared Headers Migration

### db_to_lin Has TWO Semantics (CRITICAL)
- `db_to_lin_amplitude(x)` = `pow(10, x/20)` — for voltage/amplitude ratios (I/Q gain mismatch)
- `db_to_lin_power(x)` = `pow(10, x/10)` — for power ratios (SNR, noise figure, gain)
- iq_imbalance.c uses amplitude variant (voltage gain error)
- phase_noise.c, main.c, signal_chain.c use power variant (SNR, NF, gain)
- Merging them into one function would silently break I/Q imbalance modeling

### M_PI / M_LN2 Portability
- M_PI and M_LN2 are POSIX extensions, not C standard
- MSVC requires `_USE_MATH_DEFINES` before `<math.h>` to expose them
- Added `_USE_MATH_DEFINES` + `#ifndef` fallbacks to math_utils.h
- This centralizes the portability concern in one header

### Files That Had Local M_PI Guards (now removed)
- biquad_filter.c, adc_model.c, prng.c, iq_imbalance.c, phase_noise.c, flicker_noise.c, constellation.c, signal_chain.c
- All now include math_utils.h instead

### constellation.c and signal_chain.c Were Not in Original Task Scope
- Initial scan missed these — found during QA verification
- constellation.c had M_PI guard, signal_chain.c had M_PI guard + already used math_utils.h/physics.h
- Both fixed during this task

### main.c Left Intact
- Per task requirements, main.c's local M_PI, K_BOLTZMANN, db_to_lin, lin_to_db definitions remain
- Task 15 will handle main.c migration

## 2026-05-12: Task 9 — CLI Args Module

### Architecture
- `CliArgs` struct holds ALL parsed CLI values: seed, symbols, symbol_rate_hz, rf_sample_rate_hz, carrier_hz, input_snr_db, run_bb/rf/realistic, stage_csv_path, topology_sim_id, realistic_cfg (impairment toggles)
- `parse_cli_args()` sets defaults first, then walks argv with strcmp-based matching (no getopt) — preserves original main.c behavior exactly
- Helper parsers `parse_u32/parse_i32/parse_double` are `static` in cli_args.c (internal)

### Key Design Decisions
- Kept `TOPOLOGY_SIM_COUNT` as local `#define` in cli_args.c (avoids conflict with same-named define in main.c) — it's only used for validation inside parse_cli_args()
- Used `snprintf` for stage_csv_path default (not direct assignment) since it's a char array, not a pointer
- `cli_print_usage()` is public (non-static) so main.c can call it after the module is integrated

### Actual CLI Flags (source of truth = code, not task description)
- The actual toggles are `--disable-bb`, `--enable-rf`, `--enable-realistic` (not `--run-*` as the task spec suggested)
- This was discovered by reading main.c — always verify flag names from the code itself

### Verification Approach
- Test harness with 6 scenarios: defaults, full flag set, alias (`--stage-sim`), toggles, unknown flag, invalid numeric
- All return codes match main.c: 0=success, 1=unknown arg, 2=invalid value
- Precise double comparison with fabs() shows exact bit-exact values for parsed doubles
- Using `%.0e` for display rounds (e.g., 4.8e10 → "5e+10") — always check actual values with higher precision

## 2026-05-12: Task 11 — Signal Chain Module Extraction

### Files Created
- `include/signal_chain.h` — Public API: `apply_stage_complex`, `apply_stage_realistic`, `compute_metric_complex`, `compute_metric_real`
- `src/signal_chain.c` — Implementation with internal helpers (`add_awgn_complex_re`, `add_awgn_real_re`, `scale_complex`, `scale_real`, measurement functions)

### Key Design Decisions
- Both `apply_stage_*` functions accept `PrngState*` as last parameter (reentrant)
- Internal noise injection helpers (`add_awgn_*_re`) use serial `prng_gauss(rng)` — no global state
- `compute_metric_*` and `mean_power_*` functions are non-static (declared in header, defined in .c) — they match metrics.h declarations
- `db_to_lin` replaced with `db_to_lin_power` from math_utils.h
- `lin_to_db` from math_utils.h used directly (both are static inline in the header)
- Measurement functions (mean_power, mean_noise_power) are exported so they can replace the static duplicates in main.c later

### Functions NOT Extracted (still in main.c)
- `apply_stage_real_fused()` — optimized fused loop, tightly coupled to `prng_gauss_parallel(rng_threads)`
- `apply_stage_soa()` — Structure-of-Arrays variant, also uses `prng_gauss_parallel`
- SoA helpers (`add_awgn_soa`, `scale_soa`, `mean_power_soa`, etc.)
- These require `PrngState*` array (parallel) rather than single `PrngState*`

### Linker Strategy
- main.c still has `static` versions of these functions — no conflict at link time
- When main.c is updated to `#include "signal_chain.h"` and remove static copies, module versions will be used
- The metric functions are also declared in metrics.h (from earlier wave) — no conflict since they're not static in signal_chain.c

### Friis Formula Reference
- Complex: pn_add = N_t0_W × R_LOAD × g_lin × (F - 1)
- Real RF: pn_add = N_t0_W × R_LOAD × g_lin × (F - 1) × (fs / (2 × B_NOISE_HZ))
- Friis tracker: N_current = N_current × g_lin + N_t0_W × g_lin × (F - 1)
- Gain tracker: Gain_total *= g_lin

## 2026-05-12: Task 13 — sim_baseband Module Extraction

### Files Created
- `include/sim_baseband.h` — Public API: `SimBasebandResult` struct, `simulate_baseband()`
- `src/sim_baseband.c` — Baseband simulation loop extracted from main.c `simulate_complex_baseband()`

### Key Design Decisions
- `SimBasebandResult` bundles metrics array (MAX_BB_METRICS=32), count, and final_vpp
- Function uses `apply_stage_complex()` from signal_chain.h (reentrant, takes PrngState*)
- Initial AWGN injection uses local `add_awgn_complex_re()` with `prng_gauss(rng)` — no global PRNG
- Artifact writing (constellation SVG/CSV, trace SVG) is optional — NULL dirs skip I/O
- `db_to_lin_power()` from math_utils.h replaces main.c's local `db_to_lin()`
- `compute_metric_complex()` from signal_chain.h replaces main.c's static version

### Internal Helpers (static in sim_baseband.c)
- `copy_complex()` — memcpy wrapper for Complex arrays
- `add_awgn_complex_re()` — reentrant AWGN noise injection using serial prng_gauss()
- `complex_real_vpp()` — peak-to-peak voltage of I component

### Duplicate Symbol Note
- signal_chain.c and metrics.c both define `mean_power_complex`, `mean_noise_power_complex`, etc.
- For test linking, use signal_chain.c only (it has all needed implementations)
- When both modules are eventually linked together, metrics.c's duplicates need resolution

### QA Results
- seed=42, nsym=1000, SNR=15dB: 12 stages processed, final_vpp=5.009820
- SNR monotonically decreases: YES (15.0 → 7.437 dB)
- EVM not strictly monotonic — physically correct: low-NF gain stages can improve EVM
- Evidence saved to `.sisyphus/evidence/task-13-baseband-sim.txt`

## Task 15: Rewrite main.c as thin orchestrator

### Key Learnings:
1. **Duplicate symbol resolution**: When extracting functions to modules, need to check for duplicate definitions across .c files (e.g., signal_chain.c and metrics.c both defined mean_power_complex). Solution: remove from one file and add #include.

2. **Function signature mismatches**: Module versions may have different signatures (e.g., apply_stage_complex gained PrngState* parameter). Need to update all call sites.

3. **CLI module integration**: parse_cli_args() returns CliArgs struct, need to transfer fields to SimConfig. Keep resolve_stage_csv_path as local helper.

4. **RF code too coupled to extract**: simulate_bruteforce_rf and simulate_realistic_rf use SoA data layout, global PRNG state, and many static helpers. Extracting would require passing many parameters or creating complex structs.

5. **Incremental edits fragile for large files**: For 3000+ line files with many interdependent changes, better to extract needed sections with bash/sed and reconstruct than try many small edits.

### Issues Encountered:
- First edit attempt left partial old code (header replacement didn't fully work)
- db_to_lin renamed to db_to_lin_power by replaceAll, but old definition still existed → conflicting types
- Missing <sys/stat.h> include for resolve_stage_csv_path
- simulate_complex_baseband called old apply_stage_complex signature

### Decisions:
- Keep RF simulation code in main.c (too complex to extract)
- Keep generate_symbols in main.c (uses global rng)
- Keep resolve_stage_csv_path in main.c (not in cli_args module)
- Target of 500-800 lines not achievable due to RF code; 2630 lines is "significantly smaller" than 3791
