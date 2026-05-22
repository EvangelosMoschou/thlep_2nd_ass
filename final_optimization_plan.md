# Receiver Dual Sim — Structural Refactor + Portable Optimization

## TL;DR

> **Quick Summary**: Modularize the tightly-coupled RF simulation code from main.c into dedicated modules, consolidate two redundant FFT implementations, apply portable cache/loop optimizations, and deduplicate shared logic between bruteforce and realistic RF paths. Educational angle throughout — every optimization explains WHY it works.
> 
> **Deliverables**:
> - Extracted modules: `sim_rf.c`, `sim_rf_realistic.c`, `soa_utils.c`, `spectrum.c`
> - Unified FFT module with algorithmic improvements + precomputed twiddle factors
> - Cache-tiled loops, loop unroll hints, prefetch on hot paths
> - Deduplicated RF setup (~400 lines shared between two paths)
> - Reduced SoA↔AoS conversions
> - Timing infrastructure for measuring per-path performance
> - Thin `main.c` orchestrator (~200 lines)
> 
> **Estimated Effort**: Medium
> **Parallel Execution**: YES — 4 waves
> **Critical Path**: FFT consolidation → RF extraction → Deduplication → Performance tuning

---

## Context

### Original Request
"Can we refactor it to make it run faster etc" — user wants both structural improvement AND speed, with a learning focus.

### Interview Summary
**Key Discussions**:
- Both speed AND structure: Refactor main.c modularization + portable performance optimizations
- Portable only: No CPU-specific SIMD intrinsics (AVX2/AVX-512). Auto-vectorization via `restrict` + SoA layout remains
- Agent-executed QA only: Verify by running simulator and checking output artifacts (CSV metrics, constellation SVGs, trace SVGs, SNR/EVM values)
- Educational angle: User wants to learn optimization techniques — each task includes "Why This Works" explanation

**Research Findings**:
- 10 prior optimizations already done (Ziggurat RNG, buffer preallocation, FFT convolution, SoA, OpenMP, fused stage pass, etc.)
- Current runtime ~3.76s (was ~13.78s serial — 3.7× total speedup already achieved)
- **Critical finding** (Metis): There are TWO separate radix-2 FFT implementations — `fft.c` (spectrum visualization only, NOT hot path) and `main.c:386-503` (`simple_fft`/`fft_convolve_complex`, IS hot path for RRC matched filtering). Optimizing fft.c would have ZERO impact.
- `simulate_bruteforce_rf`: ~434 lines, `simulate_realistic_rf`: ~1325 lines. First ~400 lines are identical.
- FFT uses AoS (Complex struct) internally → requires pack/unpack from SoA on every call

### Metis Review
**Identified Gaps** (addressed):
- Two-FFT confusion: Clarified — fft.c is display-only, main.c FFT is the hot-path target → Plan targets main.c FFT
- RF function sizes: Bruteforce ~434 lines, realistic ~1325 lines → Separate extraction + deduplication tasks
- SoA conversion overhead: FFT uses AoS internally, requires pack/unpack → Task to minimize conversions
- test_verify_chain.csv: Not a unit test, just test data → Confirmed: no test infrastructure

---

## Work Objectives

### Core Objective
Modularize main.c by extracting ~2000 lines of RF simulation code into dedicated, reusable modules while applying portable C optimization techniques (cache tiling, loop hints, FFT improvements, SoA conversion reduction) to improve both code structure and runtime performance.

### Concrete Deliverables
- `src/sim_rf.c` + `include/sim_rf.h` — extracted bruteforce RF simulation
- `src/sim_rf_realistic.c` + `include/sim_rf_realistic.h` — extracted realistic RF simulation
- `src/soa_utils.c` + `include/soa_utils.h` — pack/unpack/comparison utilities
- `src/spectrum.c` + `include/spectrum.h` — spectrum visualization extracted from main.c
- Unified `fft.c` — signal-processing FFT with algorithmic improvements (precomputed twiddle factors, better radix)
- Updated `Makefile` + `CMakeLists.txt` with new sources
- `src/perf_timer.c` + `include/perf_timer.h` — timing infrastructure

### Definition of Done
- [ ] `make clean && make` succeeds with zero warnings
- [ ] `./run.sh --seed 42 --symbols 3000` produces identical SNR/EVM values (±0.1%) to pre-refactor baseline
- [ ] All output artifacts present: CSV metrics, per-stage CSVs, constellation SVGs, trace SVGs, spectrum SVGs
- [ ] `main.c` reduced to ≤400 lines (thin orchestrator)
- [ ] Performance timing report shows per-path wall-clock times
- [ ] No regression: total runtime ≤ current baseline (3.76s)

### Must Have
- Identical numerical output to baseline (same seed → same SNR/EVM within floating-point tolerance)
- All 46 output files generated correctly
- Zero compiler warnings with `-Wall -Wextra -pedantic`
- New modules follow existing code style and conventions

### Must NOT Have (Guardrails)
- NO SIMD intrinsics (no `_mm256_*`, `_mm512_*`, `__m256`, `__m512`)
- NO external library dependencies (no FFTW, no kissfft, no GSL)
- NO changes to the physical model (Friis, ITU formulas must produce identical results)
- NO breaking of existing CLI interface (`--seed`, `--symbols`, `--outdir`, etc.)
- NO changes to output file format or directory structure
- NO removal of the complex baseband analytical path (must still work)

---

## Verification Strategy (MANDATORY)

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed. No exceptions.

### Test Decision
- **Infrastructure exists**: NO
- **Automated tests**: None — Agent-executed QA only
- **Framework**: N/A

### QA Policy
Every task MUST include agent-executed QA scenarios using these tools:
- **CLI**: `bash` (compile, run simulator, check exit codes)
- **Output verification**: `bash` (compare CSV metrics, check file existence, diff SNR/EVM values)
- **Git diff**: `bash` (verify refactored code produces clean, reviewable diffs)

Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

---

## Execution Strategy

### Parallel Execution Waves

> Maximize throughput by grouping independent tasks into parallel waves.
> Each wave completes before the next begins.

```
Wave 1 (Start Immediately — foundation + utilities, ALL parallel):
├── Task 1: Consolidate FFT implementations + improve algorithm [deep]
├── Task 2: Extract SoA utilities into soa_utils.c/.h [quick]
├── Task 3: Extract spectrum visualization helpers [quick]
├── Task 4: Add timing infrastructure [quick]
├── Task 5: Add portable optimization hints (unroll/prefetch macros) [quick]

Wave 2 (After Wave 1 — RF modularization, MAX PARALLEL):
├── Task 6: Extract shared RF setup from both RF functions [deep]
├── Task 7: Extract simulate_bruteforce_rf → sim_rf.c [deep]
├── Task 8: Extract simulate_realistic_rf → sim_rf_realistic.c [deep]
├── Task 9: Deduplicate common logic between bruteforce and realistic [deep]
├── Task 10: Reduce SoA↔AoS conversions at FFT/artifact boundaries [unspecified-high]
├── Task 11: Extract RRC pulse shaping helpers [quick]

Wave 3 (After Wave 2 — performance tuning, ALL parallel):
├── Task 12: Cache tiling on large SoA array loops [unspecified-high]
├── Task 13: Loop unrolling + prefetch on hot paths [unspecified-high]
├── Task 14: OpenMP schedule tuning [quick]

Wave 4 (After Wave 3 — build integration + cleanup):
├── Task 15: Update Makefile and CMakeLists.txt [quick]
├── Task 16: Thin out main.c to orchestrator [quick]

Wave FINAL (After ALL tasks — 2 parallel reviews):
├── Task F1: Baseline comparison — verify identical output
├── Task F2: Performance measurement — compare to pre-refactor baseline
```

```
Critical Path: Task 1 → Task 6/7/8 → Task 9 → Task 12 → Task 15 → Task 16 → F1-F2
Parallel Speedup: ~50% faster than sequential
Max Concurrent: 6 (Wave 2)
```

### Agent Dispatch Summary

- **Wave 1**: **5** — T1 → `deep`, T2 → `quick`, T3 → `quick`, T4 → `quick`, T5 → `quick`
- **Wave 2**: **6** — T6 → `deep`, T7 → `deep`, T8 → `deep`, T9 → `deep`, T10 → `unspecified-high`, T11 → `quick`
- **Wave 3**: **3** — T12 → `unspecified-high`, T13 → `unspecified-high`, T14 → `quick`
- **Wave 4**: **2** — T15 → `quick`, T16 → `quick`
- **FINAL**: **2** — F1 → `unspecified-high`, F2 → `unspecified-high`

---

## TODOs

> Implementation + Verification = ONE Task. Never separate.
> EVERY task MUST have: Recommended Agent Profile + Parallelization info + QA Scenarios.
> **A task WITHOUT QA Scenarios is INCOMPLETE. No exceptions.**

- [ ] 1. Consolidate FFT implementations + improve algorithm

  **What to do**:
  - Locate `simple_fft()`, `simple_ifft()`, `fft_convolve_complex()`, `next_pow2()`, `bitrev()` in `main.c` (lines ~386–503). These are the signal-processing FFT functions used for RRC matched-filter convolution in `synchronize_and_downsample()` — they ARE in the hot path.
  - Move them from `main.c` into `fft.c` (currently only has display-only `radix2_fft_inplace()` + `fft_spectrum_dB()`).
  - Add corresponding declarations to `fft.h`.
  - **Algorithm improvement**: Precompute twiddle factors once (statically or on first call) instead of computing `cos(2π*k/N)` and `sin(2π*k/N)` on every FFT call. Store in global/file-scope arrays of size MAX_FFT_SIZE.
  - **Algorithm improvement**: Replace naive radix-2 with in-place radix-2 Cooley-Tukey using precomputed twiddle factors (avoids recomputing cos/sin in the butterfly loops).
  - Add `fft_init()` to precompute twiddle factors up to a configurable max size.
  - Update `fft_convolve_complex()` to call the improved FFT.
  - Update `main.c` to call the new `fft.h` API instead of the local `simple_fft`/`fft_convolve_complex`.
  - **Warning**: Do NOT touch `radix2_fft_inplace()` or `fft_spectrum_dB()` in fft.c — those are for spectrum visualization only. Keep them working. The new functions coexist alongside them.

  **Why This Works (Educational)**:
  - **Twiddle factor precomputation**: Every FFT call re-computes `cos(2πk/N)` and `sin(2πk/N)` for each butterfly stage. For a 1024-point FFT called thousands of times, that's `1024 * log2(1024) * num_calls` trig function calls. Trig functions are ~50-100 CPU cycles each. Precomputing once reduces this to table lookups (~1-3 cycles). This is a textbook "precompute what doesn't change" optimization.
  - **Consolidation**: Having two separate FFT implementations in one codebase is a maintenance hazard. Unifying them ensures improvements benefit all callers and eliminates confusion.

  **Must NOT do**:
  - Do NOT use external FFT libraries (FFTW, kissfft, etc.)
  - Do NOT use SIMD intrinsics
  - Do NOT change the FFT algorithm's numerical output (must be bit-identical or within floating-point epsilon)

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Requires understanding both existing FFT implementations, careful algorithm modification, and ensuring numerical correctness
  - **Skills**: []
  - **Skills Evaluated but Omitted**:
    - None needed — this is pure C algorithmic work

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 2, 3, 4, 5)
  - **Blocks**: Tasks 6, 7, 8 (RF extraction needs the new fft.h API)
  - **Blocked By**: None (can start immediately)

  **References** (CRITICAL):
  - **FFT (hot-path) to extract**: `src/main.c:386-503` — `simple_fft()`, `simple_ifft()`, `fft_convolve_complex()`, `next_pow2()`, `bitrev()` — these are the functions that must be moved
  - **FFT (display-only) to leave untouched**: `src/fft.c` — `radix2_fft_inplace()`, `fft_spectrum_dB()` — keep these, they're for spectrum SVG generation
  - **FFT header**: `include/fft.h` — current declarations; will be extended
  - **Call site**: `src/main.c` — `synchronize_and_downsample()` — where `fft_convolve_complex()` is called; must update to new API
  - **OPTIMIZATIONS.md:106-143** — Optimization 4 documents the FFT convolution rationale and original code patterns

  **Why Each Reference**:
  - `main.c:386-503`: The exact code to extract — don't miss any helper functions
  - `fft.c`: Shows existing FFT API style to match (parameters, naming, comments)
  - `main.c:synchronize_and_downsample`: Only call site — must work after refactor
  - `OPTIMIZATIONS.md:4`: Documents why FFT convolution was added, confirms it's in the hot path

  **Acceptance Criteria**:
  - [ ] `simple_fft`, `simple_ifft`, `fft_convolve_complex`, `next_pow2`, `bitrev` removed from `main.c`
  - [ ] New functions in `fft.c` match existing code style
  - [ ] `fft.h` updated with new declarations
  - [ ] `make` compiles with zero warnings

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: FFT consolidation compiles and produces identical output
    Tool: Bash
    Preconditions: Clean build directory, baseline output exists
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings in output
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Run: diff <(grep "snr_db\|evm_pct" out/rf_baseline/csv/stage_metrics.csv) <(grep "snr_db\|evm_pct" baseline_out/rf_baseline/csv/stage_metrics.csv)
      6. Assert: no differences (or differences ≤ 0.01% on numeric values)
    Expected Result: Identical SNR/EVM values to baseline
    Failure Indicators: diff shows numeric differences >0.01%, compile errors, missing output files
    Evidence: .sisyphus/evidence/task-1-fft-consolidation.txt
  ```

  **Commit**: YES (groups with Wave 2 tasks)
  - Message: `refactor: consolidate FFT implementations into fft.c with precomputed twiddle factors`
  - Files: `src/fft.c`, `include/fft.h`, `src/main.c`

- [ ] 2. Extract SoA utilities into soa_utils.c/.h

  **What to do**:
  - Create `include/soa_utils.h` and `src/soa_utils.c`.
  - Move `pack_complex()`, `unpack_complex()` from `main.c` into `soa_utils.c`.
  - Add any other SoA↔AoS conversion helpers found in the codebase.
  - Add helper functions for SoA array operations: `soa_copy()`, `soa_scale()`, `soa_zero()` — common patterns currently inlined.
  - Ensure all moved functions are `static inline` where appropriate (small functions) or regular functions (larger ones).
  - Update `main.c` to `#include "soa_utils.h"` and remove local definitions.
  - **Educational**: Document in comments why SoA layout is faster (cache-line utilization, SIMD auto-vectorization) vs AoS.

  **Why This Works (Educational)**:
  - **Locality of reference**: When you process `re[0..N]` then `im[0..N]` separately, each array is contiguous in memory. The CPU's hardware prefetcher can predict these sequential accesses perfectly. With interleaved AoS `[re0,im0,re1,im1,...]`, processing just the real parts means loading every other 8 bytes — half the cache line is wasted on im values you're not using yet.

  **Must NOT do**:
  - Do NOT change function signatures in ways that break callers
  - Do NOT inline large functions (keep them in .c files)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Straightforward extraction of well-understood utility functions
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 3, 4, 5)
  - **Blocks**: Tasks 6, 7, 8 (RF extraction needs soa_utils)
  - **Blocked By**: None

  **References** (CRITICAL):
  - **pack/unpack functions**: `src/main.c` — search for `pack_complex`, `unpack_complex` — these are the extraction targets
  - **Existing SoA usage**: `OPTIMIZATIONS.md:193-248` — Optimization 6 documents all SoA functions and why some buffers stay Complex
  - **Call sites in main.c**: `simulate_bruteforce_rf()` and `simulate_realistic_rf()` — all pack/unpack callers

  **Why Each Reference**:
  - `OPTIMIZATIONS.md:6`: Shows which buffers are SoA vs Complex — critical for understanding the conversion boundaries
  - `main.c` call sites: Every pack/unpack call must still compile after extraction

  **Acceptance Criteria**:
  - [ ] `soa_utils.h` and `soa_utils.c` created with proper header guards
  - [ ] All pack/unpack functions removed from `main.c`
  - [ ] `make` compiles with zero warnings

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: SoA utilities compile and don't break simulation
    Tool: Bash
    Preconditions: Task 1 completed, baseline output exists
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Assert: all output files present (ls out/rf_baseline/constellations/*.svg | wc -l → 13)
    Expected Result: All output files generated, no compile errors
    Failure Indicators: Undefined symbol errors, missing output files, wrong number of artifacts
    Evidence: .sisyphus/evidence/task-2-soa-utils.txt
  ```

  **Commit**: YES (groups with Wave 1)
  - Message: `refactor: extract SoA utilities into soa_utils.c`
  - Files: `src/soa_utils.c`, `include/soa_utils.h`, `src/main.c`

- [ ] 3. Extract spectrum visualization helpers from main.c

  **What to do**:
  - Locate `write_stage_spectrum()` in `main.c` (lines ~42-75) — generates spectrum SVG files for RF stages.
  - Create `include/spectrum.h` and `src/spectrum.c`.
  - Move `write_stage_spectrum()` and its helper functions into `spectrum.c`.
  - Include `fft.h` (uses `fft_spectrum_dB` for the actual FFT computation).
  - Update `main.c` to `#include "spectrum.h"`.
  - This is a code organization task — no algorithmic changes.

  **Why This Works (Educational)**:
  - **Separation of concerns**: Spectrum visualization is an output concern, not simulation logic. Moving it out makes main.c focus purely on orchestration. This is the Single Responsibility Principle applied to C modules — each .c file should do one thing.

  **Must NOT do**:
  - Do NOT change the spectrum visualization algorithm
  - Do NOT change SVG output format

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Pure extraction of a well-defined helper with no algorithmic changes
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 4, 5)
  - **Blocks**: None directly
  - **Blocked By**: None

  **References** (CRITICAL):
  - **Function to extract**: `src/main.c:42-75` — `write_stage_spectrum()` — exact code to move
  - **Dependencies**: `include/fft.h` — `fft_spectrum_dB()` — used by this function
  - **Header**: `include/stage_artifacts.h` — `humanize_stage_name()`, `write_spectrum_svg()` — also used

  **Why Each Reference**:
  - `main.c:42-75`: The exact function — don't miss the helper logic for frequency bin decimation
  - `fft.h` and `stage_artifacts.h`: Must include these in the new spectrum.h

  **Acceptance Criteria**:
  - [ ] `spectrum.h` and `spectrum.c` created
  - [ ] `write_stage_spectrum()` removed from `main.c`
  - [ ] `make` compiles with zero warnings

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Spectrum extraction compiles and produces spectrum SVGs
    Tool: Bash
    Preconditions: Clean build, baseline output exists
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Assert: spectrum SVG files exist (ls out/rf_baseline/*_spectrum.svg | wc -l should match baseline count)
    Expected Result: All spectrum SVGs generated, no compile errors
    Failure Indicators: Missing spectrum SVGs, compile errors about undefined symbols
    Evidence: .sisyphus/evidence/task-3-spectrum.txt
  ```

  **Commit**: YES (groups with Wave 1)
  - Message: `refactor: extract spectrum visualization into spectrum.c`
  - Files: `src/spectrum.c`, `include/spectrum.h`, `src/main.c`

- [ ] 4. Add timing infrastructure (perf_timer.c/.h)

  **What to do**:
  - Create `include/perf_timer.h` and `src/perf_timer.c`.
  - Implement `perf_timer_start()` and `perf_timer_stop()` using `clock_gettime(CLOCK_MONOTONIC, ...)` for wall-clock timing.
  - Implement `perf_timer_elapsed_ms()` to return elapsed milliseconds.
  - Implement `perf_timer_report()` that prints formatted timing: path name, elapsed ms, symbols processed, symbols/second throughput.
  - Add timing calls in `main.c` around each simulation path: complex baseband, RF bruteforce, RF realistic.
  - Print timing summary at end of simulation run (before exit).
  - **Educational**: Document the difference between wall-clock time, CPU time, and why `CLOCK_MONOTONIC` is preferred over `clock()` for benchmarking.

  **Why This Works (Educational)**:
  - **You can't optimize what you don't measure**: Before applying any performance optimization, you need a baseline. This infrastructure makes every run self-benchmarking. The throughput metric (symbols/sec) is hardware-independent — it lets you compare performance across different machines or after code changes.
  - **CLOCK_MONOTONIC vs clock()**: `clock()` measures CPU time (sum of all threads), which is misleading for OpenMP code. `CLOCK_MONOTONIC` measures real wall-clock time — what the user actually experiences.

  **Must NOT do**:
  - Do NOT add timing overhead to hot loops (only around top-level path calls)
  - Do NOT change existing output format — timing report is additive, not replacement

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple C utility with well-known POSIX API
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 3, 5)
  - **Blocks**: None directly (but used in F1/F2 verification)
  - **Blocked By**: None

  **References** (CRITICAL):
  - **OpenMP timing reference**: `src/main.c` — search for existing timing (if any) using `omp_get_wtime` — match existing style
  - **POSIX timing**: `man clock_gettime` — `CLOCK_MONOTONIC` usage pattern
  - **Existing print patterns**: `src/main.c` — how `fprintf(stderr, ...)` or `printf(...)` is used for status output

  **Why Each Reference**:
  - `main.c` timing: If there's existing `omp_get_wtime()` usage, wrap it with perf_timer for consistency
  - `main.c` print style: Match existing output formatting conventions

  **Acceptance Criteria**:
  - [ ] `perf_timer.h` and `perf_timer.c` created
  - [ ] `make` compiles with zero warnings
  - [ ] Running simulator prints timing summary at end

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Timing infrastructure works and reports per-path times
    Tool: Bash
    Preconditions: Clean build
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000 2>&1 | tee /tmp/perf_output.txt
      4. Assert: output contains "complex baseband" or "bb" timing line
      5. Assert: output contains "rf baseline" or "bruteforce" timing line
      6. Assert: output contains "rf realistic" or "realistic" timing line
      7. Assert: all timing values are positive numbers
    Expected Result: Three timing lines with positive ms values in output
    Failure Indicators: Missing timing lines, negative or zero times, compile errors
    Evidence: .sisyphus/evidence/task-4-perf-timer.txt
  ```

  **Commit**: YES (groups with Wave 1)
  - Message: `feat: add timing infrastructure for per-path performance measurement`
  - Files: `src/perf_timer.c`, `include/perf_timer.h`, `src/main.c`

- [ ] 5. Add portable optimization hint macros to math_utils.h

  **What to do**:
  - Add `UNROLL_HINT(n)` macro that expands to `#pragma GCC unroll n` (GCC/Clang) or no-op on other compilers.
  - Add `PREFETCH_R(ptr)` and `PREFETCH_W(ptr)` macros using `__builtin_prefetch(ptr, 0, 3)` (read, high temporal locality) and `__builtin_prefetch(ptr, 1, 3)` (write).
  - Add `LIKELY(cond)` and `UNLIKELY(cond)` macros using `__builtin_expect(cond, 1)` and `__builtin_expect(cond, 0)` for branch prediction hints.
  - Add `ALWAYS_INLINE` macro using `__attribute__((always_inline)) inline`.
  - Add `ASSUME_ALIGNED(ptr, N)` macro using `__builtin_assume_aligned(ptr, N)` for the 64-byte-aligned SoA buffers.
  - Document each macro with what it does and when to use it (educational angle).
  - **Do NOT apply these macros anywhere yet** — just make them available. Tasks 12–13 will apply them.

  **Why This Works (Educational)**:
  - **`__builtin_prefetch`**: Tells the CPU to start loading data into cache before you need it. Useful when you know the access pattern (e.g., sequential scan). The CPU can hide memory latency by overlapping prefetch with computation.
  - **`__builtin_expect`**: Tells the branch predictor which path is more likely. Reduces pipeline stalls from mispredicted branches in hot loops.
  - **`__builtin_assume_aligned`**: Tells the compiler a pointer is aligned, enabling aligned SIMD loads (`movapd` vs `movupd`) even without explicit intrinsics.

  **Must NOT do**:
  - Do NOT apply macros to any existing code — this task is macro definitions only
  - Do NOT use SIMD intrinsics (the macros are purely compiler hints, not intrinsics)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple header-only macro definitions with documentation
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 3, 4)
  - **Blocks**: Tasks 12, 13 (which apply these macros to hot paths)
  - **Blocked By**: None

  **References** (CRITICAL):
  - **Existing macros**: `include/math_utils.h` — existing inline functions and `#define` patterns — follow the same style
  - **GCC builtins**: `__builtin_prefetch`, `__builtin_expect`, `__builtin_assume_aligned` — GCC documentation
  - **Alignment usage**: `src/main.c` — `ALIGNMENT 64u`, `alloc_aligned()` — the aligned allocation that these macros complement

  **Why Each Reference**:
  - `math_utils.h`: Match existing macro naming convention (lowercase with underscores, descriptive names)
  - `main.c:ALIGNMENT`: The `ASSUME_ALIGNED` macro should reference 64 (matching the existing alignment)

  **Acceptance Criteria**:
  - [ ] All macros added to `math_utils.h` with documentation comments
  - [ ] `make` compiles with zero warnings (macros unused is fine)
  - [ ] Each macro has a comment explaining its purpose and "when to use"

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Optimization macros compile without errors
    Tool: Bash
    Preconditions: Clean build
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0 (macros don't break existing code)
    Expected Result: Clean compile, simulation runs normally
    Failure Indicators: Compiler warnings about unused macros (acceptable), compile errors (unacceptable)
    Evidence: .sisyphus/evidence/task-5-macros.txt
  ```

  **Commit**: YES (groups with Wave 1)
  - Message: `feat: add portable optimization hint macros (unroll, prefetch, likely, aligned)`
  - Files: `include/math_utils.h`

- [ ] 6. Extract shared RF setup into rf_setup.c/.h

  **What to do**:
  - Both `simulate_bruteforce_rf()` and `simulate_realistic_rf()` in `main.c` start with ~400 lines of nearly identical code: buffer allocation (SoA arrays for env, rf, bb), SoA-to-Complex allocation, config extraction, envelope computation, constellation template construction.
  - Create `include/rf_setup.h` and `src/rf_setup.c`.
  - Define a struct `RfSimBuffers` that holds all allocated buffers (env_re/im, rf_ref/sig, bb_ref_re/im, temp buffers, Complex symbol buffers, etc.) and their sizes.
  - Define a struct `RfSimParams` that holds derived simulation parameters (sps, fs_hz, nrf, nbb, etc.).
  - Implement `rf_setup_allocate(RfSimBuffers *bufs, const SimConfig *cfg, const StageModelsConfig *stage_cfg)` — allocates all buffers.
  - Implement `rf_setup_free(RfSimBuffers *bufs)` — frees all buffers.
  - Implement `rf_setup_compute_params(RfSimParams *params, const SimConfig *cfg, const StageModelsConfig *stage_cfg)` — computes all derived parameters.
  - Implement `rf_setup_build_envelope(RfSimBuffers *bufs, const Complex *tx_symbols, size_t nsym, const SimConfig *cfg, const RfSimParams *params)` — computes the SoA RF envelope from symbols (RRC pulse shaping + upconversion prep).
  - Update both RF functions to use `RfSimBuffers` and `RfSimParams` instead of local variables.
  - **Educational**: Document why this deduplication matters — "Don't Repeat Yourself" principle applied to performance-critical code.

  **Why This Works (Educational)**:
  - **DRY principle for systems code**: When two functions share 400 lines of identical setup, any bug fix or optimization to the setup must be applied twice. This is how codebases accumulate subtle differences between "identical" paths. Extracting shared setup into one location ensures both paths benefit from every improvement.
  - **Struct-based buffer management**: Grouping all buffers into a struct makes ownership clear (one alloc, one free), prevents leaks, and makes it obvious what each function needs.

  **Must NOT do**:
  - Do NOT change buffer sizes or allocation strategy
  - Do NOT change the numerical values in parameter computation
  - Do NOT break the existing `alloc_aligned()` pattern — `rf_setup_allocate` must use `ALLOC_ALIGNED_D`/`ALLOC_ALIGNED_C` macros

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Requires deep understanding of both RF functions' initialization, careful extraction without breaking subtle dependencies
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 7, 8, 9, 10, 11)
  - **Blocks**: Tasks 7, 8 (RF extraction uses RfSimBuffers/RfSimParams)
  - **Blocked By**: Tasks 1, 2 (needs fft.h and soa_utils.h APIs)

  **References** (CRITICAL):
  - **Bruteforce setup section**: `src/main.c` — `simulate_bruteforce_rf()` — first ~200 lines (buffer allocation + config + envelope)
  - **Realistic setup section**: `src/main.c` — `simulate_realistic_rf()` — first ~250 lines (larger because adds impairment buffers)
  - **Buffer types**: `OPTIMIZATIONS.md:214-238` — Optimization 6/7 documents which buffers are SoA vs Complex and why
  - **Allocation macros**: `src/main.c:116-126` — `alloc_aligned()`, `ALLOC_ALIGNED_D`, `ALLOC_ALIGNED_C` — must use these
  - **Header includes needed**: `include/sim_types.h` (SimConfig, Complex), `include/stage_models.h` (StageModelsConfig), `include/prng.h` (PrngState)

  **Why Each Reference**:
  - Both RF functions' first sections: The exact code to deduplicate — must capture ALL allocations from BOTH functions
  - OPTIMIZATIONS.md:6/7: Documents buffer layout decisions — critical for getting the struct fields right
  - main.c alloc macros: Must reuse, not recreate

  **Acceptance Criteria**:
  - [ ] `rf_setup.h` and `rf_setup.c` created
  - [ ] Both RF functions' setup replaced with calls to `rf_setup_*` functions
  - [ ] `make` compiles with zero warnings
  - [ ] Simulation produces identical output

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Shared RF setup produces identical results to baseline
    Tool: Bash
    Preconditions: Baseline output saved, Tasks 1-2 completed
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Run: diff out/rf_baseline/csv/stage_metrics.csv baseline_out/rf_baseline/csv/stage_metrics.csv
      6. Assert: no differences (or floating-point differences ≤ 0.01%)
    Expected Result: Identical metrics to baseline
    Failure Indicators: diff shows changes, missing output files, segfault (buffer size mismatch)
    Evidence: .sisyphus/evidence/task-6-rf-setup.txt
  ```

  **Commit**: YES (groups with Wave 2)
  - Message: `refactor: extract shared RF setup into rf_setup.c`
  - Files: `src/rf_setup.c`, `include/rf_setup.h`, `src/main.c`

- [ ] 7. Extract simulate_bruteforce_rf → sim_rf.c

  **What to do**:
  - Create `include/sim_rf.h` (extends existing header — currently has SoA function declarations) and `src/sim_rf.c`.
  - Move the full `simulate_bruteforce_rf()` function body from `main.c` into `sim_rf.c`.
  - The function signature stays the same as declared in `include/sim_rf.h`.
  - Update `main.c` to call `simulate_bruteforce_rf()` via the header instead of having the function inline.
  - Ensure all includes in the new file are correct (may need to add some that were previously included transitively via main.c).
  - **No algorithmic changes** — pure extraction.

  **Why This Works (Educational)**:
  - **Module boundaries = cognitive boundaries**: When a function is 400+ lines, understanding it requires holding the entire function in your head. Extracting it into its own file lets you reason about it in isolation. This is the fundamental unit of software architecture — the file/module boundary.
  - **Compilation units**: Each .c file is compiled independently. This means faster incremental builds (changing sim_rf.c only recompiles sim_rf.o, not main.o). For large projects, this can cut build times by 90%.

  **Must NOT do**:
  - Do NOT change function behavior or algorithm
  - Do NOT change the function signature
  - Do NOT miss any `static` helper functions used by the bruteforce path

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Large function extraction with complex dependencies — must verify all includes and static helpers
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 6, 8, 9, 10, 11)
  - **Blocks**: Tasks 12, 13 (performance tuning targets sim_rf.c)
  - **Blocked By**: Tasks 1 (fft.h), 2 (soa_utils.h), 6 (rf_setup.h)

  **References** (CRITICAL):
  - **Function to extract**: `src/main.c` — `simulate_bruteforce_rf()` — full function (~434 lines including internal helpers)
  - **Existing header**: `include/sim_rf.h` — already has function declaration — verify it matches
  - **Helper functions used**: `src/main.c` — `write_stage_spectrum()`, `synchronize_and_downsample()`, SoA functions — may need additional includes
  - **OPTIMIZATIONS.md:9** — documents OpenMP parallelization in this function

  **Why Each Reference**:
  - `main.c:simulate_bruteforce_rf`: The exact code — don't miss internal helper functions
  - `sim_rf.h`: Check existing declaration for signature match
  - OPTIMIZATIONS.md:9: Shows which sections are parallelized — important context

  **Acceptance Criteria**:
  - [ ] `sim_rf.c` created with full `simulate_bruteforce_rf()` implementation
  - [ ] `main.c` calls via `sim_rf.h` header
  - [ ] `make` compiles with zero warnings
  - [ ] Identical output to baseline

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Extracted bruteforce RF function produces identical output
    Tool: Bash
    Preconditions: Tasks 1, 2, 6 completed, baseline output saved
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Run: diff -r out/rf_baseline/ baseline_out/rf_baseline/ 2>&1
      6. Assert: no differences (or only floating-point in CSV values ≤ 0.01%)
    Expected Result: Identical output in rf_baseline/ directory
    Failure Indicators: diff shows file-level differences, missing files, compile errors
    Evidence: .sisyphus/evidence/task-7-sim-rf.txt
  ```

  **Commit**: YES (groups with Wave 2)
  - Message: `refactor: extract simulate_bruteforce_rf into sim_rf.c`
  - Files: `src/sim_rf.c`, `include/sim_rf.h`, `src/main.c`

- [ ] 8. Extract simulate_realistic_rf → sim_rf_realistic.c

  **What to do**:
  - Create `include/sim_rf_realistic.h` and `src/sim_rf_realistic.c`.
  - Move the full `simulate_realistic_rf()` function body from `main.c` into `sim_rf_realistic.c`.
  - This function is ~1325 lines — much larger than bruteforce because it includes all impairment models (phase noise, I/Q imbalance, flicker noise, ADC, biquad filters, LO leakage).
  - Ensure all impairment module headers are included: `phase_noise.h`, `iq_imbalance.h`, `flicker_noise.h`, `adc_model.h`, `biquad_filter.h`.
  - Update `main.c` to call `simulate_realistic_rf()` via the new header.
  - **No algorithmic changes** — pure extraction.

  **Why This Works (Educational)**:
  - **Same principle as Task 7, but more critical here**: This function is 3× larger than the bruteforce path. At 1325 lines, it's impossible to fully understand without extraction. The impairment models are complex and deserve their own context.

  **Must NOT do**:
  - Do NOT change any impairment model behavior
  - Do NOT change function signature
  - Do NOT miss any `static` helper functions used only by realistic path

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Very large function extraction, complex dependency chain across 5 impairment modules
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 6, 7, 9, 10, 11)
  - **Blocks**: Tasks 12, 13
  - **Blocked By**: Tasks 1, 2, 6

  **References** (CRITICAL):
  - **Function to extract**: `src/main.c` — `simulate_realistic_rf()` — full function (~1325 lines)
  - **Impairment headers**: `include/phase_noise.h`, `include/iq_imbalance.h`, `include/flicker_noise.h`, `include/adc_model.h`, `include/biquad_filter.h`
  - **RealisticPathConfig**: `include/sim_types.h:56-74` — the config struct controlling which impairments are enabled

  **Why Each Reference**:
  - Each impairment header: Must be included in the new file — they're currently included in main.c
  - `RealisticPathConfig`: The struct that controls which impairments run — important context

  **Acceptance Criteria**:
  - [ ] `sim_rf_realistic.c` created with full function
  - [ ] `main.c` calls via new header
  - [ ] `make` compiles with zero warnings
  - [ ] Identical realistic output to baseline

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Extracted realistic RF function produces identical output
    Tool: Bash
    Preconditions: Tasks 1, 2, 6 completed, baseline output saved
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Run: diff -r out/realistic/ baseline_out/realistic/ 2>&1
      6. Assert: no differences (or only floating-point ≤ 0.01%)
    Expected Result: Identical output in realistic/ directory
    Failure Indicators: diff shows differences, missing files, compile/link errors
    Evidence: .sisyphus/evidence/task-8-sim-rf-realistic.txt
  ```

  **Commit**: YES (groups with Wave 2)
  - Message: `refactor: extract simulate_realistic_rf into sim_rf_realistic.c`
  - Files: `src/sim_rf_realistic.c`, `include/sim_rf_realistic.h`, `src/main.c`

- [ ] 9. Deduplicate common logic between bruteforce and realistic RF paths

  **What to do**:
  - With both RF functions now in separate files (Tasks 7–8), compare them side-by-side to identify further shared logic beyond the setup already extracted in Task 6.
  - Common patterns to look for:
    - Stage metric recording loops (same pattern in both)
    - Output artifact generation (constellation SVG, trace SVG, CSV writes)
    - Constellation template construction (64-APSK table)
    - `synchronize_and_downsample` call pattern
    - Final VPP measurement
  - Create shared helper functions in `rf_setup.c` or a new `rf_common.c`:
    - `rf_record_stage_metrics()` — unified metric recording
    - `rf_write_stage_artifacts()` — unified artifact generation call
  - Update both RF functions to use the shared helpers.
  - **Educational**: Show a before/after diff demonstrating the code reduction.

  **Why This Works (Educational)**:
  - **The "Rule of Three"**: When you copy-paste code twice, it's tolerable. The third time, you MUST extract it. Here we have two copies that are ~80% similar — extracting shared patterns reduces maintenance burden and ensures both paths benefit from future improvements.

  **Must NOT do**:
  - Do NOT merge the two RF functions into one — they serve different purposes (baseline vs impaired)
  - Do NOT change any output format or artifact generation

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Requires careful comparison of two large functions, identifying shared patterns without breaking differences
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO — depends on Tasks 7 and 8 being complete (needs the extracted files to compare)
  - **Parallel Group**: Wave 2 (sequential within wave — must run after 7, 8)
  - **Blocks**: None directly
  - **Blocked By**: Tasks 7, 8

  **References** (CRITICAL):
  - **Bruteforce function**: `src/sim_rf.c` — `simulate_bruteforce_rf()` — full implementation (after Task 7)
  - **Realistic function**: `src/sim_rf_realistic.c` — `simulate_realistic_rf()` — full implementation (after Task 8)
  - **Shared setup**: `src/rf_setup.c` — already extracted common setup (from Task 6)
  - **OPTIMIZATIONS.md:9** — documents which sections are identical vs different between paths

  **Why Each Reference**:
  - Both RF files: The comparison targets — must read both to find common patterns
  - rf_setup.c: Already has some shared code — extend it
  - OPTIMIZATIONS.md:9: Shows known differences (e.g., realistic adds impairment processing at specific points)

  **Acceptance Criteria**:
  - [ ] Shared helper functions reduce code duplication by ≥50 lines total
  - [ ] Both RF functions use shared helpers where applicable
  - [ ] `make` compiles with zero warnings
  - [ ] Both baseline and realistic paths produce identical output

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Deduplication preserves both path outputs
    Tool: Bash
    Preconditions: Tasks 1-8 completed, baseline output saved
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Run: diff -r out/rf_baseline/ baseline_out/rf_baseline/ 2>&1
      6. Assert: no significant differences
      7. Run: diff -r out/realistic/ baseline_out/realistic/ 2>&1
      8. Assert: no significant differences
    Expected Result: Both paths produce baseline-identical output
    Failure Indicators: Differences in either path, compile errors
    Evidence: .sisyphus/evidence/task-9-dedup.txt
  ```

  **Commit**: YES (groups with Wave 2)
  - Message: `refactor: deduplicate common logic between bruteforce and realistic RF paths`
  - Files: `src/rf_setup.c`, `src/sim_rf.c`, `src/sim_rf_realistic.c`

- [ ] 10. Reduce SoA↔AoS conversions at FFT/artifact boundaries

  **What to do**:
  - The current flow: SoA arrays → `pack_complex()` → Complex array → FFT → Complex array → `unpack_complex()` → SoA arrays. Each conversion touches every sample.
  - Analysis: Find all call sites of `pack_complex()` and `unpack_complex()`. For each, determine if the conversion is truly necessary or if the downstream consumer could work with SoA directly.
  - **FFT boundary**: `fft_convolve_complex()` takes Complex (AoS) — this is the main conversion hotspot. Can it be modified to accept SoA and internally use the split arrays? Measure: modifying FFT internals would eliminate one pack+unpack pair per convolution call.
  - **Artifact boundary**: `write_constellation_svg()`, `write_trace_svg()`, CSV writers take Complex. This is lower priority — I/O is not the bottleneck.
  - For the FFT boundary: Create a SoA-aware wrapper `fft_convolve_soa(re_in, im_in, n_in, re_kernel, im_kernel, n_kernel, re_out, im_out)` that handles the conversion internally only where needed (butterfly operations on pairs).
  - **Educational**: Measure the overhead with the timing infrastructure (Task 4) before and after.

  **Why This Works (Educational)**:
  - **Memory bandwidth is the real bottleneck**: Modern CPUs can do ~50 GFLOPS but memory bandwidth is ~50 GB/s. For 28M samples at 16 bytes each (Complex), that's 448 MB per pass. Each pack/unpack adds another pass, doubling memory traffic. Eliminating one conversion pair saves ~1 GB of memory traffic per simulation run.
  - **Amdahl's Law in practice**: If pack/unpack is 10% of runtime and you eliminate half of it, you get ~5% speedup. Not dramatic, but adds up with other optimizations.

  **Must NOT do**:
  - Do NOT change FFT numerical output
  - Do NOT remove pack/unpack at artifact boundaries (SVG/CSV writers still need Complex)
  - Do NOT use SIMD intrinsics

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Requires modifying FFT internals and understanding SoA data flow
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 6, 7, 8, 11)
  - **Blocks**: None
  - **Blocked By**: Tasks 1 (fft.h API), 2 (soa_utils.h)

  **References** (CRITICAL):
  - **FFT convolution**: `src/fft.c` — `fft_convolve_complex()` — the function to create SoA variant of
  - **pack/unpack**: `src/soa_utils.c` — `pack_complex()`, `unpack_complex()` — the functions to eliminate calls to
  - **Conversion call sites**: `src/sim_rf.c`, `src/sim_rf_realistic.c` — search for `pack_complex`, `unpack_complex`
  - **OPTIMIZATIONS.md:6** — explains why some buffers stay Complex (FFT, artifact writers)

  **Why Each Reference**:
  - `fft_convolve_complex`: Must understand its algorithm to create SoA variant
  - `soa_utils.c`: Must know what pack/unpack do to eliminate them
  - OPTIMIZATIONS.md:6: Documents the design decision for buffer layout

  **Acceptance Criteria**:
  - [ ] At least one pack/unpack pair eliminated from the hot path
  - [ ] `make` compiles with zero warnings
  - [ ] Numerical output matches baseline

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Reduced conversions produce identical output
    Tool: Bash
    Preconditions: Tasks 1-9 completed, baseline output saved, timing infrastructure active
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000 2>&1 | tee /tmp/after_soa.txt
      4. Assert: exit code 0
      5. Note: timing report should show slight improvement in RF paths
      6. Run: diff out/rf_baseline/csv/stage_metrics.csv baseline_out/rf_baseline/csv/stage_metrics.csv
      7. Assert: no significant differences
    Expected Result: Identical output, slight timing improvement
    Failure Indicators: Numerical divergence, compile errors
    Evidence: .sisyphus/evidence/task-10-soa-conv.txt
  ```

  **Commit**: YES (groups with Wave 2)
  - Message: `perf: reduce SoA↔AoS conversions at FFT boundaries`
  - Files: `src/fft.c`, `include/fft.h`, `src/sim_rf.c`, `src/sim_rf_realistic.c`

- [ ] 11. Extract RRC pulse shaping helpers

  **What to do**:
  - Locate RRC (Root Raised Cosine) pulse shaping code in both RF functions: the `shape_symbols_rrc_to_env_soa()` call and its surrounding setup (RRC filter coefficient computation, rolloff factor handling).
  - Create a helper `rrc_build_pulse(double rolloff, int sps, int span, double *pulse_out)` in `rf_setup.c` (or new `rrc_filter.c`) that precomputes the RRC filter coefficients once.
  - Create a helper `rrc_apply_pulse(const Complex *symbols, size_t nsym, const double *pulse, int pulse_len, int sps, double *env_re, double *env_im)` that applies the pulse to symbols.
  - The current RRC pulse shaping is noted as "serial" in OPTIMIZATIONS.md — overlapping writes prevent OpenMP parallelization. Document this limitation clearly.
  - **Future work note**: This is a known hard-to-parallelize section. Document strategies (e.g., tiled approach with overlap regions) for future investigation.

  **Why This Works (Educational)**:
  - **Not all code can be parallelized**: The RRC pulse shaping has an overlapping write pattern (each symbol's pulse tail overlaps the next symbol's pulse head across `pulse_len` samples). This is an inherent data dependency — teaching when NOT to optimize is as important as knowing how to optimize.

  **Must NOT do**:
  - Do NOT attempt to parallelize RRC (known limitation — document it)
  - Do NOT change the pulse shape or filter coefficients

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Well-scoped extraction of a single DSP helper with clear mathematical definition
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 6, 7, 8, 10)
  - **Blocks**: None
  - **Blocked By**: Tasks 7, 8 (needs extracted RF functions to find call sites)

  **References** (CRITICAL):
  - **RRC call sites**: `src/sim_rf.c`, `src/sim_rf_realistic.c` — search for `shape_symbols_rrc_to_env_soa`
  - **RRC parameters**: `include/sim_types.h` — `SimConfig.rolloff` — the rolloff factor
  - **OPTIMIZATIONS.md:9** — documents RRC as still-serial, with explanation of why
  - **RRC formula**: Standard RRC filter: `h(t) = (sin(πt/T(1-β)) + 4βt/T·cos(πt/T(1+β))) / (πt/T·(1-(4βt/T)²))`

  **Why Each Reference**:
  - RF functions: Find the exact RRC pulse shaping code
  - OPTIMIZATIONS.md:9: Documents the serial limitation — must mention this in comments

  **Acceptance Criteria**:
  - [ ] RRC helpers extracted into clean functions
  - [ ] Both RF paths use the helpers
  - [ ] Comments document why RRC can't be parallelized
  - [ ] `make` compiles with zero warnings

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: RRC extraction preserves pulse shaping behavior
    Tool: Bash
    Preconditions: Tasks 1-10 completed, baseline output saved
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      4. Assert: exit code 0
      5. Run: diff out/rf_baseline/csv/stage_metrics.csv baseline_out/rf_baseline/csv/stage_metrics.csv
      6. Assert: no significant differences (RRC is deterministic — should be exact match)
    Expected Result: Exact match with baseline (RRC is deterministic, no random component)
    Failure Indicators: Any differences in metrics, compile errors
    Evidence: .sisyphus/evidence/task-11-rrc.txt
  ```

  **Commit**: YES (groups with Wave 2)
  - Message: `refactor: extract RRC pulse shaping helpers`
  - Files: `src/rf_setup.c`, `src/sim_rf.c`, `src/sim_rf_realistic.c`

- [ ] 12. Apply cache tiling/blocking on large SoA array loops

  **What to do**:
  - Identify the largest loops in the RF paths: `add_awgn_soa()`, `apply_stage_soa()`, `env_to_rf_soa()`, `mix_down_soa()`. These process arrays of 20-30 million doubles each.
  - For each large loop, implement **cache tiling**: instead of `for (i = 0; i < N; i++) process(re[i], im[i])` in one giant pass, process in **tiles** of ~8K-32K elements that fit in L2/L3 cache:
    ```c
    #define TILE_SIZE 8192
    for (size_t tile = 0; tile < N; tile += TILE_SIZE) {
        size_t end = min(tile + TILE_SIZE, N);
        #pragma omp parallel for schedule(static)
        for (size_t i = tile; i < end; i++) {
            // process re[i], im[i]
        }
    }
    ```
  - The key insight: When processing 28M elements, the data doesn't fit in L3 cache (~30MB). Each pass evicts the beginning before reaching the end. Tiling ensures each tile stays in cache for the duration of its processing.
  - **Priority targets**: `add_awgn_soa` (noise injection — called per stage, runs on all samples), `apply_stage_soa` (gain + nonlinearity — per stage).
  - Add `PREFETCH_R` hints (from Task 5) at tile boundaries to prefetch the next tile while processing the current one.
  - Use the timing infrastructure (Task 4) to measure before/after for each loop.
  - **Educational**: Explain L1/L2/L3 cache hierarchy and why tiling helps — include a diagram in comments.

  **Why This Works (Educational)**:
  - **Cache hierarchy**: L1 = 32KB (1ns), L2 = 256KB-1MB (3ns), L3 = 8-30MB (12ns), RAM = 50-100ns. If your working set fits in L2, you're 30× faster than going to RAM. For 28M doubles (224 MB), a single linear pass guarantees every access is a cache miss. Tiling makes each tile hot in cache.
  - **Temporal locality**: The tile stays in cache for the duration of its inner loop — every element in the tile is accessed from L2/L3 instead of RAM. This is the single most impactful portable optimization for large-array processing.

  **Must NOT do**:
  - Do NOT change the mathematical operations in the loops
  - Do NOT use SIMD intrinsics
  - Do NOT break OpenMP parallelization (tile outer loop must not conflict with inner `parallel for`)

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Requires understanding cache architecture, careful loop transformation, and verification that numerical results don't change
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 3 (with Tasks 13, 14)
  - **Blocks**: None
  - **Blocked By**: Tasks 7, 8 (needs sim_rf.c and sim_rf_realistic.c extracted)

  **References** (CRITICAL):
  - **Hot loops to tile**: `src/sim_rf.c` — `add_awgn_soa()`, `apply_stage_soa()`, `env_to_rf_soa()`, `mix_down_soa()` — the large-array processing loops
  - **Realistic path equivalents**: `src/sim_rf_realistic.c` — same function calls, plus impairment model loops
  - **OpenMP usage**: `OPTIMIZATIONS.md:278-337` — Optimization 9 documents which loops are parallelized and with what schedule
  - **Prefetch macros**: `include/math_utils.h` — `PREFETCH_R`, `PREFETCH_W` (from Task 5)
  - **Timing infrastructure**: `include/perf_timer.h` — for before/after measurement

  **Why Each Reference**:
  - sim_rf.c hot loops: The exact loops to tile — must identify all of them
  - OPTIMIZATIONS.md:9: Shows existing OpenMP structure — must not break it
  - math_utils.h: Macros to apply at tile boundaries

  **Acceptance Criteria**:
  - [ ] Cache tiling applied to ≥3 hot loops
  - [ ] `make` compiles with zero warnings
  - [ ] Timing report shows measurable improvement (≥5%) on RF paths
  - [ ] Numerical output matches baseline

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Cache tiling improves performance without affecting output
    Tool: Bash
    Preconditions: Tasks 1-11 completed, timing infrastructure active
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 10000 2>&1 | tee /tmp/tiled_perf.txt
      4. Assert: exit code 0
      5. Assert: timing report shows RF path runtimes
      6. Run: diff out/rf_baseline/csv/stage_metrics.csv baseline_out/rf_baseline/csv/stage_metrics.csv
      7. Assert: no significant differences
    Expected Result: Identical output, reduced runtime on larger symbol counts
    Failure Indicators: Numerical divergence, segfault, longer runtime (tiling overhead > benefit)
    Evidence: .sisyphus/evidence/task-12-cache-tiling.txt
  ```

  **Commit**: YES (groups with Wave 3)
  - Message: `perf: apply cache tiling on large SoA array loops`
  - Files: `src/sim_rf.c`, `src/sim_rf_realistic.c`

- [ ] 13. Apply loop unrolling + prefetch hints on identified hot paths

  **What to do**:
  - Review all hot loops identified in Tasks 7-8. For each loop where the iteration count is known and the body is simple arithmetic (no function calls, no branches), apply:
    - `UNROLL_HINT(4)` or `UNROLL_HINT(8)` before the loop — tells the compiler to unroll the loop body 4-8 times, reducing branch overhead and enabling instruction-level parallelism
    - `ASSUME_ALIGNED(ptr, 64)` at the top of functions receiving SoA buffers — tells the compiler the pointer is 64-byte aligned, enabling aligned SIMD loads
    - `PREFETCH_R(next_tile_ptr)` before entering the inner loop — prefetches the next tile's data
  - **Targets** (from Task 5's macros):
    - `add_awgn_soa`: `UNROLL_HINT(8)` on the element loop — the body is simple multiply-add
    - `scale_soa`: `UNROLL_HINT(8)` — pure multiplication
    - `apply_stage_soa`: `UNROLL_HINT(4)` (more complex body with nonlinearity)
    - Any loop where `n` is a multiple of the unroll factor (check for remainder handling)
  - **Before/after measurement**: Use timing infrastructure to measure impact. Not all loops benefit from unrolling — if the loop body is already memory-bound, unrolling may have zero effect. Document which loops benefited and which didn't.
  - **Educational**: Explain the trade-off between code size and speed. Over-aggressive unrolling can hurt by blowing out the instruction cache.

  **Why This Works (Educational)**:
  - **Loop overhead**: For a 28M-iteration loop, the branch at the end (`i < N`) executes 28M times. Unrolling by 4 reduces this to 7M branches — fewer pipeline stalls. The compiler can also interleave instructions from different iterations (instruction-level parallelism).
  - **Alignment matters**: Without `__builtin_assume_aligned`, the compiler must emit unaligned loads (`movupd`) which are 2-3× slower on some microarchitectures. With the hint, it emits aligned loads (`movapd`) which run at full speed and enable further auto-vectorization.

  **Must NOT do**:
  - Do NOT apply unroll hints where the loop body is complex (many branches, function calls)
  - Do NOT use SIMD intrinsics
  - Do NOT unroll so aggressively that the binary size doubles

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Requires judgment about which loops benefit from unrolling, careful application with remainder handling
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 3 (with Tasks 12, 14)
  - **Blocks**: None
  - **Blocked By**: Tasks 5 (macro definitions), 7, 8 (extracted RF functions)

  **References** (CRITICAL):
  - **Optimization macros**: `include/math_utils.h` — `UNROLL_HINT`, `ASSUME_ALIGNED`, `PREFETCH_R` (from Task 5)
  - **Hot loop locations**: `src/sim_rf.c`, `src/sim_rf_realistic.c` — `add_awgn_soa`, `scale_soa`, `apply_stage_soa`, etc.
  - **OPTIMIZATIONS.md:6,7** — SoA and alignment rationale — confirms these loops are good candidates

  **Why Each Reference**:
  - math_utils.h: The macros to apply — must use them, not recreate
  - sim_rf.c: The exact loops — must identify the right loops (simple body, known iteration count)

  **Acceptance Criteria**:
  - [ ] Loop hints applied to ≥3 hot loops with measured impact
  - [ ] Comments explain which loops benefited and which didn't
  - [ ] `make` compiles with zero warnings
  - [ ] Numerical output matches baseline

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Loop hints don't break correctness and show performance impact
    Tool: Bash
    Preconditions: Tasks 1-12 completed, timing infrastructure active
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings (check for "ignored pragma" warnings — those are OK but should be documented)
      3. Run: ./bin/dual_receiver_sim --seed 42 --symbols 10000 2>&1 | tee /tmp/unrolled_perf.txt
      4. Assert: exit code 0
      5. Assert: timing report shows RF path runtimes
      6. Run: diff out/rf_baseline/csv/stage_metrics.csv baseline_out/rf_baseline/csv/stage_metrics.csv
      7. Assert: no significant differences
    Expected Result: Identical output, timing report shows any changes
    Failure Indicators: "ignored pragma" is OK, "unrecognized pragma" with errors is NOT OK
    Evidence: .sisyphus/evidence/task-13-loop-hints.txt
  ```

  **Commit**: YES (groups with Wave 3)
  - Message: `perf: apply loop unrolling, prefetch, and alignment hints on hot paths`
  - Files: `src/sim_rf.c`, `src/sim_rf_realistic.c`, `src/signal_chain.c`

- [ ] 14. OpenMP schedule tuning

  **What to do**:
  - The current OpenMP parallelization uses `schedule(static)` everywhere (OPTIMIZATIONS.md:303-309). `static` divides the work into equal chunks — optimal when all iterations take the same time.
  - Identify loops where iteration cost varies:
    - Loops with conditional impairment application (e.g., `if (cfg->enable_iq_gain_error)` inside the loop) — some iterations do more work
    - Loops near the end of the realistic path where some stages have filters and others don't
  - For these, change `schedule(static)` to `schedule(guided)` or `schedule(dynamic, chunk_size)`:
    - `guided`: Starts with large chunks, decreases over time — good for unknown imbalance
    - `dynamic, 256`: Each thread grabs 256 iterations at a time — good for known moderate imbalance, minimal overhead
  - **Measurement**: Run with `OMP_DISPLAY_ENV=TRUE` and compare thread utilization. The goal is to reduce the gap between `real time` and `user time / num_threads`.
  - **Educational**: Document the three schedule types and when to use each — a mini-tutorial on OpenMP load balancing.

  **Why This Works (Educational)**:
  - **Load imbalance**: With `static` scheduling, if thread 0 gets iterations 0-3.5M and thread 15 gets 24.5-28M, but the realistic path adds impairment processing only in certain segments, threads finish at different times. Thread 15 sits idle while thread 0 works. `guided` or `dynamic` lets faster threads pick up more work.
  - **The trade-off**: `static` has zero scheduling overhead but can be imbalanced. `dynamic` has per-chunk overhead (~100ns) but perfect balance. `guided` is the middle ground. For 28M iterations with 256-sized chunks, overhead is ~10ms total — negligible compared to seconds of compute.

  **Must NOT do**:
  - Do NOT change schedule for loops that are perfectly balanced (uniform operations per iteration)
  - Do NOT use `schedule(dynamic, 1)` — chunk size of 1 has prohibitive overhead
  - Do NOT remove any OpenMP directives

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Parameter tuning — changing `schedule(static)` to `schedule(guided)` in specific locations, measuring impact
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 3 (with Tasks 12, 13)
  - **Blocks**: None
  - **Blocked By**: Tasks 7, 8 (needs extracted RF functions)

  **References** (CRITICAL):
  - **Current OpenMP directives**: `src/sim_rf.c`, `src/sim_rf_realistic.c`, `src/signal_chain.c` — search for `#pragma omp parallel for schedule(static)`
  - **OPTIMIZATIONS.md:278-337** — Optimization 9 documents all parallelized loops and their current schedules
  - **Impaired loops**: `src/sim_rf_realistic.c` — realistic path loops with conditional impairment — most likely to benefit from `guided`

  **Why Each Reference**:
  - sim_rf*.c OpenMP directives: The exact lines to modify
  - OPTIMIZATIONS.md:9: Shows which loops are parallelized and with what performance — baseline for comparison

  **Acceptance Criteria**:
  - [ ] Schedule types tuned on ≥2 loops (where iteration cost varies)
  - [ ] Comments document the schedule choice rationale
  - [ ] `make` compiles with zero warnings
  - [ ] Numerical output matches baseline

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Schedule tuning doesn't break output, may improve performance
    Tool: Bash
    Preconditions: Tasks 1-13 completed, timing infrastructure active
    Steps:
      1. Run: make clean && make 2>&1
      2. Assert: exit code 0, zero warnings
      3. Run: OMP_NUM_THREADS=16 ./bin/dual_receiver_sim --seed 42 --symbols 10000 2>&1 | tee /tmp/schedule_perf.txt
      4. Assert: exit code 0
      5. Assert: timing report shows RF path runtimes
      6. Compare user time vs real time ratio — should be closer to num_threads with better scheduling
      7. Run: diff out/rf_baseline/csv/stage_metrics.csv baseline_out/rf_baseline/csv/stage_metrics.csv
      8. Assert: no significant differences
    Expected Result: Identical output, user/real time ratio closer to 16
    Failure Indicators: Numerical differences, segfault, significantly worse runtime
    Evidence: .sisyphus/evidence/task-14-schedule.txt
  ```

  **Commit**: YES (groups with Wave 3)
  - Message: `perf: tune OpenMP schedules for better load balancing`
  - Files: `src/sim_rf.c`, `src/sim_rf_realistic.c`, `src/signal_chain.c`

- [ ] 15. Update Makefile and CMakeLists.txt for new modules

  **What to do**:
  - Add new source files to `Makefile` `SRC` list: `soa_utils.c`, `spectrum.c`, `perf_timer.c`, `rf_setup.c`, `sim_rf.c`, `sim_rf_realistic.c` (some may be added to existing files like `rf_setup.c` expanding `signal_chain.c` or being new).
  - Add new source files to `CMakeLists.txt` `SOURCES` list.
  - Verify that the object file list (`OBJS`) in the Makefile is computed correctly — the `VPATH := src` should handle the `src/` prefix.
  - Verify that `make clean` removes all new `.o` files (the existing `rm -rf $(BIN_DIR)` already handles this since all .o files go to `bin/`).
  - Run `make clean && make -j$(nproc)` to verify parallel build works.
  - Run `make sweep` to verify the Python sweep script still works.
  - **Educational**: Document the Makefile's `VPATH` pattern and why it makes adding new files easy.

  **Why This Works (Educational)**:
  - **Build system hygiene**: Every new .c file needs to be registered in both build systems (Makefile + CMake). Missing one means it compiles with Make but not CMake (or vice versa). This is a common source of "it works on my machine" bugs.

  **Must NOT do**:
  - Do NOT change compiler flags or optimization levels
  - Do NOT remove any existing sources from the build

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple file list update in two build files
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 4 (with Task 16)
  - **Blocks**: None (but must run after all module creation)
  - **Blocked By**: Tasks 1, 2, 3, 4, 6, 7, 8 (new files must exist)

  **References** (CRITICAL):
  - **Makefile**: `Makefile:5` — `SRC` line — add new .c filenames
  - **CMakeLists.txt**: `CMakeLists.txt:24-44` — `SOURCES` list — add new .c filenames with `src/` prefix
  - **New files to add**: `soa_utils.c`, `spectrum.c`, `perf_timer.c`, `rf_setup.c`, `sim_rf.c`, `sim_rf_realistic.c`

  **Why Each Reference**:
  - Makefile SRC line: Exact location to add filenames
  - CMakeLists SOURCES: Exact location — uses `src/` prefix, Makefile uses VPATH

  **Acceptance Criteria**:
  - [ ] `make clean && make` compiles with zero warnings
  - [ ] `cmake -B build && cmake --build build` compiles with zero warnings
  - [ ] `make sweep` runs successfully
  - [ ] All 46 output files generated

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Both build systems compile and produce working binary
    Tool: Bash
    Preconditions: All implementation tasks completed
    Steps:
      1. Run: make clean && make -j$(nproc) 2>&1
      2. Assert: exit code 0, zero warnings
      3. Assert: bin/dual_receiver_sim exists and is executable
      4. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      5. Assert: exit code 0, all output files present
      6. Run: rm -rf build && cmake -B build 2>&1 && cmake --build build 2>&1
      7. Assert: exit code 0 (CMake build succeeds)
    Expected Result: Both Make and CMake builds succeed
    Failure Indicators: Linker errors (missing symbols), compile errors (file not found)
    Evidence: .sisyphus/evidence/task-15-build.txt
  ```

  **Commit**: YES (groups with Wave 4)
  - Message: `build: add new modules to Makefile and CMakeLists.txt`
  - Files: `Makefile`, `CMakeLists.txt`

- [ ] 16. Thin out main.c to orchestrator (~200-400 lines)

  **What to do**:
  - After all extractions (Tasks 1-3, 6-8), `main.c` should only contain:
    - The `main()` function — CLI parsing, calling propagation, cascade, and simulation
    - The 64-APSK constellation construction (if not already in constellation.c)
    - Any remaining `static` helpers too small to extract
    - The complex baseband analytical path orchestration (if not yet extracted)
  - Remove ALL functions that have been extracted to other files.
  - Remove any now-unused `#include` directives.
  - Clean up any orphaned `static` variables or declarations.
  - Add section comments clearly delineating what main() does: (1) Parse CLI, (2) Propagation analysis, (3) Cascade analysis, (4) Complex baseband simulation, (5) RF bruteforce simulation, (6) RF realistic simulation, (7) Print timing report.
  - Target: main.c ≤ 400 lines (down from ~2800+).
  - **Educational**: This is the "thin orchestrator" pattern — main.c should read like a table of contents, with all implementation details delegated to modules.

  **Why This Works (Educational)**:
  - **The "new developer test"**: Can a new developer understand what the program does by reading main.c in under 2 minutes? If main.c is 2800 lines, no. If it's 300 lines of clear orchestration, yes. This is the difference between a codebase that's maintainable and one that's not.

  **Must NOT do**:
  - Do NOT change any remaining logic in `main()`
  - Do NOT remove the complex baseband path (must keep working)
  - Do NOT break the CLI interface

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Cleanup task — remove extracted code, clean includes, add section comments
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 4 (with Task 15)
  - **Blocks**: None
  - **Blocked By**: Tasks 1, 2, 3, 6, 7, 8 (extracted code must exist elsewhere)

  **References** (CRITICAL):
  - **Current main.c**: `src/main.c` — the full file (~2800+ lines) — identify what remains after all extractions
  - **Extracted modules**: Check each: `src/sim_rf.c`, `src/sim_rf_realistic.c`, `src/soa_utils.c`, `src/spectrum.c`, `src/rf_setup.c`, `src/fft.c`, `src/perf_timer.c` — confirm the extracted code is present
  - **Existing includes**: `src/main.c:12-39` — clean up unused includes

  **Why Each Reference**:
  - main.c: Must read the full file to identify what can be removed vs what must stay
  - Extracted modules: Must verify extracted code exists before removing from main.c

  **Acceptance Criteria**:
  - [ ] `main.c` ≤ 400 lines
  - [ ] `make` compiles with zero warnings
  - [ ] All simulation paths produce output
  - [ ] CLI interface unchanged (`./bin/dual_receiver_sim --help` works)

  **QA Scenarios (MANDATORY)**:

  ```
  Scenario: Thin orchestrator compiles and runs all paths
    Tool: Bash
    Preconditions: All implementation tasks completed
    Steps:
      1. Run: wc -l src/main.c → assert ≤ 400 lines
      2. Run: make clean && make 2>&1
      3. Assert: exit code 0, zero warnings
      4. Run: ./bin/dual_receiver_sim --seed 42 --symbols 3000
      5. Assert: exit code 0
      6. Assert: output in out/rf_baseline/ AND out/realistic/ AND out/ (complex baseband) all present
      7. Run: ./bin/dual_receiver_sim --help 2>&1
      8. Assert: help text displays correctly
    Expected Result: All paths work, CLI unchanged, main.c ≤ 400 lines
    Failure Indicators: Missing output directories, CLI broken, main.c > 400 lines
    Evidence: .sisyphus/evidence/task-16-thin-main.txt
  ```

  **Commit**: YES (groups with Wave 4)
  - Message: `refactor: thin main.c to orchestrator (≤400 lines)`
  - Files: `src/main.c`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 2 review tasks run in PARALLEL. Both must PASS. Present results to user and get explicit "okay" before completing.

- [ ] F1. **Baseline Comparison** — `unspecified-high`
  Build from clean: `make clean && make`. Run: `./bin/dual_receiver_sim --seed 42 --symbols 3000`. Compare against pre-refactor baseline output:
  - CSV metrics: diff `out/rf_baseline/csv/` and `out/realistic/csv/` against saved baseline copies
  - Constellation SVGs: verify all 13 stages produce valid SVGs with correct dimensions
  - Trace SVGs: verify all 13 stages produce valid SVGs
  - SNR/EVM values: compare against baseline — must match within ±0.1%
  - File count: verify exactly 46 output files generated
  Output: `Baseline comparison: [N/N files match] | SNR/EVM delta ≤ 0.1% | VERDICT: PASS/FAIL`
  Evidence: `.sisyphus/evidence/final-baseline-diff.txt`

- [ ] F2. **Performance Measurement** — `unspecified-high`
  Run 5 trials each: `./bin/dual_receiver_sim --seed 42 --symbols 3000` and `./bin/dual_receiver_sim --seed 42 --symbols 10000`.
  - Record wall-clock time per trial
  - Compare average to pre-refactor baseline (3.76s for 3000 symbols)
  - Must be ≤ baseline (no regression)
  - Check per-path timing report (if timing infrastructure added)
  Output: `Runtime [avg X.XXs] | vs baseline [Y%] | VERDICT: NO REGRESSION/REGRESSION`
  Evidence: `.sisyphus/evidence/final-perf.txt`

---

## Commit Strategy

- **Waves 1-2 grouped**: `refactor: extract RF modules and consolidate FFT from main.c`
- **Waves 3-4 grouped**: `perf: cache tiling, loop optimization, schedule tuning`
- Each commit includes `make clean && make` verification

---

## Success Criteria

### Verification Commands
```bash
make clean && make                          # Expected: zero warnings, exit 0
./bin/dual_receiver_sim --seed 42 --symbols 3000  # Expected: exit 0, 46 output files
diff -r out/ baseline_out/                  # Expected: no differences in CSV metrics
```

### Final Checklist
- [ ] All "Must Have" present
- [ ] All "Must NOT Have" absent
- [ ] `make` succeeds with zero warnings
- [ ] Output matches baseline (±0.1% on SNR/EVM)
- [ ] All 46 output files generated
- [ ] Runtime ≤ baseline (no regression)
- [ ] `main.c` ≤ 400 lines
- [ ] New modules follow existing code conventions
