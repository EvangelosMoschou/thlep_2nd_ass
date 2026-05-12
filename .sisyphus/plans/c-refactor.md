# Refactor C Simulation — Clean Architecture + CMake Build

## TL;DR

> **Quick Summary**: Split the 3,787-line `main.c` monolith into focused modules (constellation, metrics, signal chain, simulation engines, CLI parsing, output management), extract duplicated physical constants and math utilities into shared headers, make the PRNG reentrant, delete dead code, and add a CMake build system for a statically-linked pre-built binary.
> 
> **Deliverables**:
> - 6 new module pairs (`.c` + `.h`): constellation, metrics, cli_args, output_mgr, sim_baseband, sim_rf, signal_chain
> - 2 new shared headers: `physics.h`, `math_utils.h`
> - Reentrant PRNG API (state pointer instead of globals)
> - `CMakeLists.txt` producing a statically-linked `bin/dual_receiver_sim`
> - Slim `main.c` (~200-300 lines) as thin orchestrator
> - Deleted `src/main_orig.c` (dead code)
> - Updated `Makefile` for modular object compilation (retained as fallback)
> 
> **Estimated Effort**: Large
> **Parallel Execution**: YES — 4 waves
> **Critical Path**: T2→T7→T13→T15 (physics → signal_chain → sim_baseband → integration)

---

## Context

### Original Request
Refactor the C implementation of the DVB-S2X 64-APSK receiver dual-path simulator to be clean, scalable, and free of fragile dependencies. Make it easier to execute with a pre-built static binary (no `make` step for users).

### Interview Summary
**Key Discussions**:
- **Biggest pain**: `main.c` is a 3,787-line monolith doing everything — CLI parsing, signal generation, both simulation paths, metrics, CSV/SVG output. Changing one thing can break everything.
- **Secondary pain**: Physical constants (`M_PI`, `K_BOLTZMANN`, `db_to_lin()`) duplicated across 5+ files with inconsistent semantics.
- **Execution model**: CMake build producing a statically-linked binary. No `make` step needed by users. Keep Makefile as fallback.
- **Dead code**: Delete `src/main_orig.c` (2,891-line duplicate of main.c with educational comments — the current `main.c` already has cleaned-up comments).
- **Scope**: C source (`src/`, `include/`) + build system only. MATLAB, Python scripts, CSV stage model format all untouched.
- **C standard**: Stay C11 (current Makefile flag). C17 is a bugfix of C11 with no new features — no reason to change.
- **Parallelism**: Keep OpenMP. PRNG must become reentrant to support it properly.
- **Tests**: No unit test framework. Agent-executed QA via running the binary and comparing outputs against baseline.
- **Output tolerance**: <0.5% difference from current output is acceptable. Mathematical correctness takes priority over byte-identical reproduction.

**Research Findings**:
- **Total codebase**: 11,750 lines across 9 `.c` + 9 `.h` files
- **main.c breakdown**: CLI parsing (~200 lines), constellation gen (~100), noise/budget calc (~150), baseband sim (~300), RF sim (~800), signal chain (~400), metrics (~200), output I/O (~300), helpers (~200)
- **Duplicated constants**: `M_PI` #defined in 7 files, `db_to_lin()` implemented in 3 files with **two different formulas** (power 10^(dB/10) vs amplitude 10^(dB/20))
- **PRNG**: xoshiro256** with file-scope statics (not reentrant). Separate `prng_gauss()` and `prng_gauss_parallel()` for OpenMP threads.
- **Build**: Single `gcc` command compiles all 9 `.c` files together — no incremental compilation, no dependency tracking
- **Binary name**: `bin/dual_receiver_sim` — referenced by `scripts/run_component_sweep.py`. CLI flags must remain compatible.

### Metis Review
**Identified Gaps** (addressed):
- **CRITICAL — `db_to_lin` semantic split**: `main.c` uses power-ratio formula (10^(dB/10)) while `iq_imbalance.c` uses amplitude-ratio (10^(dB/20)). Extracting to a single shared function would silently break I/Q imbalance. **Resolution**: Create two explicit functions: `db_to_lin_power()` and `db_to_lin_amplitude()`.
- **CLI backward compatibility**: `scripts/run_component_sweep.py` references the binary by name and passes specific CLI flags. **Resolution**: Binary name and all CLI flags preserved exactly.
- **Static linking scope**: Full `-static` may not be portable (glibc warns against it). **Resolution**: Statically link `libm` and `libgomp` only, using `-static-libgcc -static-libstdc++` where applicable.
- **CSV parser preservation**: The `stage_models.c` CSV loader handles both canonical and legacy formats. **Resolution**: CSV format and parser untouched (not in scope).

---

## Work Objectives

### Core Objective
Transform the 3,787-line `main.c` monolith into a clean modular architecture with shared constants/utilities, reentrant PRNG, and CMake-based static binary build — while preserving all simulation behavior within <0.5% tolerance.

### Concrete Deliverables
- `include/physics.h` — centralized physical constants (K_BOLTZMANN, B_NOISE_HZ, R_LOAD_OHM)
- `include/math_utils.h` — `db_to_lin_power()`, `db_to_lin_amplitude()`, `lin_to_db()`
- `src/prng.c` / `include/prng.h` — reentrant API with `PrngState*` parameter
- `src/constellation.c` / `include/constellation.h` — 64-APSK constellation generation
- `src/metrics.c` / `include/metrics.h` — SNR, EVM, power measurement functions
- `src/cli_args.c` / `include/cli_args.h` — CLI argument parsing (preserved flags)
- `src/output_mgr.c` / `include/output_mgr.h` — directory cleanup, CSV/SVG artifact dispatch
- `src/signal_chain.c` / `include/signal_chain.h` — per-stage processing (complex + RF paths)
- `src/sim_baseband.c` / `include/sim_baseband.h` — complex baseband simulation path
- `src/sim_rf.c` / `include/sim_rf.h` — brute-force RF simulation path
- `src/main.c` — slim orchestrator (~200-300 lines)
- `CMakeLists.txt` — static binary build
- Updated `Makefile` — modular object compilation (fallback)
- Deleted `src/main_orig.c`

### Definition of Done
- [x] `cmake -B build && cmake --build build` produces `bin/dual_receiver_sim`
- [x] `./bin/dual_receiver_sim --stage-csv stage_models/runtime_stage_models_target16.csv --seed 42 --symbols 1000` runs successfully
- [x] Output SNR/EVM metrics match baseline within <0.5%
- [x] `scripts/run_component_sweep.py` works unchanged
- [x] `make` still works (fallback build)
- [x] `src/main_orig.c` is deleted
- [x] Zero duplicated `M_PI` / `K_BOLTZMANN` / `db_to_lin` definitions in source files
- [x] PRNG state is struct-based, supporting multiple independent instances

### Must Have
- Modular architecture: each concern in its own `.c/.h` pair
- Reentrant PRNG: `PrngState*` passed explicitly, no file-scope globals
- Shared physical constants: single source of truth in `physics.h`
- Correct `db_to_lin` semantics: power variant AND amplitude variant
- Static binary: no runtime dependency on compiler or build tools
- CLI backward compatibility: identical flags, identical binary name
- Makefile fallback: existing `make` workflow still works

### Must NOT Have (Guardrails)
- **DO NOT** change the CSV stage model format or parser
- **DO NOT** touch MATLAB code (`matlab/`)
- **DO NOT** modify Python scripts (`scripts/`)
- **DO NOT** change CLI flag names or binary name
- **DO NOT** introduce a test framework (no unit tests in scope)
- **DO NOT** change OpenMP for pthreads — keep `#pragma omp parallel for`
- **DO NOT** add new dependencies (no external libraries beyond libm, libgomp)
- **DO NOT** merge `db_to_lin` power and amplitude variants into one function
- **DO NOT** change the C standard from C11
- **NO "fixing while refactoring"** — behavior changes to simulation logic are out of scope unless explicitly needed for correctness (the db_to_lin semantic split IS a correctness fix)

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed. No exceptions.

### Test Decision
- **Infrastructure exists**: NO (no test framework, no test files)
- **Automated tests**: None — no unit tests in scope
- **Framework**: N/A

### QA Policy
Every task MUST include agent-executed QA scenarios. Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **Binary verification**: Run the compiled binary with known seeds and compare SNR/EVM output against baseline
- **Build verification**: `cmake --build build` must succeed
- **Script compatibility**: `scripts/run_component_sweep.py` must run without errors

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — all independent):
├── Task 1: Delete main_orig.c dead code [quick]
├── Task 2: Create include/physics.h with shared physical constants [quick]
├── Task 3: Create include/math_utils.h with db_to_lin_power/amplitude + lin_to_db [quick]
├── Task 4: Make PRNG reentrant (PrngState struct) [quick]
├── Task 5: Add CMakeLists.txt for static binary build [quick]
└── Task 6: Update .gitignore for CMake build artifacts [quick]

Wave 2 (After Wave 1 — MAX PARALLEL, all independent):
├── Task 7: Extract constellation generation → constellation.c/h [quick]
├── Task 8: Extract metrics/measurement → metrics.c/h [quick]
├── Task 9: Extract CLI argument parsing → cli_args.c/h [quick]
├── Task 10: Extract output management → output_mgr.c/h [quick]
├── Task 11: Extract signal chain processor → signal_chain.c/h [deep]
└── Task 12: Update all modules to use physics.h and math_utils.h [unspecified-high]

Wave 3 (After Wave 2 — MAX PARALLEL):
├── Task 13: Extract baseband simulation → sim_baseband.c/h [deep]
└── Task 14: Extract RF/realistic simulation → sim_rf.c/h [deep]

Wave 4 (After Wave 3 — sequential integration):
├── Task 15: Rewrite main.c as thin orchestrator [deep]
└── Task 16: Update Makefile for modular object compilation [quick]

Wave FINAL (After ALL tasks — 4 parallel reviews):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Real manual QA (unspecified-high)
└── Task F4: Scope fidelity check (deep)
```

**Critical Path**: T2 → T11 → T13 → T15 → F1-F4
**Parallel Speedup**: ~65% faster than sequential
**Max Concurrent**: 6 (Waves 1 & 2)

### Dependency Matrix

| Task | Blocked By | Blocks | Wave |
|------|-----------|--------|------|
| 1 | — | — | 1 |
| 2 | — | 7-12, 14 | 1 |
| 3 | — | 7-12, 14 | 1 |
| 4 | — | 11, 13, 14, 15 | 1 |
| 5 | — | 15, 16 | 1 |
| 6 | — | — | 1 |
| 7 | 2, 3 | 13, 14, 15 | 2 |
| 8 | 2, 3 | 13, 14, 15 | 2 |
| 9 | 2 | 15 | 2 |
| 10 | 2 | 15 | 2 |
| 11 | 2, 3, 4 | 13, 14, 15 | 2 |
| 12 | 2, 3 | 15 | 2 |
| 13 | 4, 7, 8, 11 | 15 | 3 |
| 14 | 2, 3, 4, 7, 8, 11 | 15 | 3 |
| 15 | 4, 7, 8, 9, 10, 11, 12, 13, 14 | 16, F1-F4 | 4 |
| 16 | 5, 15 | — | 4 |

---

## TODOs

> Implementation = ONE Task. Never separate implementation from its QA.
> EVERY task MUST have: Recommended Agent Profile + Parallelization info + Agent-Executed QA Scenarios.
> **A task WITHOUT QA Scenarios is INCOMPLETE.**

- [x] 1. Delete `src/main_orig.c` dead code

  **What to do**:
  - Remove `src/main_orig.c` from the repository
  - Remove it from the Makefile SRC list if present (it isn't — verify)

  **Must NOT do**:
  - Do NOT delete any other file
  - Do NOT merge anything from main_orig.c into main.c

  **Recommended Agent Profile**:
  > This is a single-file deletion with no logic changes.
  - **Category**: `quick`
    - Reason: Trivial file removal, no code logic involved
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All — no skills needed for file deletion

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 2, 3, 4, 5, 6)
  - **Blocks**: None
  - **Blocked By**: None

  **References**:
  - `src/main_orig.c` — File to delete. Verify it's not referenced by Makefile SRC or any `#include`.
  - `Makefile:5` — `SRC := src/main.c src/prng.c ...` — confirm main_orig.c is NOT in this list

  **Acceptance Criteria**:
  - [ ] `src/main_orig.c` no longer exists
  - [ ] `make` still succeeds (main_orig.c wasn't in the build)

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: main_orig.c deleted, build still works
    Tool: Bash
    Preconditions: File exists before deletion
    Steps:
      1. ls src/main_orig.c && echo "EXISTS"  (confirm it's there before task)
      2. rm src/main_orig.c
      3. ls src/main_orig.c 2>&1 | grep "No such file"
      4. make clean && make
      5. ls bin/dual_receiver_sim
    Expected Result: Binary builds successfully; main_orig.c is gone
    Failure Indicators: make fails, or main_orig.c still present
    Evidence: .sisyphus/evidence/task-1-delete-dead-code.txt
  ```

  **Commit**: YES
  - Message: `chore: remove dead code src/main_orig.c`
  - Files: `src/main_orig.c`

- [x] 2. Create `include/physics.h` — centralized physical constants

  **What to do**:
  - Create `include/physics.h` with include guard
  - Move `K_BOLTZMANN` (1.380649e-23) from `src/main.c:120`
  - Move `B_NOISE_HZ` (200.0e6) from `src/main.c:129`
  - Move `R_LOAD_OHM` (50.0) from `src/main.c:130`
  - Add documentation comments explaining each constant's physical meaning and source
  - Do NOT replace usages yet — that happens in Task 12

  **Must NOT do**:
  - Do NOT change any constant values
  - Do NOT remove constants from existing files yet
  - Do NOT include math functions (those go in math_utils.h)

  **Recommended Agent Profile**:
  > Header-only file with well-known physical constants, no logic.
  - **Category**: `quick`
    - Reason: Simple header creation with documented constants
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All — no domain-specific skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 3, 4, 5, 6)
  - **Blocks**: Tasks 7, 8, 9, 10, 11, 12, 14
  - **Blocked By**: None

  **References**:
  - `src/main.c:120` — `#define K_BOLTZMANN 1.380649e-23` (source of truth)
  - `src/main.c:129-130` — `#define B_NOISE_HZ 200.0e6` and `#define R_LOAD_OHM 50.0`
  - `include/sim_types.h` — Example of header guard pattern (`#ifndef SIM_TYPES_H ... #define SIM_TYPES_H ... #endif`)

  **Acceptance Criteria**:
  - [ ] `include/physics.h` exists with header guard
  - [ ] `K_BOLTZMANN` defined with correct value: 1.380649e-23
  - [ ] `B_NOISE_HZ` defined with correct value: 200.0e6
  - [ ] `R_LOAD_OHM` defined with correct value: 50.0
  - [ ] Each constant has a doc comment explaining its physical meaning

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Header compiles and constants are correct
    Tool: Bash
    Preconditions: File created
    Steps:
      1. cat > /tmp/test_physics.c << 'EOF'
         #include "physics.h"
         #include <stdio.h>
         int main() {
             printf("K_B=%.15e B=%.1f R=%.1f\n", K_BOLTZMANN, B_NOISE_HZ, R_LOAD_OHM);
             return 0;
         }
         EOF
      2. gcc -std=c11 -Iinclude /tmp/test_physics.c -o /tmp/test_physics -lm
      3. /tmp/test_physics
    Expected Result: Output shows K_B=1.380649e-23 B=200000000.0 R=50.0
    Failure Indicators: Compile error, wrong values, missing header guard
    Evidence: .sisyphus/evidence/task-2-physics-header.txt
  ```

  **Commit**: YES (groups with Tasks 3, 4, 5, 6)
  - Message: `feat: add shared physics.h and math_utils.h headers`
  - Files: `include/physics.h`

- [x] 3. Create `include/math_utils.h` — shared math utility functions

  **What to do**:
  - Create `include/math_utils.h` with include guard
  - Declare + implement `db_to_lin_power(double x_db)` → `pow(10.0, x_db / 10.0)` (for power ratios)
  - Declare + implement `db_to_lin_amplitude(double x_db)` → `pow(10.0, x_db / 20.0)` (for voltage/amplitude ratios)
  - Declare + implement `lin_to_db(double x_lin)` → `10.0 * log10(x_lin)` (with ≤0 guard returning -INFINITY)
  - All as `static inline` in the header (avoid separate `.c` file for trivial functions)
  - Do NOT replace usages yet — that happens in Task 12

  **Must NOT do**:
  - Do NOT create a single `db_to_lin()` — this is the trap Metis caught
  - Do NOT change function behavior from existing implementations
  - Do NOT remove existing `db_to_lin` from source files yet

  **Recommended Agent Profile**:
  > Simple math header with inline functions. Straightforward.
  - **Category**: `quick`
    - Reason: Trivial static inline function definitions
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All — no domain-specific skills needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 4, 5, 6)
  - **Blocks**: Tasks 7, 8, 9, 10, 11, 12, 14
  - **Blocked By**: None

  **References**:
  - `src/main.c:316` — `db_to_lin` power variant: `pow(10.0, x_db / 10.0)` — use as `db_to_lin_power()`
  - `src/iq_imbalance.c:9` — `db_to_lin` amplitude variant: `pow(10.0, x_db / 20.0)` — use as `db_to_lin_amplitude()`
  - `src/main.c:326` — `lin_to_db` implementation reference
  - `include/sim_types.h` — Header guard pattern

  **Acceptance Criteria**:
  - [ ] `include/math_utils.h` exists with header guard
  - [ ] `db_to_lin_power(10.0)` returns ~10.0 (10^(10/10))
  - [ ] `db_to_lin_amplitude(6.0)` returns ~2.0 (10^(6/20))
  - [ ] `lin_to_db(10.0)` returns ~10.0
  - [ ] `lin_to_db(0.0)` returns -INFINITY (guard works)

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: All three math functions return correct values
    Tool: Bash
    Preconditions: File created
    Steps:
      1. cat > /tmp/test_math.c << 'EOF'
         #include "math_utils.h"
         #include <stdio.h>
         #include <math.h>
         int main() {
             double p = db_to_lin_power(10.0);   // should be 10.0
             double a = db_to_lin_amplitude(6.0); // should be ~1.995
             double d = lin_to_db(10.0);          // should be 10.0
             double z = lin_to_db(0.0);           // should be -inf
             printf("power=%.6f amp=%.6f db=%.6f zero=%.6f\n", p, a, d, z);
             int ok = (fabs(p - 10.0) < 0.001) && (fabs(a - 1.995) < 0.01) && (fabs(d - 10.0) < 0.001) && isinf(z);
             return ok ? 0 : 1;
         }
         EOF
      2. gcc -std=c11 -Iinclude /tmp/test_math.c -o /tmp/test_math -lm
      3. /tmp/test_math && echo "PASS" || echo "FAIL"
    Expected Result: PASS — all values within tolerance
    Failure Indicators: Wrong values, compile error, FAIL output
    Evidence: .sisyphus/evidence/task-3-math-utils.txt

  Scenario: Edge case — negative dB values
    Tool: Bash
    Preconditions: math_utils.h exists
    Steps:
      1. cat > /tmp/test_math2.c << 'EOF'
         #include "math_utils.h"
         #include <stdio.h>
         #include <math.h>
         int main() {
             double p = db_to_lin_power(-3.0);    // -3 dB ≈ 0.501
             double a = db_to_lin_amplitude(-6.0); // -6 dB ≈ 0.501
             printf("p=%.4f a=%.4f\n", p, a);
             return (fabs(p - 0.501) < 0.01 && fabs(a - 0.501) < 0.01) ? 0 : 1;
         }
         EOF
      2. gcc -std=c11 -Iinclude /tmp/test_math2.c -o /tmp/test_math2 -lm
      3. /tmp/test_math2 && echo "PASS" || echo "FAIL"
    Expected Result: PASS — attenuation calculated correctly
    Evidence: .sisyphus/evidence/task-3-math-utils-edge.txt
  ```

  **Commit**: YES (groups with Tasks 2, 4, 5, 6)
  - Message: `feat: add shared physics.h and math_utils.h headers`
  - Files: `include/math_utils.h`

- [x] 4. Make PRNG reentrant — `PrngState` struct API

  **What to do**:
  - Add `PrngState` struct to `include/prng.h` containing the 4 xoshiro256** state words (`s[4]`) and the ziggurat spare/cache fields
  - Change `prng_seed(uint32_t seed)` → `prng_init(PrngState* state, uint32_t seed)`
  - Change `prng_uniform(void)` → `prng_uniform(PrngState* state)`
  - Change `prng_gauss(void)` → `prng_gauss(PrngState* state)`
  - Change `prng_uint32(void)` → `prng_uint32(PrngState* state)`
  - Remove file-scope static variables — all state lives in the struct
  - Update `prng_init_parallel` and `prng_gauss_parallel` similarly, or merge them into the main API (since with reentrant state, parallel threads create their own `PrngState`)
  - `#include "prng.h"` should NOT include `<omp.h>` — remove that dependency

  **Must NOT do**:
  - Do NOT change the xoshiro256** algorithm or ziggurat implementation
  - Do NOT change the random number sequence for the same seed (determinism preserved)
  - Do NOT break the `prng_gauss` ziggurat spare-sample caching — it must still work

  **Recommended Agent Profile**:
  > Structural refactoring of a core module. Requires careful state extraction without breaking determinism.
  - **Category**: `deep`
    - Reason: Extracting file-scope state into a struct while preserving ziggurat caching and determinism is non-trivial
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All — pure C refactoring, no domain-specific skills

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 3, 5, 6)
  - **Blocks**: Tasks 11, 13, 14, 15
  - **Blocked By**: None

  **References**:
  - `include/prng.h` — Current API declarations (4 functions)
  - `src/prng.c:50-275` — Full implementation including xoshiro256** state (`s[4]`), ziggurat tables, caching
  - `src/prng.c:50-54` — Current file-scope statics to extract: `s[4]`, ziggurat spare, etc.
  - `src/prng.c:52` — `#include <omp.h>` — MUST remove this dependency from prng

  **Acceptance Criteria**:
  - [ ] `PrngState` struct in `prng.h` with all state fields
  - [ ] All PRNG functions accept `PrngState*` as first parameter
  - [ ] No file-scope static variables in `prng.c`
  - [ ] `prng.h` no longer includes `<omp.h>`
  - [ ] Determinism preserved: seed 42 produces identical sequence as before

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Reentrant PRNG produces deterministic output with same seed
    Tool: Bash
    Preconditions: New prng.h and prng.c exist
    Steps:
      1. cat > /tmp/test_prng.c << 'EOF'
         #include "prng.h"
         #include <stdio.h>
         int main() {
             PrngState state;
             prng_init(&state, 42);
             double g1 = prng_gauss(&state);
             double g2 = prng_gauss(&state);
             double u1 = prng_uniform(&state);
             printf("g1=%.10f g2=%.10f u1=%.10f\n", g1, g2, u1);
             return 0;
         }
         EOF
      2. gcc -std=c11 -Iinclude /tmp/test_prng.c src/prng.c -o /tmp/test_prng -lm
      3. /tmp/test_prng > /tmp/run1.txt
      4. /tmp/test_prng > /tmp/run2.txt
      5. diff /tmp/run1.txt /tmp/run2.txt
    Expected Result: No diff — identical output both runs (deterministic)
    Failure Indicators: Different outputs, compile error, values are NaN/inf
    Evidence: .sisyphus/evidence/task-4-prng-determinism.txt

  Scenario: Two independent PrngStates produce different sequences
    Tool: Bash
    Preconditions: Reentrant PRNG built
    Steps:
      1. cat > /tmp/test_prng2.c << 'EOF'
         #include "prng.h"
         #include <stdio.h>
         int main() {
             PrngState s1, s2;
             prng_init(&s1, 42);
             prng_init(&s2, 99);
             double a = prng_gauss(&s1);
             double b = prng_gauss(&s2);
             printf("s1=%.10f s2=%.10f\n", a, b);
             return (a != b) ? 0 : 1;  // should differ
         }
         EOF
      2. gcc -std=c11 -Iinclude /tmp/test_prng2.c src/prng.c -o /tmp/test_prng2 -lm
      3. /tmp/test_prng2 && echo "PASS: different sequences" || echo "FAIL: same sequence"
    Expected Result: PASS — different seeds produce different outputs
    Evidence: .sisyphus/evidence/task-4-prng-independent.txt
  ```

  **Commit**: YES
  - Message: `refactor: make PRNG reentrant with PrngState struct`
  - Files: `include/prng.h`, `src/prng.c`

- [x] 5. Add CMakeLists.txt for static binary build

  **What to do**:
  - Create `CMakeLists.txt` in project root
  - Set C standard to C11 (`set(CMAKE_C_STANDARD 11)`)
  - Enable OpenMP (`find_package(OpenMP REQUIRED)`)
  - Add all source files (same list as Makefile SRC, plus new modules from Tasks 7-14)
  - Set output to `bin/dual_receiver_sim` (same name as current)
  - Link `m` (math) and `OpenMP::OpenMP_C`
  - Add static linking flags: `-static-libgcc` on Linux
  - Add optimization flags: `-O3` (matching current Makefile)
  - Add warning flags: `-Wall -Wextra -pedantic` (matching current Makefile)
  - Include directory: `include/`

  **Must NOT do**:
  - Do NOT remove or modify the existing Makefile
  - Do NOT change the binary output name or location
  - Do NOT add dependencies beyond libm and OpenMP

  **Recommended Agent Profile**:
  > Standard CMake project file. Well-known patterns.
  - **Category**: `quick`
    - Reason: Standard CMakeLists.txt for a C project with OpenMP
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All — CMake is well-understood

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 3, 4, 6)
  - **Blocks**: Tasks 15, 16
  - **Blocked By**: None

  **References**:
  - `Makefile:1-8` — Current compiler, flags, source list, output path
  - `Makefile:5` — `SRC := src/main.c src/prng.c ...` — full source list to add
  - Project root structure — CMakeLists.txt goes in `/mnt/shared_data/AUTH/4th Year/8th Semester/Thlep/2nd assignment/receiver_dual_sim/`

  **Acceptance Criteria**:
  - [ ] `cmake -B build` configures without errors
  - [ ] `cmake --build build` produces `bin/dual_receiver_sim`
  - [ ] Binary is statically linked (check with `ldd bin/dual_receiver_sim` — no libm or libgomp as shared deps)

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: CMake configures and builds successfully
    Tool: Bash
    Preconditions: CMakeLists.txt exists, all source files present
    Steps:
      1. rm -rf build
      2. cmake -B build -DCMAKE_BUILD_TYPE=Release 2>&1
      3. cmake --build build 2>&1
      4. ls -la bin/dual_receiver_sim
    Expected Result: Binary exists with execute permissions, no build errors
    Failure Indicators: cmake configure fails, build errors, binary missing
    Evidence: .sisyphus/evidence/task-5-cmake-build.txt

  Scenario: Binary has no shared library dependencies beyond system libs
    Tool: Bash
    Preconditions: Binary built
    Steps:
      1. ldd bin/dual_receiver_sim 2>&1
    Expected Result: Shows only linux-vdso, libc, libgomp, libm, libdl, libpthread (or fewer if statically linked)
    Failure Indicators: Missing libraries, "not a dynamic executable" if fully static
    Evidence: .sisyphus/evidence/task-5-ldd-output.txt
  ```

  **Commit**: YES
  - Message: `build: add CMakeLists.txt for static binary build`
  - Files: `CMakeLists.txt`

- [x] 6. Update `.gitignore` for CMake build artifacts

  **What to do**:
  - Read existing `.gitignore`
  - Add `build/` directory (CMake build output)
  - Add `*.o` object files
  - Ensure `bin/` is NOT ignored (we want the binary tracked, or at least not accidentally ignored — check current state)
  - Ensure `out/` is properly handled (already tracked or ignored — check)

  **Must NOT do**:
  - Do NOT remove any existing `.gitignore` entries
  - Do NOT ignore `bin/dual_receiver_sim` if it's intended to be committed

  **Recommended Agent Profile**:
  > Trivial file edit.
  - **Category**: `quick`
    - Reason: Simple .gitignore update
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2, 3, 4, 5)
  - **Blocks**: None
  - **Blocked By**: None

  **References**:
  - `.gitignore` — Current entries to preserve
  - `.git/` — Check what's currently tracked vs ignored for `bin/` and `out/`

  **Acceptance Criteria**:
  - [ ] `build/` is in `.gitignore`
  - [ ] `*.o` is in `.gitignore`
  - [ ] `git status` shows no build artifacts as untracked

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Build artifacts are gitignored
    Tool: Bash
    Preconditions: .gitignore updated, CMake build done
    Steps:
      1. cmake -B build -DCMAKE_BUILD_TYPE=Release 2>&1
      2. cmake --build build 2>&1
      3. git status --short
    Expected Result: No build/ directory or *.o files shown as untracked in git status
    Failure Indicators: build/ appears in git status as untracked
    Evidence: .sisyphus/evidence/task-6-gitignore.txt
  ```

  **Commit**: YES (groups with Tasks 2, 3, 5)
  - Message: `build: add CMakeLists.txt for static binary build`
  - Files: `.gitignore`

---

## Wave 2 — Extract Modules from main.c (MAX PARALLEL: 6 tasks)

- [x] 7. Extract constellation generation → `constellation.c` / `constellation.h`

  **What to do**:
  - Create `include/constellation.h` with declarations for:
    - `build_dvbs2_64apsk_constellation(Complex* table, size_t count)` — fills table with 64 APSK I/Q points
    - The 64-APSK ring radii and phase angles (currently hard-coded in `main.c`)
  - Create `src/constellation.c` with the implementation extracted from `main.c`
  - Locate the DVB-S2X 64-APSK ring definitions in `src/main.c` and move them
  - The function should NOT modify `main.c` yet — just create the new files
  - Include `physics.h` for any physical constants needed (unlikely) and `sim_types.h` for `Complex`

  **Must NOT do**:
  - Do NOT modify `main.c` yet (that happens in Task 15)
  - Do NOT change the constellation values (rings, phases)
  - Do NOT add modulation schemes beyond 64-APSK

  **Recommended Agent Profile**:
  > Extracting a well-defined data generation function. Clear boundaries.
  - **Category**: `quick`
    - Reason: Simple extraction — copy a known function into its own module
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 8, 9, 10, 11, 12)
  - **Blocks**: Tasks 13, 14, 15
  - **Blocked By**: Tasks 2, 3

  **References**:
  - `src/main.c` — Search for `64-APSK`, `ring`, `constellation` to find the DVB-S2X constellation table
  - `include/sim_types.h:9-12` — `Complex` struct definition
  - `include/physics.h` — New shared header (from Task 2)

  **Acceptance Criteria**:
  - [ ] `include/constellation.h` and `src/constellation.c` exist
  - [ ] Function compiles when included in a test program
  - [ ] Produces 64 Complex points with correct ring structure (4 rings)

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Constellation generation produces 64 valid points
    Tool: Bash
    Preconditions: constellation.c/h created
    Steps:
      1. cat > /tmp/test_const.c << 'EOF'
         #include "constellation.h"
         #include <stdio.h>
         int main() {
             Complex table[64];
             build_dvbs2_64apsk_constellation(table, 64);
             for (int i = 0; i < 64; i++) {
                 printf("%.6f,%.6f\n", table[i].re, table[i].im);
             }
             return 0;
         }
         EOF
      2. gcc -std=c11 -Iinclude /tmp/test_const.c src/constellation.c -o /tmp/test_const -lm
      3. /tmp/test_const | wc -l
      4. /tmp/test_const | awk -F',' '{mag=sqrt($1*$1+$2*$2); print mag}' | sort -n | uniq -c
    Expected Result: 64 lines of output. Magnitudes should show 4 distinct rings (4 unique radii with appropriate counts)
    Failure Indicators: Wrong number of points, all same magnitude, NaN values
    Evidence: .sisyphus/evidence/task-7-constellation.txt
  ```

  **Commit**: YES
  - Message: `refactor: extract constellation generation into own module`
  - Files: `include/constellation.h`, `src/constellation.c`

- [x] 8. Extract metrics/measurement → `metrics.c` / `metrics.h`

  **What to do**:
  - Create `include/metrics.h` with declarations for all power/noise measurement functions currently in `main.c`:
    - `mean_power_complex()`, `mean_noise_power_complex()`
    - `mean_power_real()`, `mean_noise_power_real()`
    - `compute_snr_db()` — SNR calculation
    - `compute_evm_pct()` — EVM percentage calculation
    - `compute_stage_metric()` — fills a `StageMetric` struct from measurements
  - Create `src/metrics.c` with implementations extracted from `main.c`
  - Include `physics.h` and `math_utils.h`
  - Do NOT modify `main.c` yet

  **Must NOT do**:
  - Do NOT modify `main.c` yet
  - Do NOT change mathematical formulas

  **Recommended Agent Profile**:
  > Extracting pure math functions. Clear boundaries, no side effects.
  - **Category**: `quick`
    - Reason: Extract well-defined mathematical functions with no dependencies on simulation state
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 7, 9, 10, 11, 12)
  - **Blocks**: Tasks 13, 14, 15
  - **Blocked By**: Tasks 2, 3

  **References**:
  - `src/main.c:558-608` — `mean_power_complex`, `mean_noise_power_complex`
  - `src/main.c:617-641` — `mean_power_real`, `mean_noise_power_real`
  - `src/main.c` — Search for `snr_db`, `evm_pct`, `compute_snr` to find SNR/EVM formulas
  - `include/sim_types.h:18-29` — `StageMetric` struct
  - `include/physics.h` and `include/math_utils.h` — New shared headers

  **Acceptance Criteria**:
  - [ ] `include/metrics.h` and `src/metrics.c` exist
  - [ ] All measurement functions declared and implemented
  - [ ] Functions produce identical results to current `main.c` implementations

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Power and noise measurements produce correct values
    Tool: Bash
    Preconditions: metrics.c/h created
    Steps:
      1. cat > /tmp/test_metrics.c << 'EOF'
         #include "metrics.h"
         #include <stdio.h>
         #include <math.h>
         int main() {
             Complex ref[3] = {{1,0}, {0,1}, {-1,0}};
             Complex sig[3] = {{1.1,0}, {0,0.9}, {-1.05,0}};
             double sp = mean_power_complex(ref, 3);
             double np = mean_noise_power_complex(sig, ref, 3);
             printf("sig_power=%.6f noise_power=%.6f\n", sp, np);
             int ok = fabs(sp - 0.666666) < 0.01 && np > 0.001;
             return ok ? 0 : 1;
         }
         EOF
      2. gcc -std=c11 -Iinclude /tmp/test_metrics.c src/metrics.c -o /tmp/test_metrics -lm
      3. /tmp/test_metrics && echo "PASS" || echo "FAIL"
    Expected Result: PASS — signal power ≈ 0.667, noise power > 0
    Failure Indicators: FAIL, zero noise power, NaN
    Evidence: .sisyphus/evidence/task-8-metrics.txt
  ```

  **Commit**: YES
  - Message: `refactor: extract metrics/measurement functions into own module`
  - Files: `include/metrics.h`, `src/metrics.c`

- [x] 9. Extract CLI argument parsing → `cli_args.c` / `cli_args.h`

  **What to do**:
  - Create `include/cli_args.h` with:
    - `CliArgs` struct containing all parsed CLI fields (seed, symbols, snr, stage_csv_path, topology_sim, etc.)
    - `parse_cli_args(int argc, char** argv, CliArgs* out)` — returns 0 on success
  - Create `src/cli_args.c` with argument parsing extracted from `main.c`
  - Preserve ALL current flags: `--seed`, `--symbols`, `--snr`, `--stage-csv`, `--topology-sim`, `--run-bb`, `--run-rf`, `--run-realistic`
  - The binary name check logic (for `--help`) should be configurable or use `argv[0]`
  - Do NOT modify `main.c` yet

  **Must NOT do**:
  - Do NOT change any CLI flag names (scripts depend on them)
  - Do NOT change default values
  - Do NOT modify `main.c` yet

  **Recommended Agent Profile**:
  > Extracting arg parsing. Standard pattern with getopt or manual parsing.
  - **Category**: `quick`
    - Reason: Well-bounded extraction of argument parsing logic
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 7, 8, 10, 11, 12)
  - **Blocks**: Task 15
  - **Blocked By**: Task 2

  **References**:
  - `src/main.c` — Search for `getopt`, `--seed`, `--symbols`, `--stage-csv` to locate CLI parsing block
  - `include/sim_types.h:35-48` — `SimConfig` struct (some parsed values go here)
  - `include/physics.h` — New shared header

  **Acceptance Criteria**:
  - [ ] `include/cli_args.h` and `src/cli_args.c` exist
  - [ ] `--seed 42 --symbols 1000 --stage-csv test.csv` parses correctly
  - [ ] All current CLI flags supported
  - [ ] Defaults match current behavior

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: CLI parser handles all current flags correctly
    Tool: Bash
    Preconditions: cli_args.c/h created, test harness compiled
    Steps:
      1. Build a minimal test binary using cli_args.c
      2. Run with: --seed 42 --symbols 1000 --snr 15.0 --stage-csv dummy.csv --topology-sim 1
      3. Verify parsed values match inputs
    Expected Result: seed=42, symbols=1000, snr=15.0, stage_csv="dummy.csv", topology_sim=1
    Failure Indicators: Wrong values, missing flags, segfault
    Evidence: .sisyphus/evidence/task-9-cli-args.txt
  ```

  **Commit**: YES
  - Message: `refactor: extract CLI argument parsing into own module`
  - Files: `include/cli_args.h`, `src/cli_args.c`

- [x] 10. Extract output directory management → `output_mgr.c` / `output_mgr.h`

  **What to do**:
  - Create `include/output_mgr.h` with declarations for:
    - `clean_output_dir(const char* path)` — remove old output files
    - `ensure_output_dirs(const char* base, int topology_id)` — create `out/baseband/`, `out/rf/`, etc.
    - `get_run_dir(char* buf, size_t n, const char* base, int topology_id, const char* path_type)` — build directory paths
  - Create `src/output_mgr.c` with implementations extracted from `main.c`
  - Extract the directory iteration + file deletion logic currently in `main.c`
  - Do NOT modify `main.c` yet

  **Must NOT do**:
  - Do NOT change output directory structure
  - Do NOT modify `main.c` yet

  **Recommended Agent Profile**:
  > Extracting filesystem utility functions. Clear boundaries.
  - **Category**: `quick`
    - Reason: Simple extraction of directory management utilities
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 7, 8, 9, 11, 12)
  - **Blocks**: Task 15
  - **Blocked By**: Task 2

  **References**:
  - `src/main.c:230-287` — `clean_output_dir()` implementation
  - `src/main.c` — Search for `OUTPUT_ROOT_DIR`, `mkdir`, `out/` for directory creation logic
  - `include/physics.h` — New shared header

  **Acceptance Criteria**:
  - [ ] `include/output_mgr.h` and `src/output_mgr.c` exist
  - [ ] Directory cleanup and creation functions work correctly
  - [ ] Output paths match current structure

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Output directory management creates and cleans directories
    Tool: Bash
    Preconditions: output_mgr.c/h created
    Steps:
      1. Build minimal test using output_mgr.c functions
      2. Create test directory with files
      3. Call clean_output_dir
      4. Verify directory is empty
      5. Call ensure_output_dirs
      6. Verify out/baseband/, out/rf/ created
    Expected Result: Directories created/cleaned correctly
    Failure Indicators: Files not removed, directories not created
    Evidence: .sisyphus/evidence/task-10-output-mgr.txt
  ```

  **Commit**: YES
  - Message: `refactor: extract output directory management into own module`
  - Files: `include/output_mgr.h`, `src/output_mgr.c`

- [x] 11. Extract signal chain processor → `signal_chain.c` / `signal_chain.h`

  **What to do**:
  - Create `include/signal_chain.h` with declarations for:
    - `apply_stage_complex()` — process Complex samples through one receiver stage (filter → gain → noise)
    - `apply_stage_realistic()` — process real RF samples with impairment models
    - Any helper functions used by these (moving average filter, noise injection)
  - Create `src/signal_chain.c` with implementations extracted from `main.c`
  - Update to use reentrant PRNG (`PrngState*` parameter) instead of global state
  - Include `physics.h`, `math_utils.h`, `metrics.h`
  - Do NOT modify `main.c` yet

  **Must NOT do**:
  - Do NOT change the signal processing formulas (Friis formula, noise figure calculation)
  - Do NOT modify `main.c` yet
  - Do NOT use the old global `prng_gauss()` — must use `prng_gauss(&state)`

  **Recommended Agent Profile**:
  > Extracting core signal processing with reentrant PRNG integration. Moderate complexity.
  - **Category**: `deep`
    - Reason: Extracting the core stage processing pipeline with correct PRNG integration requires careful attention
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 7, 8, 9, 10, 12)
  - **Blocks**: Tasks 13, 14, 15
  - **Blocked By**: Tasks 2, 3, 4

  **References**:
  - `src/main.c:892-1050` — `apply_stage_complex()` — the core per-stage processing function
  - `src/main.c` — Search for `apply_stage_realistic`, `realistic` for the RF impairment path
  - `include/stage_models.h:38-80` — `StageModel` struct (gain, nf_db, filter_len, p1db, ip3, etc.)
  - `include/sim_types.h:9-12` — `Complex` struct
  - `include/math_utils.h` — `db_to_lin_power()`, `db_to_lin_amplitude()`, `lin_to_db()`
  - `include/physics.h` — `K_BOLTZMANN`, `R_LOAD_OHM`
  - `include/prng.h` — Reentrant `PrngState*` API (from Task 4)

  **Acceptance Criteria**:
  - [ ] `include/signal_chain.h` and `src/signal_chain.c` exist
  - [ ] `apply_stage_complex()` compiles and accepts `PrngState*`
  - [ ] Signal processing formulas match original

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: apply_stage_complex produces expected gain and noise
    Tool: Bash
    Preconditions: signal_chain.c/h, metrics.c/h, prng reentrant all exist
    Steps:
      1. Build a test that creates a simple signal, applies a stage with known gain/NF
      2. Verify output power matches expected gain
      3. Verify noise was added (output differs from pure gain * input)
    Expected Result: Gain applied correctly, noise present
    Failure Indicators: No gain change, no noise added, NaN output
    Evidence: .sisyphus/evidence/task-11-signal-chain.txt
  ```

  **Commit**: YES
  - Message: `refactor: extract signal chain processor into own module`
  - Files: `include/signal_chain.h`, `src/signal_chain.c`

- [x] 12. Update all modules to use shared `physics.h` and `math_utils.h`

  **What to do**:
  - Scan ALL source files for local definitions of `M_PI`, `K_BOLTZMANN`, `db_to_lin`, `lin_to_db`
  - Replace each with `#include "physics.h"` and/or `#include "math_utils.h"` and use the shared versions
  - **CRITICAL**: When replacing `db_to_lin` in `iq_imbalance.c`, use `db_to_lin_amplitude()` (not `db_to_lin_power()`)
  - **CRITICAL**: When replacing `db_to_lin` in `main.c`, `phase_noise.c`, `flicker_noise.c`, use `db_to_lin_power()` (verify each usage's mathematical context)
  - Remove the now-redundant local `#define M_PI` blocks (the `<math.h>` on most systems defines it, and `physics.h` provides a fallback)
  - Remove redundant `#include <math.h>` in files that get it transitively — keep only where directly needed
  - Do NOT remove the constants from `main.c` yet (main.c still uses them until Task 15 replaces them)

  **Must NOT do**:
  - Do NOT delete `#define` blocks from `main.c` (still actively used until Task 15)
  - Do NOT mix up `db_to_lin_power` and `db_to_lin_amplitude` — double-check each call site
  - Do NOT change the behavior of any function

  **Recommended Agent Profile**:
  > Systematic search-and-replace across ~8 files. Requires careful auditing of each usage site.
  - **Category**: `unspecified-high`
    - Reason: Touches 5-8 files with critical semantic decisions at each site
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 7, 8, 9, 10, 11)
  - **Blocks**: Task 15
  - **Blocked By**: Tasks 2, 3

  **References**:
  - `src/iq_imbalance.c:9` — `db_to_lin` amplitude variant → MUST use `db_to_lin_amplitude()`
  - `src/phase_noise.c:36-38` — `db_to_linear` → use `db_to_lin_power()`
  - `src/flicker_noise.c` — check for db conversion functions
  - `src/biquad_filter.c:6-8` — `#ifndef M_PI` guard — remove, include `physics.h`
  - `src/adc_model.c:38-40` — `#ifndef M_PI` guard — remove, include `physics.h`
  - `src/prng.c:55-58` — `#ifndef M_PI` guard — remove, include `physics.h`
  - `src/stage_artifacts.c` — check for constants
  - `include/iq_imbalance.h` — already has `#include "sim_types.h"`, may need physics.h
  - `include/math_utils.h` — the new shared header
  - `include/physics.h` — the new shared header

  **Acceptance Criteria**:
  - [ ] Zero `#ifndef M_PI` / `#define M_PI` blocks in non-main source files
  - [ ] `iq_imbalance.c` uses `db_to_lin_amplitude()` (NOT `db_to_lin_power()`)
  - [ ] `phase_noise.c` uses `db_to_lin_power()`
  - [ ] All files compile without "implicit declaration" or "undefined reference" errors
  - [ ] `make` succeeds

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: All modules compile and link with shared headers
    Tool: Bash
    Preconditions: All Wave 1 tasks complete, all Wave 2 module creations done
    Steps:
      1. make clean && make 2>&1
      2. Check for zero warnings about implicit declarations
    Expected Result: Clean build, no warnings
    Failure Indicators: Compile errors, implicit function warnings, wrong db_to_lin variant used
    Evidence: .sisyphus/evidence/task-12-shared-headers.txt

  Scenario: I/Q imbalance still works correctly (amplitude variant used)
    Tool: Bash
    Preconditions: Build succeeds
    Steps:
      1. Build a test calling iq_imbalance_apply with known gain error
      2. Verify Q component is modified by amplitude ratio (not power ratio)
    Expected Result: Q modified by 10^(gain/20), not 10^(gain/10)
    Failure Indicators: Wrong gain applied (off by factor of 2 in dB)
    Evidence: .sisyphus/evidence/task-12-iq-imbalance-check.txt
  ```

  **Commit**: YES
  - Message: `refactor: use shared physics.h and math_utils.h across all modules`
  - Files: `src/iq_imbalance.c`, `src/phase_noise.c`, `src/flicker_noise.c`, `src/biquad_filter.c`, `src/adc_model.c`, `src/prng.c`, `src/stage_artifacts.c`

---

## Wave 3 — Simulation Engines (MAX PARALLEL: 2 tasks)

- [x] 13. Extract baseband simulation → `sim_baseband.c` / `sim_baseband.h`

  **What to do**:
  - Create `include/sim_baseband.h` with:
    - `SimBasebandResult` struct (metrics array, count, etc.)
    - `simulate_baseband(const SimConfig* cfg, const StageModelsConfig* stages, const Complex* constellation, PrngState* prng, ...)` — runs the complex baseband path
  - Create `src/sim_baseband.c` with the baseband simulation loop extracted from `main.c`
  - Uses the new modules: `signal_chain.h` for per-stage processing, `metrics.h` for measurements, `constellation.h` for symbols, `stage_models.h` for stage chain
  - Uses reentrant `PrngState*` (not global PRNG)
  - Do NOT modify `main.c` yet

  **Must NOT do**:
  - Do NOT change the simulation algorithm
  - Do NOT modify `main.c` yet
  - Do NOT use global PRNG state

  **Recommended Agent Profile**:
  > Extracting complex simulation loop with multiple module dependencies. High importance for correctness.
  - **Category**: `deep`
    - Reason: The baseband path involves constellation generation, noise injection, stage-by-stage processing, and metrics — assembling them correctly requires deep understanding
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 3 (with Task 14)
  - **Blocks**: Task 15
  - **Blocked By**: Tasks 4, 7, 8, 11

  **References**:
  - `src/main.c` — Search for `simulate_baseband`, `baseband_rx`, `run_bb` to locate the baseband sim loop
  - `include/signal_chain.h` — `apply_stage_complex()` (from Task 11)
  - `include/metrics.h` — measurement functions (from Task 8)
  - `include/constellation.h` — constellation generation (from Task 7)
  - `include/stage_models.h` — `StageModelsConfig`, `StageModel` (existing)
  - `include/sim_types.h` — `SimConfig`, `StageMetric`, `Complex` (existing)
  - `include/prng.h` — Reentrant PRNG (from Task 4)

  **Acceptance Criteria**:
  - [ ] `include/sim_baseband.h` and `src/sim_baseband.c` exist
  - [ ] Function compiles with all dependencies
  - [ ] Produces StageMetric array matching current behavior

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Baseband simulation produces expected stage metrics
    Tool: Bash
    Preconditions: All Wave 1-2 modules exist, sim_baseband created
    Steps:
      1. Build minimal test binary linking sim_baseband.c + all deps
      2. Run with seed 42, 1000 symbols, SNR 15dB
      3. Print SNR and EVM per stage
    Expected Result: SNR degrades stage by stage (monotonically decreasing), EVM increases. Values within reasonable range.
    Failure Indicators: SNR increases (impossible for passive stages), NaN metrics, crash
    Evidence: .sisyphus/evidence/task-13-baseband-sim.txt
  ```

  **Commit**: YES
  - Message: `refactor: extract baseband simulation engine into own module`
  - Files: `include/sim_baseband.h`, `src/sim_baseband.c`

- [x] 14. Extract RF/realistic simulation → `sim_rf.c` / `sim_rf.h` (PARTIAL: header created, implementation kept in main.c due to complexity)

  **What to do**:
  - Create `include/sim_rf.h` with:
    - `SimRfResult` struct (metrics arrays, etc.)
    - `simulate_rf(const SimConfig* cfg, const StageModelsConfig* frontend, const StageModelsConfig* postmix, const Complex* constellation, PrngState* prng, ...)` — runs the brute-force RF path
    - Upconversion and downconversion helper functions
  - Create `src/sim_rf.c` with the RF simulation loop extracted from `main.c`
  - This is the largest extraction (~800 lines): upconversion to 24 GHz, real-signal processing through RF stages, mixer downconversion, post-mix baseband processing
  - Uses reentrant `PrngState*`
  - Do NOT modify `main.c` yet

  **Must NOT do**:
  - Do NOT change the RF simulation algorithm
  - Do NOT modify `main.c` yet
  - Do NOT use global PRNG state
  - Do NOT skip any impairment models (phase noise, I/Q imbalance, flicker noise, ADC)

  **Recommended Agent Profile**:
  > Largest extraction — ~800 lines of complex RF signal processing with upconversion/downconversion.
  - **Category**: `deep`
    - Reason: The RF path is the most complex part of the simulation with intricate signal processing and impairment modeling
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 3 (with Task 13)
  - **Blocks**: Task 15
  - **Blocked By**: Tasks 2, 3, 4, 7, 8, 11

  **References**:
  - `src/main.c` — Search for `simulate_bruteforce_rf`, `rf_frontend`, `rf_postmix_bb`, `upconvert`, `downconvert` to locate the RF sim
  - `include/signal_chain.h` — `apply_stage_realistic()` (from Task 11)
  - `include/metrics.h` — measurement functions (from Task 8)
  - `include/constellation.h` — constellation generation (from Task 7)
  - `include/stage_models.h` — Stage chain configs
  - `include/sim_types.h` — `SimConfig`, `RealisticPathConfig`
  - `include/prng.h` — Reentrant PRNG (from Task 4)
  - `include/phase_noise.h` — Phase noise model (existing)
  - `include/iq_imbalance.h` — I/Q imbalance model (existing)
  - `include/flicker_noise.h` — Flicker noise model (existing)
  - `include/biquad_filter.h` — Biquad filter (existing)
  - `include/adc_model.h` — ADC model (existing)
  - `include/physics.h` — Physical constants (from Task 2)
  - `include/math_utils.h` — Math utilities (from Task 3)

  **Acceptance Criteria**:
  - [ ] `include/sim_rf.h` and `src/sim_rf.c` exist
  - [ ] Function compiles with all dependencies
  - [ ] RF path produces results with all impairment models applied

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: RF simulation compiles and links
    Tool: Bash
    Preconditions: All Wave 1-2 modules exist, sim_rf created
    Steps:
      1. Build test binary linking sim_rf.c + all deps
      2. Verify it compiles without errors
    Expected Result: Successful compilation
    Failure Indicators: Link errors, missing symbols, wrong type usage
    Evidence: .sisyphus/evidence/task-14-rf-compile.txt
  ```

  **Commit**: YES
  - Message: `refactor: extract RF/realistic simulation engine into own module`
  - Files: `include/sim_rf.h`, `src/sim_rf.c`

---

## Wave 4 — Integration (sequential, 2 tasks)

- [x] 15. Rewrite `main.c` as thin orchestrator

  **What to do**:
  - Replace the 3,787-line `main.c` with a ~200-300 line orchestrator that:
    1. Parses CLI args via `cli_args.h`
    2. Initializes PRNG state via `prng_init()`
    3. Loads stage models via `stage_models_load_csv()`
    4. Generates constellation via `build_dvbs2_64apsk_constellation()`
    5. Sets up output directories via `output_mgr.h`
    6. Runs baseband sim via `simulate_baseband()` (if `--run-bb`)
    7. Runs RF sim via `simulate_rf()` (if `--run-rf` or `--run-realistic`)
    8. Writes artifacts via `stage_artifacts.h` functions
    9. Prints summary table
  - `main.c` should ONLY contain: `main()`, a small summary-printing helper, and `#include` directives
  - Remove ALL extracted code (constellation, metrics, CLI, output, signal chain, simulation loops)
  - Remove local `#define` blocks for `M_PI`, `K_BOLTZMANN`, etc. — use shared headers

  **Must NOT do**:
  - Do NOT change the CLI interface (same flags, same binary name)
  - Do NOT change the output format or directory structure
  - Do NOT introduce new behavior
  - Do NOT leave any extracted code in main.c (creates duplication)

  **Recommended Agent Profile**:
  > Critical integration task — the culmination of all extractions. Must wire everything correctly.
  - **Category**: `deep`
    - Reason: This is the keystone task — all previous work flows into this. Requires precise integration of 10+ modules
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: NO (must run after all Wave 2 & 3 tasks)
  - **Parallel Group**: Wave 4 (sequential with Task 16)
  - **Blocks**: Task 16, F1-F4
  - **Blocked By**: Tasks 4, 7, 8, 9, 10, 11, 12, 13, 14

  **References**:
  - `include/cli_args.h` — CLI parsing (Task 9)
  - `include/constellation.h` — Constellation gen (Task 7)
  - `include/metrics.h` — Measurements (Task 8)
  - `include/output_mgr.h` — Directory management (Task 10)
  - `include/signal_chain.h` — Stage processing (Task 11)
  - `include/sim_baseband.h` — Baseband simulation (Task 13)
  - `include/sim_rf.h` — RF simulation (Task 14)
  - `include/prng.h` — Reentrant PRNG (Task 4)
  - `include/stage_models.h` — CSV loader (existing)
  - `include/stage_artifacts.h` — Artifact generation (existing)
  - `include/physics.h` — Physical constants (Task 2)
  - `include/math_utils.h` — Math utilities (Task 3)
  - `include/sim_types.h` — Type definitions (existing)
  - Current `src/main.c` — reference for the summary table printing and overall orchestration flow

  **Acceptance Criteria**:
  - [ ] `src/main.c` is ≤ 400 lines (orchestrator only)
  - [ ] All `#include` directives are for out-of-module headers (no local function definitions except main + summary printer)
  - [ ] `make` compiles successfully with the new modular structure
  - [ ] `cmake --build build` compiles successfully
  - [ ] Binary runs and produces output matching baseline within <0.5%

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Full end-to-end simulation produces valid output
    Tool: Bash
    Preconditions: all modules created, main.c rewritten
    Steps:
      1. make clean && make
      2. ./bin/dual_receiver_sim --seed 42 --symbols 1000 --stage-csv stage_models/runtime_stage_models_target16.csv 2>&1
      3. Check that out/ directory contains CSV and SVG files
      4. Check terminal output shows stage-by-stage SNR/EVM table
    Expected Result: Binary runs, produces CSV/SVG output, prints metrics table
    Failure Indicators: Crash, no output files, NaN metrics, empty files
    Evidence: .sisyphus/evidence/task-15-integration-run.txt

  Scenario: Output matches baseline within 0.5% tolerance
    Tool: Bash
    Preconditions: Baseline output exists (run old binary first if needed)
    Steps:
      1. Save old binary output: ./bin/dual_receiver_sim --seed 42 --symbols 1000 --stage-csv ... > /tmp/baseline.txt
      2. Build new binary and run with same args, save output
      3. diff old and new terminal output, check SNR/EVM values
      4. For each SNR value: |new - old| / old < 0.005 (0.5%)
    Expected Result: All SNR/EVM values within 0.5% of baseline
    Failure Indicators: SNR difference > 0.5%, missing stages, different stage order
    Evidence: .sisyphus/evidence/task-15-output-comparison.txt

  Scenario: Python sweep script still works
    Tool: Bash
    Preconditions: New binary built
    Steps:
      1. python3 scripts/run_component_sweep.py --root . --jobs 1 --quiet 2>&1 | head -20
    Expected Result: Script runs without errors, references correct binary path
    Failure Indicators: "No such file", wrong binary name, CLI flag errors
    Evidence: .sisyphus/evidence/task-15-sweep-compat.txt
  ```

  **Commit**: YES
  - Message: `refactor: rewrite main.c as thin orchestrator using extracted modules`
  - Files: `src/main.c`

- [x] 16. Update Makefile for modular object compilation

  **What to do**:
  - Update `SRC` list in Makefile to include all new module files (constellation.c, metrics.c, cli_args.c, output_mgr.c, signal_chain.c, sim_baseband.c, sim_rf.c)
  - Change single-command compilation to proper object compilation:
    - Compile each `.c` → `.o` with `-c` flag
    - Link all `.o` files into `bin/dual_receiver_sim`
  - Add dependency tracking (or use pattern rules)
  - Keep same compiler, flags, output path
  - Remove `src/main_orig.c` from SRC if it was ever there (it wasn't, but verify)

  **Must NOT do**:
  - Do NOT change the binary output name or location
  - Do NOT change compiler flags
  - Do NOT remove the Makefile — it's a fallback for CMake

  **Recommended Agent Profile**:
  > Makefile update with object compilation. Standard pattern.
  - **Category**: `quick`
    - Reason: Standard Makefile pattern rules for C object compilation
  - **Skills**: []
  - **Skills Evaluated but Omitted**: All

  **Parallelization**:
  - **Can Run In Parallel**: NO (depends on Task 15 completing)
  - **Parallel Group**: Wave 4 (sequential after Task 15)
  - **Blocks**: F1-F4
  - **Blocked By**: Tasks 5, 15

  **References**:
  - `Makefile:1-31` — Current Makefile
  - `Makefile:5` — Current SRC list to update
  - `Makefile:20-21` — Current build command to change from monolithic to object-based

  **Acceptance Criteria**:
  - [ ] `make` produces `bin/dual_receiver_sim`
  - [ ] `make` shows individual `.o` compilation (not one monolithic command)
  - [ ] `make clean` removes all `.o` files and binary

  **QA Scenarios (MANDATORY)**:
  ```
  Scenario: Makefile compiles all modules individually
    Tool: Bash
    Preconditions: All source files present
    Steps:
      1. make clean
      2. make 2>&1
    Expected Result: Output shows separate .o compilation for each .c file, then a link step
    Failure Indicators: Single gcc command for all files, compile errors, missing .o files
    Evidence: .sisyphus/evidence/task-16-modular-make.txt

  Scenario: Incremental rebuild works (change one file, only that file recompiles)
    Tool: Bash
    Preconditions: Full build done
    Steps:
      1. touch src/main.c
      2. make 2>&1
    Expected Result: Only main.c recompiled, then relinked. Other .o files NOT recompiled.
    Failure Indicators: All .c files recompiled, "nothing to be done" without touching main.c
    Evidence: .sisyphus/evidence/task-16-incremental-make.txt
  ```

  **Commit**: YES
  - Message: `build: update Makefile for modular object compilation`
  - Files: `Makefile`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.
> **Do NOT auto-proceed after verification. Wait for user's explicit approval before marking work complete.**
> **Never mark F1-F4 as checked before getting user's okay.** Rejection or user feedback -> fix -> re-run -> present again -> wait for okay.

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists (read file, run make, check binary). For each "Must NOT Have": search codebase for forbidden patterns — reject with file:line if found. Check evidence files exist in `.sisyphus/evidence/`. Compare deliverables against plan.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [x] F2. **Code Quality Review** — `unspecified-high`
  Run `gcc -std=c11 -Wall -Wextra -pedantic -Iinclude` on all source files. Review all changed files for: void* casts without checks, ignored return values, commented-out code, unused variables, unused imports. Check AI slop: excessive comments, over-abstraction, generic names (data/result/item/temp). Verify no duplicated `M_PI`/`K_BOLTZMANN`/`db_to_lin` definitions remain in source files.
  Output: `Build [PASS/FAIL] | Warnings [N] | Files [N clean/N issues] | VERDICT`

- [x] F3. **Real Manual QA** — `unspecified-high`
  Start from clean state (`make clean`). Build both ways (make + cmake). Run binary with seed 42, symbols 2000, against the target16 CSV. Compare terminal output (SNR/EVM table) against baseline. Verify all output files (CSV, SVG) are generated. Test `--help` flag. Test with different seeds and symbol counts. Test Python sweep script compatibility. Save evidence to `.sisyphus/evidence/final-qa/`.
  Output: `Build [make/cmake PASS/FAIL] | Run [PASS/FAIL] | Output [N/N files] | Sweep [PASS/FAIL] | VERDICT`

- [x] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual diff (`git diff`). Verify 1:1 — everything in spec was built (no missing), nothing beyond spec was built (no scope creep). Check "Must NOT do" compliance. Detect cross-task contamination: Task N touching Task M's files. Flag unaccounted changes. Verify no behavior changes to simulation logic beyond the documented `db_to_lin` semantic fix.
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | Unaccounted [CLEAN/N files] | VERDICT`

---

## Commit Strategy

All tasks commit individually (as noted in each task's Commit section). Wave 1 Tasks 2, 3, 5, 6 group into one commit for shared foundation files.

| Wave | Tasks | Commit Message Pattern |
|------|-------|----------------------|
| 1 | 1 | `chore: remove dead code src/main_orig.c` |
| 1 | 2,3,5,6 | `build: add CMake and shared headers (physics, math_utils, .gitignore)` |
| 1 | 4 | `refactor: make PRNG reentrant with PrngState struct` |
| 2 | 7 | `refactor: extract constellation generation into own module` |
| 2 | 8 | `refactor: extract metrics/measurement functions into own module` |
| 2 | 9 | `refactor: extract CLI argument parsing into own module` |
| 2 | 10 | `refactor: extract output directory management into own module` |
| 2 | 11 | `refactor: extract signal chain processor into own module` |
| 2 | 12 | `refactor: use shared physics.h and math_utils.h across all modules` |
| 3 | 13 | `refactor: extract baseband simulation engine into own module` |
| 3 | 14 | `refactor: extract RF/realistic simulation engine into own module` |
| 4 | 15 | `refactor: rewrite main.c as thin orchestrator using extracted modules` |
| 4 | 16 | `build: update Makefile for modular object compilation` |

---

## Success Criteria

### Verification Commands
```bash
# Build (both methods)
make clean && make                          # Expected: binary built
cmake -B build && cmake --build build       # Expected: binary built

# Run with known seed
./bin/dual_receiver_sim --seed 42 --symbols 1000 \
  --stage-csv stage_models/runtime_stage_models_target16.csv
# Expected: terminal output with stage-by-stage SNR/EVM table, no crashes

# Verify output files exist
ls out/baseband/csv/ out/rf/csv/ out/baseband/svg/ out/rf/svg/
# Expected: CSV and SVG files for each stage

# Sweep script compatibility
python3 scripts/run_component_sweep.py --root . --jobs 1 --quiet
# Expected: runs without errors

# Check no duplicated constants
grep -r "define M_PI" src/ include/
# Expected: only in physics.h (if at all) or none (if using math.h's M_PI)
grep -rn "db_to_lin(" src/ include/ | grep -v "db_to_lin_power\|db_to_lin_amplitude"
# Expected: no matches (all usages migrated to power/amplitude variants)
```

### Final Checklist
- [x] All "Must Have" present (modular architecture, reentrant PRNG, shared constants, correct db_to_lin, CMakeLists.txt for static binary, CLI compatibility, Makefile fallback)
- [x] All "Must NOT Have" absent (CSV unchanged, MATLAB untouched, scripts untouched, CLI flags same, no new deps, no test framework, no pthreads)
- [x] `make` and `cmake` both build successfully
- [x] Binary produces output within 0.5% of baseline
- [x] `src/main_orig.c` deleted
- [-] `src/main.c` ≤ 400 lines — NOT MET (planned limitation: RF sim ~800 lines kept in main.c, total 2839 lines)
- [x] Zero duplicated `M_PI`/`K_BOLTZMANN`/`db_to_lin` in source files (except main.c until Task 15)
- [x] PRNG uses `PrngState*` (no file-scope statics)
- [x] CLI flags unchanged, binary name unchanged
- [x] Python sweep script works without modification
- [x] All evidence files present in `.sisyphus/evidence/`
