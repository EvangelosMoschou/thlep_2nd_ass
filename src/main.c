/*
 * ============================================================================
 * main.c — 64-APSK Receiver Dual-Mode Simulator
 * ============================================================================
 *
 * OVERVIEW:
 *   This is the main program body for a software simulator that models a
 *   satellite receiver processing 64-APSK signals (specifically DVB-S2X,
 *   the standard used by modern satellite TV and broadband systems).
 *
 *   "64-APSK" stands for 64-point Amplitude and Phase Shift Keying. It is
 *   a modulation scheme where information is encoded in 64 distinct complex
 *   symbols, each with a unique amplitude and phase. The 64 symbols are
 *   arranged in 4 concentric rings on the I/Q (In-phase / Quadrature) plane.
 *
 * TWO SIMULATION PATHS:
 *   The simulator runs TWO independent simulations and compares their results:
 *
 *   1. COMPLEX BASEBAND (Fast Analytical Path):
 *      - Works directly on complex I/Q symbol values
 *      - Does NOT model the actual 24 GHz RF carrier
 *      - Equivalent to assuming perfect upconversion and downconversion
 *      - Very fast: O(N_symbols × N_stages)
 *      - Uses the "baseband_rx" stage chain from the CSV
 *
 *   2. BRUTE-FORCE RF (Realistic Path):
 *      - Upconverts symbols to a real 24 GHz signal sampled at 96 GHz
 *      - Passes the real RF waveform through RF frontend stages
 *      - Downconverts back to complex baseband using a mixer model
 *      - Post-downconversion stages process the recovered I/Q symbols
 *      - Much slower: O(N_symbols × SPS × N_stages), where SPS ≈ 9600
 *      - Uses "rf_frontend" + "rf_postmix_bb" stage chains from the CSV
 *
 * SIGNAL FLOW (both paths):
 *   Transmitter:
 *     1. Build 64-APSK constellation (64 fixed I/Q points)
 *     2. Randomly select N symbols from the constellation
 *     3. Copy as "reference" (clean) and "received" (will be degraded)
 *     4. Add AWGN (Additive White Gaussian Noise) to the received copy
 *
 *   Receiver:
 *     5. For each stage in the receiver chain:
 *        a. Apply filter (moving average, if configured)
 *        b. Apply gain (amplification or attenuation)
 *        c. Add stage-specific noise (based on Noise Figure)
 *     6. Measure SNR and EVM at each stage
 *     7. Write CSV data and SVG visualizations
 *
 * KEY METRICS:
 *   - SNR (Signal-to-Noise Ratio, dB): Higher = better.
 *     Measures how much stronger the signal is than the noise.
 *   - EVM (Error Vector Magnitude, %): Lower = better.
 *     Measures how far received symbols deviate from their ideal positions.
 *
 * OUTPUTS:
 *   - out/topology_sim_N/csv/ — CSV data files (metrics, constellations, traces)
 *   - out/topology_sim_N/svg/ — SVG chart images (viewable in any browser)
 *   - stdout — Console summary with stage-by-stage SNR/EVM table
 *
 * ============================================================================
 */

#include <errno.h>        /* For errno — error code set by file/number operations */
#include <ctype.h>        /* For isspace() — whitespace detection (not heavily used here) */
#include <dirent.h>       /* For opendir(), readdir() — listing directory contents */
#include <float.h>        /* For DBL_MAX — largest possible double value */
#include <limits.h>       /* For INT_MIN, INT_MAX, UINT_MAX — integer range limits */
#include <math.h>         /* For sin(), cos(), sqrt(), log10(), pow(), hypot(), etc. */
#include <sys/stat.h>     /* For stat(), mkdir() — checking/creating directories */
#include <sys/types.h>    /* For mode_t — file permission type used with mkdir */
#include <stdint.h>       /* For uint32_t, int64_t — fixed-width integer types */
#include <stdio.h>        /* For printf(), fprintf(), fopen(), fclose(), etc. */
#include <stdlib.h>       /* For malloc(), calloc(), free(), strtod(), strtoul() */
#include <string.h>       /* For strcmp(), memset(), memcpy(), snprintf() */
#include <time.h>         /* For time() — current time (used as default PRNG seed) */

#include "prng.h"            /* Our pseudo-random number generator (see prng.c) */
#include "sim_types.h"       /* Data types: Complex, StageMetric, SimConfig */
#include "stage_artifacts.h" /* Output writers: CSV and SVG file generators */
#include "stage_models.h"    /* CSV loader for receiver stage configurations */

/*
 * M_PI — The mathematical constant π (pi ≈ 3.14159).
 * Not guaranteed to be defined by all C compilers (it's a POSIX extension),
 * so we define it ourselves if it's missing.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * K_BOLTZMANN — Boltzmann's constant in Joules per Kelvin (J/K).
 *
 * This fundamental physical constant relates temperature to energy.
 * It's used to calculate thermal noise power using the formula:
 *   P_noise = k_B × T × B
 * Where:
 *   k_B = 1.380649 × 10^-23 J/K (Boltzmann's constant)
 *   T   = antenna temperature in Kelvin (e.g., 150 K for a satellite dish)
 *   B   = noise bandwidth in Hz (e.g., 12 MHz for 10 Msym/s with rolloff 0.2)
 *
 * This gives the minimum possible noise power for a system at temperature T
 * over bandwidth B — the fundamental limit set by thermodynamics.
 */
#define K_BOLTZMANN 1.380649e-23

/*
 * OUTPUT_ROOT_DIR — Base directory for all simulation output files.
 * Output is organized as: out/baseband/ and out/rf/
 */
#define OUTPUT_ROOT_DIR "out"
#define TOPOLOGY_SIM_COUNT 1

/*
 * TOPOLOGY_SIM_COUNT — Number of simulation "slots" (separate output directories).
 * Users can run up to 5 independent simulations (e.g., with different stage CSV
 * files or different SNR values) and compare results side-by-side.
 */

/*
 * MAX_METRICS — Maximum number of stage metric entries per simulation path.
 *
 * Each simulation path records one metric entry for:
 *   - The input (before any stages)
 *   - Each stage in the chain
 *   - (RF path only) The downconverter
 *
 * The worst-case count is:
 *   Baseband: 1 (input) + N_bb_stages
 *   RF:       1 (input_rf) + N_rf_stages + 1 (downconverter) + N_postmix_stages
 *
 * 32 is enough for chains with up to ~30 stages. If you add many stages to
 * your CSV file, you may need to increase this value.
 */
#define MAX_METRICS 32


/* ============================================================================
 * DIRECTORY MANAGEMENT UTILITIES
 * ============================================================================
 *
 * These functions create and clean the output directory structure.
 * The layout is:
 *   out/
 *   ├── topology_sim_1/
 *   │   ├── csv/   (CSV data files)
 *   │   └── svg/   (SVG chart images)
 *   ├── topology_sim_2/
 *   │   ├── csv/
 *   │   └── svg/
 *   ├── topology_sim_3/
 *   │   ├── csv/
 *   │   └── svg/
 *   ├── topology_sim_4/
 *   │   ├── csv/
 *   │   └── svg/
 *   └── topology_sim_5/
 *       ├── csv/
 *       └── svg/
 * ============================================================================ */

/*
 * ensure_dir_exists — Create a directory if it doesn't already exist
 *
 * What it does:
 *   Checks if a path exists and is a directory. If it doesn't exist,
 *   creates it with 0777 permissions (readable/writable/executable by all).
 *
 * Parameters:
 *   path — Directory path to create (e.g., "out/topology_sim_1/csv")
 *
 * Returns:
 *   0  on success (directory exists or was created)
 *   -1 on failure (path is a file, not a directory, or mkdir failed)
 */
static int ensure_dir_exists(const char* path) {
    struct stat st;   /* stat struct filled by stat() system call */

    if (!path) {
        return -1;
    }

    /* Check if the path already exists */
    if (stat(path, &st) == 0) {
        /* Path exists — verify it's a directory (not a regular file) */
        return S_ISDIR(st.st_mode) ? 0 : -1;
    }

    /* Path doesn't exist — try to create it */
    if (mkdir(path, 0777) != 0 && errno != EEXIST) {
        return -1;   /* mkdir failed and it's not because it already exists */
    }

    return 0;
}


/*
 * ensure_output_dirs — Create the complete output directory hierarchy
 *
 * What it does:
 *   Creates the "out/" root directory and the "baseband/" and "rf/" subdirectories.
 *
 * Returns:
 *   0 on success, negative on failure
 */
static int ensure_output_dirs(void) {
    if (ensure_dir_exists(OUTPUT_ROOT_DIR) != 0) {
        return -1;
    }
    if (ensure_dir_exists(OUTPUT_ROOT_DIR "/baseband") != 0) {
        return -2;
    }
    if (ensure_dir_exists(OUTPUT_ROOT_DIR "/rf") != 0) {
        return -3;
    }
    return 0;
}

/*
 * clear_directory_contents — Delete all files inside a directory (but not the directory itself)
 *
 * What it does:
 *   Opens a directory, iterates through its entries, and removes each file.
 *
 * Parameters:
 *   path — Directory to clear (e.g., "out/csv")
 *
 * Returns:
 *   0 on success, negative on failure
 */
static int clear_directory_contents(const char* path) {
    DIR* dir;                  /* Directory handle */
    struct dirent* entry;      /* Individual directory entry */
    char entry_path[1024];     /* Full path to the entry (for deletion) */
    int written;

    if (!path) {
        return -1;
    }

    dir = opendir(path);
    if (!dir) {
        return -2;   /* Directory doesn't exist or can't be read */
    }

    /* Iterate through all entries in the directory */
    while ((entry = readdir(dir)) != NULL) {
        /* Skip the "." and ".." entries (current and parent directory) */
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        /* Build the full path to this entry */
        written = snprintf(entry_path, sizeof(entry_path), "%s/%s", path, entry->d_name);
        if (written < 0 || (size_t)written >= sizeof(entry_path)) {
            closedir(dir);
            return -3;   /* Path too long */
        }

        /* Delete the file */
        if (remove(entry_path) != 0) {
            closedir(dir);
            return -4;   /* Deletion failed (might be a subdirectory) */
        }
    }

    closedir(dir);
    return 0;
}


/* ============================================================================
 * UTILITY FUNCTIONS: UNIT CONVERSION
 * ============================================================================
 *
 * Decibels (dB) are a logarithmic scale commonly used in communications:
 *   - Gain of 20 dB means the signal is 100× stronger (10^(20/10) = 100)
 *   - Gain of -3 dB means the signal is about half as strong (10^(-3/10) ≈ 0.5)
 *   - 0 dB means no change (10^0 = 1)
 *
 * We constantly convert between dB (logarithmic) and linear scales because:
 *   - Humans think in dB (20 dB + 3 dB = 23 dB)
 *   - Math operations need linear (multiply: 100 × 2 = 200)
 * ============================================================================ */

/*
 * db_to_lin — Convert a value from decibels to linear scale
 *
 * Formula:  linear = 10^(dB / 10)
 *
 * Examples:
 *   db_to_lin(0)   → 1.0        (0 dB = no change)
 *   db_to_lin(10)  → 10.0       (10 dB = 10× amplification)
 *   db_to_lin(20)  → 100.0      (20 dB = 100× amplification)
 *   db_to_lin(-3)  → ~0.5       (-3 dB ≈ half power)
 *   db_to_lin(-10) → 0.1        (-10 dB = 10× attenuation)
 */
static double db_to_lin(double x_db) {
    return pow(10.0, x_db / 10.0);
}

/*
 * lin_to_db — Convert a value from linear scale to decibels
 *
 * Formula:  dB = 10 × log10(linear)
 *
 * Special case: if the linear value is ≤ 0, returns -infinity
 * (you can't take the log of zero or negative numbers)
 */
static double lin_to_db(double x_lin) {
    if (x_lin <= 0.0) {
        return -INFINITY;   /* -∞ dB = zero power (or negative = undefined) */
    }
    return 10.0 * log10(x_lin);
}


/* ============================================================================
 * 64-APSK CONSTELLATION CONSTRUCTION
 * ============================================================================
 *
 * A "constellation" is the set of all possible symbols a transmitter can send.
 * 64-APSK has 64 symbols, each represented as a complex number (I + jQ).
 *
 * Visually, on the I/Q plane (a 2D plot where X=I and Y=Q), the 64 symbols
 * form 4 concentric rings, each with a different number of evenly-spaced
 * points at different radii. This is specified by the DVB-S2X standard.
 * ============================================================================ */

/*
 * normalize_constellation — Scale all constellation points to have unit average power
 *
 * What it does:
 *   1. Calculates the mean power: P = (1/N) × Σ|c_i|² = (1/N) × Σ(I² + Q²)
 *   2. Divides every point by sqrt(P) so the new mean power is exactly 1.0
 *
 * Why it's needed:
 *   The DVB-S2X constellation is defined with specific ring radii (1.0, 1.88,
 *   2.72, 3.95), but the average power of these points is NOT 1.0.
 *   By normalizing to unit power (Es = 1), we can specify SNR directly — if
 *   the signal power is 1.0, then the noise power needed for 20 dB SNR is
 *   simply 1/100 = 0.01, regardless of the original constellation scaling.
 *
 * Parameters:
 *   c — Array of complex constellation points (MODIFIED in-place)
 *   m — Number of points (64 for 64-APSK)
 *
 * Returns:
 *   0 on success, -1 if c is NULL or m ≤ 0, -2 if mean power is zero
 */
static int normalize_constellation(Complex* c, int m) {
    double p = 0.0;   /* Running sum for mean power calculation */
    int i;

    if (!c || m <= 0) {
        return -1;
    }

    /* Calculate mean power: P = (1/m) × sum of |c_i|² */
    for (i = 0; i < m; ++i) {
        p += c[i].re * c[i].re + c[i].im * c[i].im;  /* |c_i|² = Re² + Im² */
    }
    p /= (double)m;

    if (p <= 0.0) {
        return -2;   /* All-zero constellation — physically meaningless */
    }

    /* Scale factor: multiply each point by 1/sqrt(P) to make average power = 1 */
    {
        const double s = 1.0 / sqrt(p);
        for (i = 0; i < m; ++i) {
            c[i].re *= s;
            c[i].im *= s;
        }
    }

    return 0;
}


/*
 * dvbs2x_quadrant_angle — Compute the actual angle for a DVB-S2X constellation point
 *
 * What it does:
 *   DVB-S2X uses quadrant symmetry: the constellation is defined for one
 *   quadrant (0 to π/2) and then mirrored into the other three quadrants.
 *   The bits p and q select which quadrant the point is in:
 *
 *   p=0, q=0: First quadrant  (0 to π/2)     → angle = +φ
 *   p=0, q=1: Second quadrant (mirrored)       → angle = -φ
 *   p=1, q=0: Third quadrant  (π to π/2)       → angle = π - φ
 *   p=1, q=1: Fourth quadrant (π to 3π/2)      → angle = π + φ
 *
 * Parameters:
 *   phi — Base angle within the first quadrant (radians)
 *   p   — Quadrant selector bit 1 (0 or 1)
 *   q   — Quadrant selector bit 2 (0 or 1)
 *
 * Returns:
 *   The actual angle (in radians) for the constellation point
 */
static double dvbs2x_quadrant_angle(double phi, unsigned int p, unsigned int q) {
    if (p == 0u && q == 0u) return phi;           /* 1st quadrant: angle as-is */
    if (p == 0u && q == 1u) return -phi;          /* 2nd quadrant: mirror across I-axis */
    if (p == 1u && q == 0u) return M_PI - phi;    /* 3rd quadrant: mirror across Q-axis */
    return M_PI + phi;                             /* 4th quadrant: rotate 180° */
}


/*
 * build_64apsk_constellation_dvbs2 — Generate the DVB-S2X 64-APSK constellation
 *
 * What it does:
 *   Constructs all 64 complex constellation points according to the DVB-S2X
 *   standard specification. The constellation has 4 concentric rings:
 *
 *   Ring 0: radius 1.00 — innermost ring
 *   Ring 1: radius 1.88
 *   Ring 2: radius 2.72
 *   Ring 3: radius 3.95 — outermost ring
 *
 *   Each point is identified by a 6-bit label (0 to 63). The bits of the label
 *   are interpreted as follows:
 *
 *   Bit layout: [a, b, p, q, u, v]
 *
 *   Bits [5:4] = ab → Select which ring (via ring_from_ab lookup table)
 *   Bit  [3]   = q  → Quadrant selector (combined with p)
 *   Bit  [2]   = p  → Quadrant selector (combined with q)
 *   Bits [1:0] = uv → Select phase position within ring (via phi_num_from_uv)
 *
 *   The phase angle for each point is:
 *     φ = phi_num_from_uv[uv] × π/16
 *   This gives phase positions at π/16, 3π/16, 5π/16, 7π/16 in the first quadrant.
 *
 *   The mapping tables ensure non-uniform angular spacing (different rings
 *   have 1, 3, 5, or 7 equally-spaced points per quadrant).
 *
 * After constructing all 64 points, normalize_constellation() is called to
 * set the average symbol energy to 1.0 (Es = 1).
 *
 * Parameters:
 *   out — Array of 64 Complex values to fill (must be pre-allocated)
 *
 * Returns:
 *   0 on success, -1 if out is NULL, -2 if normalization fails
 */
static int build_64apsk_constellation_dvbs2(Complex* out) {
    if (!out) return -1;

    /* DVB-S2 64-APSK: 4 rings of 4, 12, 20, 28 points */
    const int n_points[4] = {4, 12, 20, 28};
    const double ring_radii[4] = {1.0, 2.73, 4.52, 6.15};

    int idx = 0;
    for (int i = 0; i < 4; ++i) {
        int N = n_points[i];
        for (int k = 0; k < N; ++k) {
            double phase = (double)k * (2.0 * M_PI / N) + (M_PI / N);
            out[idx].re = ring_radii[i] * cos(phase);
            out[idx].im = ring_radii[i] * sin(phase);
            idx++;
        }
    }

    return normalize_constellation(out, 64);
}


/*
 * generate_symbols — Create a random sequence of transmitted symbols
 *
 * What it does:
 *   Simulates a transmitter randomly selecting symbols from the 64-APSK
 *   constellation. Each symbol is chosen uniformly at random (every symbol
 *   has a 1/64 chance of being picked).
 *
 * In a real system, the transmitter would encode user data bits into symbols
 * using a specific bit-to-symbol mapping. Here we use random selection because
 * we only care about signal quality metrics (SNR/EVM), not data throughput.
 *
 * Parameters:
 *   constellation — The 64-point I/Q constellation array
 *   m             — Number of constellation points (always 64)
 *   out           — Output array for the generated symbol sequence (n entries)
 *   labels        — Output array for the transmitted symbol indices (n entries)
 *                   labels[i] = index in [0..63] of the symbol selected for position i.
 *                   This could be used later for Symbol Error Rate (SER) analysis.
 *   n             — Number of symbols to generate
 */
static void generate_symbols(const Complex* constellation, int m, Complex* out, unsigned short* labels, size_t n) {
    size_t i;

    for (i = 0; i < n; ++i) {
        /*
         * Pick a random index between 0 and m-1 (inclusive).
         * prng_uint32() returns a random 32-bit integer; modulo m maps it
         * to the valid range. Since m=64 is a power of 2, there's zero bias.
         */
        const unsigned int idx = prng_uint32() % (unsigned int)m;

        /* Copy the selected constellation point into the output array */
        out[i] = constellation[idx];

        /* Record which symbol was transmitted (for potential SER analysis) */
        labels[i] = (unsigned short)idx;
    }
}


/* ============================================================================
 * SIGNAL POWER AND NOISE MEASUREMENT FUNCTIONS
 * ============================================================================
 *
 * These functions compute the key signal quality indicators by comparing
 * the "reference" (ideal, noise-free) signal to the "received" (degraded)
 * signal at each point in the receiver chain.
 * ============================================================================ */

/*
 * mean_power_complex — Calculate the average power of a complex signal
 *
 * Formula: P = (1/N) × Σ(I_i² + Q_i²)
 *
 * This is the mean squared magnitude of the complex signal.
 * For a unit-power constellation (Es = 1), this should return ≈ 1.0.
 *
 * Parameters:
 *   x — Array of complex samples
 *   n — Number of samples
 *
 * Returns:
 *   Mean power (always ≥ 0). Returns 0.0 if x is NULL or n is 0.
 */
static double mean_power_complex(const Complex* x, size_t n) {
    size_t i;
    double p = 0.0;

    if (!x || n == 0u) {
        return 0.0;
    }

    for (i = 0; i < n; ++i) {
        p += x[i].re * x[i].re + x[i].im * x[i].im;  /* |x_i|² = Re² + Im² */
    }
    return p / (double)n;   /* Average over all samples */
}


/*
 * mean_noise_power_complex — Calculate the noise power by comparing signal to reference
 *
 * Formula: P_noise = (1/N) × Σ|sig_i - ref_i|²
 *          = (1/N) × Σ((sig_I - ref_I)² + (sig_Q - ref_Q)²)
 *
 * This is the Mean Squared Error (MSE) between the received and reference signals.
 * It directly measures how much the signal has been corrupted by noise and
 * processing imperfections.
 *
 * Parameters:
 *   sig — Received (degraded) signal
 *   ref — Reference (ideal) signal
 *   n   — Number of samples
 *
 * Returns:
 *   Mean noise power (MSE). Returns 0.0 if signals are identical.
 */
static double mean_noise_power_complex(const Complex* sig, const Complex* ref, size_t n) {
    size_t i;
    double p = 0.0;

    if (!sig || !ref || n == 0u) {
        return 0.0;
    }

    for (i = 0; i < n; ++i) {
        const double dr = sig[i].re - ref[i].re;   /* I-component error */
        const double di = sig[i].im - ref[i].im;   /* Q-component error */
        p += dr * dr + di * di;                     /* Squared error magnitude */
    }
    return p / (double)n;
}


/*
 * mean_power_real — Calculate the average power of a real-valued signal
 *
 * Formula: P = (1/N) × Σ x_i²
 *
 * Used for RF signals, which are real-valued (not complex).
 */
static double mean_power_real(const double* x, size_t n) {
    size_t i;
    double p = 0.0;

    if (!x || n == 0u) {
        return 0.0;
    }

    for (i = 0; i < n; ++i) {
        p += x[i] * x[i];
    }
    return p / (double)n;
}


/*
 * mean_noise_power_real — Noise power for real signals (MSE between sig and ref)
 */
static double mean_noise_power_real(const double* sig, const double* ref, size_t n) {
    size_t i;
    double p = 0.0;

    if (!sig || !ref || n == 0u) {
        return 0.0;
    }

    for (i = 0; i < n; ++i) {
        const double d = sig[i] - ref[i];
        p += d * d;
    }
    return p / (double)n;
}


/*
 * add_awgn_complex — Add Additive White Gaussian Noise (AWGN) to a complex signal
 *
 * What it does:
 *   Adds random Gaussian noise to both the I and Q components of each sample.
 *   The noise has total power = noise_power across both components.
 *
 * Why σ = sqrt(noise_power / 2)?
 *   For a complex signal with independent I and Q noise:
 *     Total noise power = σ_I² + σ_Q² = σ² + σ²  = 2σ²
 *   So to achieve a total noise power of P:
 *     σ = sqrt(P/2)
 *
 * Parameters:
 *   x           — Complex signal array (MODIFIED in-place)
 *   n           — Number of samples
 *   noise_power — Desired total noise power to add (must be > 0)
 */
static void add_awgn_complex(Complex* x, size_t n, double noise_power) {
    size_t i;
    double sigma;

    if (!x || n == 0u || noise_power <= 0.0) {
        return;
    }

    sigma = sqrt(noise_power / 2.0);   /* Standard deviation per component */
    for (i = 0; i < n; ++i) {
        x[i].re += sigma * prng_gauss();  /* Add Gaussian noise to I component */
        x[i].im += sigma * prng_gauss();  /* Add Gaussian noise to Q component */
    }
}


/*
 * add_awgn_real — Add AWGN to a real-valued signal
 *
 * What it does:
 *   Adds Gaussian noise with standard deviation σ = sqrt(noise_power) to each sample.
 *   For real signals, the entire noise power goes into a single dimension.
 */
static void add_awgn_real(double* x, size_t n, double noise_power) {
    size_t i;
    double sigma;

    if (!x || n == 0u || noise_power <= 0.0) {
        return;
    }

    sigma = sqrt(noise_power);   /* σ for a single dimension */
    for (i = 0; i < n; ++i) {
        x[i] += sigma * prng_gauss();
    }
}


/* ============================================================================
 * VECTOR OPERATIONS (Copy, Scale, Filter)
 * ============================================================================ */

/* copy_complex — Memory copy of n complex values from src to dst */
static void copy_complex(Complex* dst, const Complex* src, size_t n) {
    if (!dst || !src || n == 0u) {
        return;
    }
    memcpy(dst, src, n * sizeof(Complex));
}

/* copy_real — Memory copy of n double values from src to dst */
static void copy_real(double* dst, const double* src, size_t n) {
    if (!dst || !src || n == 0u) {
        return;
    }
    memcpy(dst, src, n * sizeof(double));
}

/*
 * scale_complex — Multiply every sample by a constant amplitude factor
 *
 * This models amplification (amp > 1) or attenuation (amp < 1).
 * Both I and Q components are scaled equally.
 */
static void scale_complex(Complex* x, size_t n, double amp) {
    size_t i;

    if (!x || n == 0u) {
        return;
    }

    for (i = 0; i < n; ++i) {
        x[i].re *= amp;
        x[i].im *= amp;
    }
}

/* scale_real — Multiply every real sample by a constant */
static void scale_real(double* x, size_t n, double amp) {
    size_t i;

    if (!x || n == 0u) {
        return;
    }

    for (i = 0; i < n; ++i) {
        x[i] *= amp;
    }
}


/*
 * moving_average_complex — Causal sliding-window low-pass filter for complex signals
 *
 * What it does:
 *   Computes the average of each sample and its preceding (len-1) samples.
 *   This is equivalent to a rectangular FIR (Finite Impulse Response) low-pass
 *   filter with all coefficients equal to 1/len.
 *
 *   out[i] = average of in[max(0, i-len+1) ... i]
 *
 * Why a moving average?
 *   Real receiver components like Band-Pass Filters (BPF) and Low-Pass Filters
 *   (LPF) are modeled as moving-average filters because they average out
 *   high-frequency components and smooth the signal. The longer the filter
 *   (larger len), the more aggressive the smoothing.
 *
 * Efficient O(n) implementation:
 *   Instead of recalculating the sum of the window from scratch for every
 *   sample (O(n × len)), we maintain a running sum and add/remove one sample
 *   at each step (O(n)). This matters for the RF path where n can be millions.
 *
 * IMPORTANT: in and out must be DIFFERENT buffers (no in-place operation).
 *   If len ≤ 1, the input is copied to output unchanged (no filtering).
 *
 * Parameters:
 *   in  — Input signal (n complex samples)
 *   out — Output filtered signal (n complex samples, must differ from in)
 *   n   — Number of samples
 *   len — Filter length (window size). 1 = no filtering.
 */
static void moving_average_complex(const Complex* in, Complex* out, size_t n, int len) {
    const size_t win = (size_t)len;   /* Window size as unsigned */
    double sum_re = 0.0;              /* Running sum of I components in the window */
    double sum_im = 0.0;              /* Running sum of Q components in the window */
    size_t wcount = 0u;               /* Current number of samples in the window */
    size_t i;

    /* If no filtering needed, just copy input to output */
    if (!in || !out || n == 0u || len <= 1) {
        if (in && out && in != out) {
            memcpy(out, in, n * sizeof(Complex));
        }
        return;
    }

    for (i = 0u; i < n; ++i) {
        /* Add the newest sample to the running sum */
        sum_re += in[i].re;
        sum_im += in[i].im;
        ++wcount;

        /* If the window would exceed `len` samples, remove the oldest one */
        if (wcount > win) {
            sum_re -= in[i - win].re;
            sum_im -= in[i - win].im;
            --wcount;
        }

        /* Output is the average of all samples currently in the window */
        out[i].re = sum_re / (double)wcount;
        out[i].im = sum_im / (double)wcount;
    }
}

/*
 * bpf_moving_average_real — Causal Band-Pass Filter for RF signals
 * Acts as the RF equivalent of moving_average_complex centered at fc_hz.
 */
static void bpf_moving_average_real(const double* in, double* out, size_t n, int len, double fs_hz, double fc_hz) {
    size_t i;
    int k;
    double omega = 2.0 * M_PI * fc_hz / fs_hz;
    
    if (!in || !out || n == 0u || len <= 1) {
        if (in && out && in != out) {
            memcpy(out, in, n * sizeof(double));
        }
        return;
    }
    
    for (i = 0u; i < n; ++i) {
        double sum = 0.0;
        int limit = (i + 1 < (size_t)len) ? (int)(i + 1) : len;
        
        for (k = 0; k < limit; ++k) {
            /* Bandpass equivalent filter taps */
            double h = (2.0 / (double)len) * cos(omega * k);
            sum += in[i - k] * h;
        }
        out[i] = sum;
    }
}


/* ============================================================================
 * STAGE METRICS COMPUTATION
 * ============================================================================ */

/*
 * compute_metric_complex — Calculate SNR and EVM from reference vs. received complex signals
 *
 * What it does:
 *   Compares the clean reference signal to the degraded received signal and
 *   computes:
 *
 *   SNR (Signal-to-Noise Ratio):
 *     SNR_dB = 10 × log10(P_signal / P_noise)
 *     where P_signal = mean_power(ref), P_noise = MSE(sig, ref)
 *
 *   EVM (Error Vector Magnitude):
 *     EVM_% = sqrt(P_noise / P_signal) × 100
 *     This is the RMS of the error vectors, normalized by the signal amplitude.
 *     An EVM of 10% means the average error is 10% of the signal amplitude.
 *
 * Parameters:
 *   stage  — Name of the stage (stored in the metric, not used for computation)
 *   domain — Signal domain string (e.g., "complex_baseband", "rf_to_bb")
 *   ref    — Clean reference signal
 *   sig    — Degraded received signal
 *   n      — Number of samples
 *
 * Returns:
 *   A fully populated StageMetric struct
 */
static StageMetric compute_metric_complex(const char* stage, const char* domain, const Complex* ref, const Complex* sig, size_t n) {
    StageMetric m;

    m.stage = stage;
    m.domain = domain;
    m.signal_power = mean_power_complex(ref, n);
    m.noise_power = mean_noise_power_complex(sig, ref, n);

    /* Compute SNR and EVM, handling edge cases */
    if (m.signal_power > 0.0 && m.noise_power > 0.0) {
        m.snr_db = lin_to_db(m.signal_power / m.noise_power);     /* Normal case */
        m.evm_pct = sqrt(m.noise_power / m.signal_power) * 100.0;
    } else if (m.signal_power > 0.0 && m.noise_power == 0.0) {
        m.snr_db = INFINITY;   /* Perfect signal — no noise at all */
        m.evm_pct = 0.0;
    } else {
        m.snr_db = -INFINITY;  /* No signal — meaningless */
        m.evm_pct = NAN;       /* EVM undefined if there's no signal */
    }

    return m;
}


/*
 * compute_metric_real — Calculate SNR for real signals (EVM is undefined for real signals)
 *
 * Same as compute_metric_complex but for real-valued signals.
 * EVM is always set to NaN because it's only meaningful for I/Q constellation signals.
 */
static StageMetric compute_metric_real(const char* stage, const char* domain, const double* ref, const double* sig, size_t n) {
    StageMetric m;

    m.stage = stage;
    m.domain = domain;
    m.signal_power = mean_power_real(ref, n);
    m.noise_power = mean_noise_power_real(sig, ref, n);

    if (m.signal_power > 0.0 && m.noise_power > 0.0) {
        m.snr_db = lin_to_db(m.signal_power / m.noise_power);
    } else if (m.signal_power > 0.0 && m.noise_power == 0.0) {
        m.snr_db = INFINITY;
    } else {
        m.snr_db = -INFINITY;
    }
    m.evm_pct = NAN;  /* EVM is only defined for complex I/Q signals */

    return m;
}


/* ============================================================================
 * RECEIVER STAGE APPLICATION
 * ============================================================================
 *
 * These functions model what happens when a signal passes through one physical
 * component (stage) of the receiver. Each stage applies three operations:
 *
 *   1. FILTER: Smooth the signal (removes out-of-band noise)
 *   2. GAIN:   Amplify or attenuate the signal
 *   3. NOISE:  Add thermal noise based on the stage's Noise Figure (NF)
 *
 * The noise injection uses a "T0 reference tracker" (pn_t0_track) which models
 * the Friis noise formula behavior: noise added by later stages has less impact
 * because it's divided by the total gain of all preceding stages.
 * ============================================================================ */

/*
 * apply_stage_complex — Process a complex signal through one receiver stage
 *
 * Detailed sequence:
 *
 * STEP 1 — FILTER (optional, if filter_len > 1):
 *   Apply a moving-average low-pass filter to both reference and signal.
 *   The filter smooths the signal and reduces out-of-band noise.
 *   After filtering, the T0 tracker is adjusted proportionally to how much
 *   the filter reduced the noise power.
 *
 * STEP 2 — GAIN:
 *   Multiply both reference and signal by the voltage gain factor:
 *     voltage_gain = sqrt(10^(gain_db/10))
 *   Note: gain_db is a POWER gain in dB, but we need VOLTAGE gain (amplitude).
 *   Since power ∝ voltage², voltage_gain = sqrt(power_gain_linear).
 *   The T0 tracker is multiplied by the power gain (voltage_gain²).
 *
 * STEP 3 — NOISE INJECTION:
 *   Calculate excess noise from the stage's Noise Figure (NF):
 *     NF = the ratio of output SNR to input SNR for the stage.
 *     F = 10^(NF_dB/10)  — NF in linear scale
 *     If F < 1, clamp to 1 (NF < 0 dB is physically impossible)
 *
 *   Added noise power = pn_t0_track × (F - 1)
 *   where pn_t0_track is the reference noise power that accounts for
 *   all previous gains and filters. This makes later stages add less
 *   noise relative to the signal (Friis formula behavior).
 *
 * Parameters:
 *   stg          — Stage model (gain, NF, filter settings)
 *   ref          — Reference signal (MODIFIED in-place by filter and gain)
 *   sig          — Received signal (MODIFIED by filter, gain, AND noise)
 *   n            — Number of samples
 *   domain       — Signal domain string (for the metric label)
 *   pn_t0_track  — T0-referenced noise power tracker (MODIFIED in-place)
 *
 * Returns:
 *   StageMetric with SNR and EVM measured AFTER processing
 */
static StageMetric apply_stage_complex(const StageModel* stg, Complex* ref, Complex* sig, size_t n, const char* domain, double* pn_t0_track) {
    double amp_gain;         /* Voltage gain (linear) = sqrt(power_gain_linear) */
    double F;                /* Noise Figure in linear scale */
    double pn_before_in;     /* Noise power before filtering */
    double pn_after_filter;  /* Noise power after filtering */
    double pn_before;        /* Noise power before noise injection */
    double pn_add;           /* Additional noise power from this stage */

    /* Measure noise power before filtering (used to scale T0 tracker) */
    pn_before_in = mean_noise_power_complex(sig, ref, n);

    /* --- STEP 1: FILTER --- */
    if (stg->filter_len > 1) {
        /*
         * Allocate temporary buffers for filtered output.
         * We need separate buffers because the moving average filter
         * cannot operate in-place.
         */
        Complex* tmp_ref = (Complex*)calloc(n, sizeof(Complex));
        Complex* tmp_sig = (Complex*)calloc(n, sizeof(Complex));
        if (tmp_ref && tmp_sig) {
            /* Apply the moving-average filter to both reference and signal */
            moving_average_complex(ref, tmp_ref, n, stg->filter_len);
            moving_average_complex(sig, tmp_sig, n, stg->filter_len);

            /* Copy filtered results back into the original buffers */
            copy_complex(ref, tmp_ref, n);
            copy_complex(sig, tmp_sig, n);
        } else {
            /*
             * Memory allocation failed — skip filtering for this stage.
             * This is a graceful degradation: the simulation continues
             * without the filter, producing slightly optimistic noise results.
             */
            fprintf(stderr, "Warning: filter alloc failed for stage '%s'\n",
                    stg->name ? stg->name : "<unnamed>");
        }
        free(tmp_ref);
        free(tmp_sig);

        /*
         * Adjust the T0 tracker proportionally to the noise reduction from filtering.
         * If the filter reduced noise by X%, the tracker is also reduced by X%.
         */
        pn_after_filter = mean_noise_power_complex(sig, ref, n);
        if (pn_t0_track && pn_before_in > 0.0 && pn_after_filter >= 0.0) {
            *pn_t0_track *= (pn_after_filter / pn_before_in);
        }
    }

    /* --- STEP 2: GAIN --- */
    /*
     * Convert dB power gain to linear voltage amplitude:
     *   power_gain = 10^(gain_db / 10)
     *   voltage_gain = sqrt(power_gain) = 10^(gain_db / 20)
     */
    amp_gain = sqrt(db_to_lin(stg->gain_db));
    scale_complex(ref, n, amp_gain);     /* Scale reference (ideal) signal */
    scale_complex(sig, n, amp_gain);     /* Scale received (noisy) signal */

    /* Update T0 tracker for filter bandwidth changes, but DO NOT accumulate gain into the T0 reference. */
    /* (Gain affects the signal and already-added noise, but local stage noise only depends on local gain) */
    // removed: *pn_t0_track *= (amp_gain * amp_gain);

    /* --- STEP 3: NOISE INJECTION --- */
    pn_before = mean_noise_power_complex(sig, ref, n);

    /* Convert Noise Figure from dB to linear */
    F = db_to_lin(stg->nf_db);
    if (F < 1.0) {
        F = 1.0;  /* NF < 0 dB is physically impossible — clamp to 1 (perfect) */
    }

    /*
     * Calculate noise to add using the CORRECT Friis-like model:
     *   pn_add = pn_t0_track × local_gain × (F - 1)
     *
     * Local stage noise generated is independent of previous stages' gain!
     */
    if (pn_t0_track) {
        pn_add = (*pn_t0_track) * (amp_gain * amp_gain) * (F - 1.0);
    } else {
        pn_add = pn_before * (F - 1.0);
    }
    add_awgn_complex(sig, n, pn_add);   /* Actually inject the noise */

    /* --- STEP 4: LIMITER (Optional) --- */
    if (stg->name && strstr(stg->name, "limiter")) {
        size_t i;
        double max_amp = stg->target_vpp / 2.0;
        if (max_amp <= 0.0) max_amp = 1.0;
        for (i = 0; i < n; ++i) {
            double mag_ref = sqrt(ref[i].re*ref[i].re + ref[i].im*ref[i].im);
            double mag_sig = sqrt(sig[i].re*sig[i].re + sig[i].im*sig[i].im);
            
            if (mag_ref > max_amp && mag_ref > 0.0) {
                ref[i].re = (ref[i].re / mag_ref) * max_amp;
                ref[i].im = (ref[i].im / mag_ref) * max_amp;
            }
            if (mag_sig > max_amp && mag_sig > 0.0) {
                sig[i].re = (sig[i].re / mag_sig) * max_amp;
                sig[i].im = (sig[i].im / mag_sig) * max_amp;
            }
        }
    }

    /* Measure and return signal quality after this stage */
    return compute_metric_complex(stg->name, domain, ref, sig, n);
}


/*
 * apply_stage_real — Process a real (non-complex) signal through one receiver stage
 *
 * Identical logic to apply_stage_complex, but operates on double arrays
 * instead of Complex arrays. Used for the RF frontend path where signals
 * are real-valued (not I/Q complex pairs).
 *
 * EVM in the returned metric is always NaN (undefined for real signals).
 */
static StageMetric apply_stage_real(
    const StageModel* stg,
    double* ref,
    double* sig,
    size_t n,
    const char* domain,
    double* pn_t0_track,
    double fs_hz,
    double fc_hz) {
    double amp_gain;
    double F;
    double pn_before_in;
    double pn_after_filter;
    double pn_before;
    double pn_add;

    pn_before_in = mean_noise_power_real(sig, ref, n);

    /* STEP 1: FILTER */
    if (stg->filter_len > 1) {
        double* tmp_ref = (double*)calloc(n, sizeof(double));
        double* tmp_sig = (double*)calloc(n, sizeof(double));
        if (tmp_ref && tmp_sig) {
            bpf_moving_average_real(ref, tmp_ref, n, stg->filter_len, fs_hz, fc_hz);
            bpf_moving_average_real(sig, tmp_sig, n, stg->filter_len, fs_hz, fc_hz);
            copy_real(ref, tmp_ref, n);
            copy_real(sig, tmp_sig, n);
        } else {
            fprintf(stderr, "Warning: filter alloc failed for stage '%s'\n",
                    stg->name ? stg->name : "<unnamed>");
        }
        free(tmp_ref);
        free(tmp_sig);

        pn_after_filter = mean_noise_power_real(sig, ref, n);
        if (pn_t0_track && pn_before_in > 0.0 && pn_after_filter >= 0.0) {
            *pn_t0_track *= (pn_after_filter / pn_before_in);
        }
    }

    /* STEP 2: GAIN */
    amp_gain = sqrt(db_to_lin(stg->gain_db));
    scale_real(ref, n, amp_gain);
    scale_real(sig, n, amp_gain);
    /* removed: *pn_t0_track *= (amp_gain * amp_gain); */

    /* STEP 3: NOISE INJECTION */
    pn_before = mean_noise_power_real(sig, ref, n);
    F = db_to_lin(stg->nf_db);
    if (F < 1.0) {
        F = 1.0;
    }

    if (pn_t0_track) {
        pn_add = (*pn_t0_track) * (amp_gain * amp_gain) * (F - 1.0);
    } else {
        pn_add = pn_before * (F - 1.0);
    }
    add_awgn_real(sig, n, pn_add);

    /* --- STEP 4: LIMITER (Optional) --- */
    if (stg->name && strstr(stg->name, "limiter")) {
        size_t i;
        double max_amp = stg->target_vpp / 2.0;
        if (max_amp <= 0.0) max_amp = 1.0;
        for (i = 0; i < n; ++i) {
            if (ref[i] > max_amp) ref[i] = max_amp;
            else if (ref[i] < -max_amp) ref[i] = -max_amp;
            
            if (sig[i] > max_amp) sig[i] = max_amp;
            else if (sig[i] < -max_amp) sig[i] = -max_amp;
        }
    }

    return compute_metric_real(stg->name, domain, ref, sig, n);
}


/* ============================================================================
 * AUTO-GAIN HELPERS
 * ============================================================================ */

/*
 * complex_real_vpp — Calculate the peak-to-peak voltage of the I (real) component
 *
 * What it does:
 *   Finds the minimum and maximum values of the In-phase (real) component
 *   across all complex samples, and returns the difference (max - min).
 *   This is the "peak-to-peak voltage" — a measure of signal swing.
 *
 * Why only the I component?
 *   In many receiver designs, the final stage needs to present a signal
 *   with a specific voltage swing to the ADC (Analog-to-Digital Converter).
 *   The ADC's input range is defined in terms of peak-to-peak voltage.
 *   Using just the I component is sufficient because the I and Q channels
 *   are processed by independent ADCs.
 *
 * Parameters:
 *   x — Complex signal array
 *   n — Number of samples
 *
 * Returns:
 *   Peak-to-peak voltage of the I component (max_I - min_I)
 */
static double complex_real_vpp(const Complex* x, size_t n) {
    size_t i;
    double mn;   /* Minimum I value found */
    double mx;   /* Maximum I value found */

    if (!x || n == 0u) {
        return 0.0;
    }

    mn = x[0].re;
    mx = x[0].re;
    for (i = 1u; i < n; ++i) {
        if (x[i].re < mn) mn = x[i].re;
        if (x[i].re > mx) mx = x[i].re;
    }

    return mx - mn;
}


/*
 * gain_db_to_target_vpp — Calculate the gain (dB) needed to reach a target Vpp
 *
 * What it does:
 *   Given the current signal and a desired peak-to-peak voltage, computes
 *   how much gain (in dB) is needed to scale the signal to match the target.
 *
 * Formula: gain_dB = 20 × log10(target_vpp / current_vpp)
 *
 * Why 20 × log10 (not 10)?
 *   Because Vpp is a VOLTAGE quantity, not a POWER quantity. The relationship
 *   between voltage and dB uses 20×log10 (since power ∝ voltage²).
 *
 * Used by stages with auto_gain_to_vpp = 1. These stages automatically
 * calculate their gain at runtime to produce a specific output voltage.
 *
 * Parameters:
 *   x          — Current complex signal
 *   n          — Number of samples
 *   target_vpp — Desired peak-to-peak voltage (e.g., 1.0 V)
 *
 * Returns:
 *   Gain in dB needed to reach the target Vpp. Returns 0.0 if computation
 *   is impossible (e.g., zero signal).
 */
static double gain_db_to_target_vpp(const Complex* x, size_t n, double target_vpp) {
    const double current_vpp = complex_real_vpp(x, n);

    if (current_vpp <= 1e-15 || target_vpp <= 0.0) {
        return 0.0;   /* Can't compute — signal is essentially zero */
    }

    return 20.0 * log10(target_vpp / current_vpp);
}


/* ============================================================================
 * RF SIGNAL PROCESSING FUNCTIONS
 * ============================================================================
 *
 * These functions handle the conversion between complex baseband (I/Q)
 * signals and real RF (radio frequency) signals. This is the core of the
 * "brute-force RF" simulation path.
 * ============================================================================ */

/*
 * env_to_rf_real — Upconvert a complex baseband envelope to a real RF waveform
 *
 * What it does:
 *   Takes complex baseband samples (I + jQ) and modulates them onto an RF
 *   carrier at frequency fc_hz, producing a real-valued RF signal:
 *
 *     RF(t) = I(t) × cos(2πf_c t) − Q(t) × sin(2πf_c t)
 *
 *   This is standard IQ modulation (used in virtually all modern radios).
 *
 * Implementation:
 *   Instead of computing cos() and sin() for every sample (very expensive),
 *   we use a numerically controlled oscillator (NCO):
 *     - Start with osc = (1, 0) = cos(0) + j·sin(0)
 *     - Each sample: multiply by the rotation phasor w = (cos(Δθ), sin(Δθ))
 *     - This rotates the oscillator by one frequency step each sample
 *   This is much faster because complex multiplication is cheaper than
 *   transcendental functions (cos/sin).
 *
 * Numerical stability:
 *   Over millions of multiplications, the oscillator magnitude can drift
 *   from exactly 1.0 due to floating-point rounding errors. Every 1024
 *   samples, we re-normalize the oscillator to prevent this drift from
 *   accumulating and causing amplitude errors.
 *
 * Parameters:
 *   env    — Complex baseband envelope (I + jQ) array
 *   n      — Number of samples
 *   fs_hz  — Sampling frequency in Hz (e.g., 96 GHz = 96e9)
 *   fc_hz  — Carrier frequency in Hz (e.g., 24 GHz = 24e9)
 *   rf_out — Output real RF waveform buffer (pre-allocated, n elements)
 */
static void env_to_rf_real(const Complex* env, size_t n, double fs_hz, double fc_hz, double* rf_out) {
    size_t i;

    /* Phase increment per sample: how much the carrier rotates each sample */
    const double dtheta = 2.0 * M_PI * fc_hz / fs_hz;

    /* Pre-compute the rotation phasor (cos(Δθ) + j·sin(Δθ)) */
    const double w_re = cos(dtheta);   /* Real part of rotation phasor */
    const double w_im = sin(dtheta);   /* Imaginary part of rotation phasor */

    /* Oscillator state: starts at (1, 0) = cos(0) + j·sin(0) */
    double osc_re = 1.0;
    double osc_im = 0.0;

    for (i = 0; i < n; ++i) {
        /*
         * IQ modulation formula:
         *   RF[i] = env_I[i] × cos(ωt) − env_Q[i] × sin(ωt)
         *
         * where osc_re = cos(ωt) and osc_im = sin(ωt) from the NCO
         */
        rf_out[i] = env[i].re * osc_re - env[i].im * osc_im;

        /* Advance the oscillator by one sample (complex multiply by rotation phasor) */
        {
            const double next_re = osc_re * w_re - osc_im * w_im;
            const double next_im = osc_re * w_im + osc_im * w_re;
            osc_re = next_re;
            osc_im = next_im;
        }

        /* Re-normalize the oscillator every 1024 samples to prevent drift */
        if ((i & 1023u) == 1023u) {
            const double norm = hypot(osc_re, osc_im);  /* = sqrt(re² + im²) */
            if (norm > 0.0) {
                osc_re /= norm;
                osc_im /= norm;
            }
        }
    }
}


/*
 * mix_down_and_lowpass — Downconvert RF back to complex baseband with low-pass filtering
 *
 * What it does:
 *   This is the inverse of env_to_rf_real. It takes a real RF signal and
 *   recovers the original complex baseband (I + jQ) signal by:
 *
 *   1. MIXING (frequency translation):
 *      Multiply the RF signal by local oscillator signals at the same carrier frequency:
 *        I_raw = 2 × RF × cos(2πf_c t + φ_n(t))    — recovers the In-phase component
 *        Q_raw = −2 × RF × sin(2πf_c t + φ_n(t))   — recovers the Quadrature component
 *
 *      Added: Local Oscillator Phase Noise φ_n(t) modeled as a random walk (Wiener Process).
 *
 *   2. LOW-PASS FILTERING:
 *      The mixing process creates a copy of the signal centered at 2×f_c (the "image").
 *      A first-order exponential low-pass filter is applied to remove this image:
 *        y[n] = (1-α) × x[n] + α × y[n-1]
 *      where α = exp(−2π × f_cutoff / f_s)
 *
 *      The cutoff frequency defaults to fs/16 if not specified, but is typically
 *      set to 0.75 × Nyquist bandwidth (symbol_rate × (1 + rolloff)).
 *
 * Parameters:
 *   rf        — Input real RF signal
 *   n         — Number of samples
 *   fs_hz     — Sampling frequency (Hz)
 *   fc_hz     — Carrier frequency (Hz) — must match the transmitter's carrier
 *   cutoff_hz — Low-pass filter cutoff frequency (Hz). If ≤ 0, defaults to fs/16.
 *   bb_out    — Output complex baseband signal (pre-allocated, n elements)
 */
static void mix_down_and_lowpass(const double* rf, size_t n, double fs_hz, double fc_hz, double cutoff_hz, Complex* bb_out) {
    size_t i;

    /* Phase Noise parameters for LO */
    double phase_noise = 0.0;
    /* phase_step_stddev controls how fast the phase wanders. 
     * NOTE: Set to 0.0 currently. If > 0, calling this function separately on
     * the reference and signal streams will cause their phase random-walks to diverge,
     * drastically degrading SNR/EVM calculations. They must use the same LO phase stream. */
    const double phase_step_stddev = 0.0;

    /* NCO parameters */
    const double dtheta = 2.0 * M_PI * fc_hz / fs_hz;
    double current_theta = 0.0;

    /* Low-pass filter state (I and Q channels filtered independently) */
    double i_state = 0.0;   /* I channel filter state */
    double q_state = 0.0;   /* Q channel filter state */
    double alpha;            /* Filter smoothing coefficient */

    /* Default cutoff if not specified */
    if (cutoff_hz <= 0.0) {
        cutoff_hz = fs_hz / 16.0;
    }

    /*
     * Compute the first-order IIR filter coefficient:
     *   alpha = exp(-2π × f_cutoff / f_s)
     * Higher alpha → more smoothing → lower cutoff frequency
     * alpha close to 0 → almost no filtering
     */
    alpha = exp(-2.0 * M_PI * cutoff_hz / fs_hz);

    for (i = 0; i < n; ++i) {
        /* Update Phase Noise (Random Walk) */
        phase_noise += prng_gauss() * phase_step_stddev;

        /* Total instantaneous phase */
        double theta_total = current_theta + phase_noise;
        
        /* Oscillator components with phase noise */
        double lo_i = cos(theta_total);
        double lo_q = sin(theta_total);

        /*
         * Step 1: Mix the RF signal with local oscillator (cosine and -sine).
         * The factor of 2 compensates for the half-power lost in the mixing process.
         */
        const double i_raw = 2.0 * rf[i] * lo_i;    /* I = 2 × RF × cos(ωt + φ) */
        const double q_raw = -2.0 * rf[i] * lo_q;   /* Q = -2 × RF × sin(ωt + φ) */

        /*
         * Step 2: First-order exponential low-pass filter.
         * This is an IIR (Infinite Impulse Response) filter:
         *   y[n] = (1-α) × x[n] + α × y[n-1]
         * It smooths out the 2×fc image component while passing the baseband.
         */
        i_state = (1.0 - alpha) * i_raw + alpha * i_state;
        q_state = (1.0 - alpha) * q_raw + alpha * q_state;

        /* Store the filtered baseband output */
        bb_out[i].re = i_state;
        bb_out[i].im = q_state;

        /* Advance the pure NCO phase */
        current_theta += dtheta;
        /* Keep it within bounds to avoid precision loss on huge angles over time */
        if (current_theta > 2.0 * M_PI) {
            current_theta -= 2.0 * M_PI;
        }
    }
}

/*
 * repeat_symbols_to_env — Upsample symbols into a pulse-train envelope
 *
 * What it does:
 *   Converts N symbols into N×SPS samples by repeating each symbol SPS times.
 *   This is the simplest form of pulse shaping: rectangular (NRZ) pulses.
 *
 *   For example, with SPS=8 (samples per symbol), each symbol becomes 8
 *   identical samples. This creates a step-like waveform that can then be
 *   modulated onto the RF carrier.
 *
 * Parameters:
 *   symbols — Input symbol array (nsym complex values)
 *   nsym    — Number of symbols
 *   sps     — Samples Per Symbol (upsampling factor, e.g., 8)
 *   env     — Output envelope array (nsym × sps complex values, pre-allocated)
 */
static void repeat_symbols_to_env(const Complex* symbols, size_t nsym, int sps, Complex* env) {
    size_t i;

    for (i = 0; i < nsym; ++i) {
        int k;
        for (k = 0; k < sps; ++k) {
            /*
             * Each symbol is repeated SPS times.
             * env[i*sps + k] = symbols[i] for k = 0, 1, ..., sps-1
             */
            env[i * (size_t)sps + (size_t)k] = symbols[i];
        }
    }
}

/*
 * synchronize_and_downsample — Ideal open-loop synchronization
 * Finds the optimal symbol timing and phase rotation by cross-correlating
 * the received baseband stream with the known transmitted symbols.
 * Compensates for the deterministic group delay and phase offset introduced
 * by the RF and baseband causal filters.
 */
static void synchronize_and_downsample(const Complex* bb_stream, size_t nsym, int sps, const Complex* tx, Complex* out) {
    size_t i;
    int opt_offset = sps / 2;
    int offset;
    double max_corr = -1.0;
    double opt_phase = 0.0;
    double opt_scale = 1.0;
    double rot_re, rot_im;
    
    size_t n_tot = nsym * (size_t)sps;
    Complex* matched = (Complex*)malloc(n_tot * sizeof(Complex));
    if (!matched) return;
    
    /* 1. O(N) Matched Filter (Moving Average for Rectangular Pulses) */
    double sum_re = 0.0, sum_im = 0.0;
    for (i = 0; i < (size_t)sps && i < n_tot; ++i) {
        sum_re += bb_stream[i].re;
        sum_im += bb_stream[i].im;
    }
    
    for (i = 0; i < n_tot; ++i) {
        matched[i].re = sum_re / (double)sps;
        matched[i].im = sum_im / (double)sps;
        
        size_t sub_idx = i;
        size_t add_idx = i + sps;
        
        if (add_idx < n_tot) {
            sum_re += bb_stream[add_idx].re;
            sum_im += bb_stream[add_idx].im;
        }
        sum_re -= bb_stream[sub_idx].re;
        sum_im -= bb_stream[sub_idx].im;
    }
    
    /* 2. Search for optimal sampling offset */
    int max_offset = 4 * sps;
    if (n_tot < (size_t)max_offset) {
        max_offset = (int)n_tot;
    }
    
    for (offset = 0; offset < max_offset; ++offset) {
        double sum_re_corr = 0.0;
        double sum_im_corr = 0.0;
        int terms = 0;
        for (i = 0; i < nsym; ++i) {
            size_t idx = i * (size_t)sps + (size_t)offset;
            if (idx >= n_tot) break;
            Complex rx = matched[idx];
            /* tx * conj(rx) */
            sum_re_corr += rx.re * tx[i].re + rx.im * tx[i].im;
            sum_im_corr += rx.im * tx[i].re - rx.re * tx[i].im;
            terms++;
        }
        if (terms > 0) {
            double corr_mag = sqrt(sum_re_corr * sum_re_corr + sum_im_corr * sum_im_corr) / (double)terms;
            if (corr_mag > max_corr) {
                max_corr = corr_mag;
                opt_offset = offset;
                opt_phase = atan2(sum_im_corr, sum_re_corr);
            }
        }
    }
    
    {
        double corr_re = 0.0;
        double corr_im = 0.0;
        double rx_power = 0.0;
        int terms = 0;

        for (i = 0; i < nsym; ++i) {
            size_t idx = i * (size_t)sps + (size_t)opt_offset;
            if (idx >= n_tot) break;
            Complex rx = matched[idx];

            corr_re += rx.re * tx[i].re + rx.im * tx[i].im;
            corr_im += rx.im * tx[i].re - rx.re * tx[i].im;
            rx_power += rx.re * rx.re + rx.im * rx.im;
            terms++;
        }

        if (terms > 0 && rx_power > 0.0) {
            opt_scale = sqrt(corr_re * corr_re + corr_im * corr_im) / rx_power;
        }
    }

    /* 3. Apply optimal timing and phase rotation */
    rot_re = cos(opt_phase);
    rot_im = sin(opt_phase);
    
    for (i = 0; i < nsym; ++i) {
        size_t idx = i * (size_t)sps + (size_t)opt_offset;
        Complex rx = (idx < n_tot) ? matched[idx] : (Complex){0.0, 0.0};
        out[i].re = opt_scale * (rx.re * rot_re - rx.im * rot_im);
        out[i].im = opt_scale * (rx.re * rot_im + rx.im * rot_re);
    }
    
    free(matched);
}


/* ============================================================================
 * SIMULATION ENGINES
 * ============================================================================
 *
 * These are the two main simulation functions that orchestrate the entire
 * signal processing chain. Each one runs the signal through all stages,
 * measures quality, and writes output files.
 * ============================================================================ */

/*
 * simulate_complex_baseband — Run the fast analytical baseband simulation
 *
 * What it does:
 *   This is the "quick and clean" simulation path. It processes complex I/Q
 *   symbols directly through the baseband_rx chain without any RF modulation.
 *
 * Signal flow:
 *   1. Copy the transmitted symbols as both "reference" (clean) and "signal" (will be noisy)
 *   2. Add AWGN noise to the signal based on the configured input SNR
 *   3. Initialize the T0 noise tracker (for Friis-like stage noise behavior)
 *   4. Record the input constellation metrics
 *   5. For each stage in the baseband_rx chain:
 *      a. If auto_gain_to_vpp is set, compute the gain needed to reach target Vpp
 *      b. Apply the stage (filter → gain → noise injection)
 *      c. Write constellation CSV and SVG outputs
 *   6. Measure the final peak-to-peak voltage
 *
 * Parameters:
 *   cfg          — Simulation configuration (SNR, carrier, etc.)
 *   stage_cfg    — Loaded stage chain configuration from CSV
 *   tx_symbols   — Transmitted 64-APSK symbols (nsym values)
 *   nsym         — Number of symbols
 *   metrics      — Output array for stage metrics (caller provides MAX_METRICS buffer)
 *   metric_count — Output: number of metrics actually filled
 *   final_vpp    — Output: peak-to-peak voltage of the reference signal after all stages
 *   csv_run_dir  — Directory for CSV output files
 *   svg_run_dir  — Directory for SVG output files
 *
 * Returns:
 *   0 on success, -1 on memory error, -2 if chain is missing, -3 if MAX_METRICS exceeded
 */
static int simulate_complex_baseband(
    const SimConfig* cfg,
    const StageModelsConfig* stage_cfg,
    const Complex* tx_symbols,
    const Complex* constellation_template,
    size_t constellation_count,
    size_t nsym,
    StageMetric* metrics,
    size_t* metric_count,
    double* final_vpp,
    const char* csv_run_dir,
    const char* svg_run_dir) {

    const StageModel* stages = NULL;

    /* Get the baseband_rx stage chain */
    const size_t stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_BASEBAND_RX, &stages);
    size_t m = 0u;           /* Metric counter */
    size_t i;
    double pn_t0_track;     /* Friis noise reference tracker */
    Complex* ref;            /* Reference (clean) signal copy */
    Complex* sig;            /* Received (noisy) signal copy */

    if (!stages || stage_count == 0u || !csv_run_dir || !svg_run_dir) {
        return -2;
    }

    /* Check that we have enough metric slots */
    if (stage_count + 1u > MAX_METRICS) {
        fprintf(stderr,
                "baseband_rx has %zu stages; increase MAX_METRICS (currently %d)\n",
                stage_count, MAX_METRICS);
        return -3;
    }

    /* Allocate working copies of the signal */
    ref = (Complex*)calloc(nsym, sizeof(Complex));
    sig = (Complex*)calloc(nsym, sizeof(Complex));
    if (!ref || !sig) {
        free(ref);
        free(sig);
        return -1;   /* Memory allocation failed */
    }

    /* Copy transmitted symbols as both reference and signal */
    copy_complex(ref, tx_symbols, nsym);
    copy_complex(sig, tx_symbols, nsym);

    /* Add AWGN to the received signal at the specified input SNR level */
    {
        const double ps = mean_power_complex(ref, nsym);     /* Signal power */
        const double pn = ps / db_to_lin(cfg->input_snr_db); /* Noise power for desired SNR */
        add_awgn_complex(sig, nsym, pn);
    }

    /*
     * Initialize the T0 noise reference tracker.
     * If the antenna temperature differs from the reference temperature (290 K),
     * scale the tracker accordingly. This accounts for the fact that satellite
     * antennas typically see a much colder sky temperature (e.g., 150 K) than
     * the standard reference temperature of 290 K.
     */
    {
        const double pn_input = mean_noise_power_complex(sig, ref, nsym);
        if (cfg->antenna_temp_k > 1e-12) {
            pn_t0_track = pn_input * (cfg->t0_k / cfg->antenna_temp_k);
        } else {
            pn_t0_track = pn_input;
        }
    }

    /* Record the INPUT constellation (before any stages) */
    {
        metrics[m] = compute_metric_complex("input", "complex_baseband", ref, sig, nsym);
        write_constellation_stage_artifacts(
            csv_run_dir, svg_run_dir, "baseband", 0u, 1,
            "input", &metrics[m], "Baseband",
            constellation_template, constellation_count,
            ref, sig, nsym);
        write_complex_trace_stage_artifacts(
            svg_run_dir, "baseband", 0u, 1,
            "input", &metrics[m],
            sig, nsym, cfg->symbol_rate_hz);
        ++m;
    }

    /* Process the signal through each baseband_rx stage */
    for (i = 0u; i < stage_count; ++i) {
        StageModel stage = stages[i];   /* Make a copy (may modify gain_db) */

        /*
         * Auto-gain: if this stage has auto_gain_to_vpp enabled, calculate
         * the gain needed to scale the REFERENCE signal to the target Vpp.
         * Using the reference (not the noisy signal) ensures consistent
         * gain regardless of noise level.
         */
        if (stage.auto_gain_to_vpp) {
            const double target_vpp = (stage.target_vpp > 0.0) ? stage.target_vpp : 1.0;
            stage.gain_db = gain_db_to_target_vpp(ref, nsym, target_vpp);
        }

        /* Apply the stage and record the output metric */
        metrics[m] = apply_stage_complex(&stage, ref, sig, nsym, "complex_baseband", &pn_t0_track);

        /* Write constellation artifacts (CSV data + SVG plot) */
        write_constellation_stage_artifacts(
            csv_run_dir, svg_run_dir, "baseband", i + 1u, 0,
            stage.name, &metrics[m], "Baseband",
            constellation_template, constellation_count,
            ref, sig, nsym);
            
        write_complex_trace_stage_artifacts(
            svg_run_dir, "baseband", i + 1u, 0,
            stage.name, &metrics[m],
            sig, nsym, cfg->symbol_rate_hz);
        ++m;
    }

    /* Report the final peak-to-peak voltage (of the clean reference) */
    *final_vpp = complex_real_vpp(ref, nsym);
    *metric_count = m;

    /* Clean up */
    free(ref);
    free(sig);
    return 0;
}


/*
 * simulate_bruteforce_rf — Run the realistic RF upconversion/downconversion simulation
 *
 * What it does:
 *   This is the full "brute-force" simulation that models the actual RF signal
 *   path: upconvert to 24 GHz, process through RF stages, downconvert, then
 *   process through post-mixer baseband stages.
 *
 * Signal flow:
 *   1. Upsample symbols to RF sample rate (repeat each symbol SPS times)
 *   2. IQ-modulate onto 24 GHz carrier → real RF signal
 *   3. Add AWGN to the real RF signal
 *   4. Initialize the RF T0 noise tracker
 *   5. Record input RF trace metric
 *   6. Process through rf_frontend stages (real signal domain)
 *   7. Downconvert RF → complex baseband (mix + lowpass filter)
 *   8. Extract symbol-rate samples from the oversampled baseband
 *   9. Record the downconversion metric
 *  10. Process through rf_postmix_bb stages (complex baseband domain)
 *  11. Record the final VPP and metrics
 *
 * Sampling rate calculation:
 *   SPS = round(rf_sample_rate / symbol_rate)
 *   Minimum SPS = 8 (to satisfy Nyquist: fs > 2×fc requires SPS ≥ 4,
 *   but 8 provides better anti-aliasing margin)
 *   Example: symbol_rate=10 MHz, rf_fs=96 GHz → SPS = 9600
 *            Total RF samples = 256 symbols × 9600 = 2,457,600 samples
 *
 * Parameters:
 *   Same as simulate_complex_baseband, plus:
 *   used_sps   — Output: samples per symbol actually used
 *   used_fs_hz — Output: actual RF sampling frequency used (= symbol_rate × SPS)
 *
 * Returns:
 *   0 on success, -1 on memory error, -2 if chains missing, -3 if MAX_METRICS exceeded
 */
static int simulate_bruteforce_rf(
    const SimConfig* cfg,
    const StageModelsConfig* stage_cfg,
    const Complex* tx_symbols,
    const Complex* constellation_template,
    size_t constellation_count,
    size_t nsym,
    StageMetric* metrics,
    size_t* metric_count,
    double* final_vpp,
    int* used_sps,
    double* used_fs_hz,
    const char* csv_run_dir,
    const char* svg_run_dir) {

    const StageModel* rf_stages = NULL;
    const StageModel* bb_stages = NULL;
    const size_t rf_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_FRONTEND, &rf_stages);
    const size_t bb_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_POSTMIX_BB, &bb_stages);

    int sps;                  /* Samples Per Symbol */
    double fs_hz;             /* Actual RF sampling frequency */
    size_t nrf;               /* Total number of RF samples = nsym × sps */
    size_t m = 0u;            /* Metric counter */
    size_t i;
    double pn_t0_track_rf;   /* T0 tracker for RF stages */
    double pn_t0_track_post; /* T0 tracker for post-mixer BB stages */
    double pn_before_mix;    /* Noise power before downconversion */
    double pn_after_mix;     /* Noise power after downconversion */

    /* Working buffers */
    Complex* env = NULL;          /* Upsampled complex envelope */
    double* rf_ref = NULL;        /* Reference RF waveform (clean) */
    double* rf_sig = NULL;        /* Received RF waveform (noisy) */
    Complex* bb_ref_stream = NULL; /* Downconverted reference at RF sample rate */
    Complex* bb_sig_stream = NULL; /* Downconverted signal at RF sample rate */
    Complex* ref_sym = NULL;      /* Downsampled reference at symbol rate */
    Complex* sig_sym = NULL;      /* Downsampled signal at symbol rate */

    if (!rf_stages || !bb_stages || rf_stage_count == 0u || bb_stage_count == 0u || !csv_run_dir || !svg_run_dir) {
        return -2;
    }

    /* Check metric buffer capacity */
    if (rf_stage_count + bb_stage_count + 2u > (size_t)MAX_METRICS) {
        fprintf(stderr,
                "RF chain has %zu+%zu stages; increase MAX_METRICS (currently %d)\n",
                rf_stage_count, bb_stage_count, MAX_METRICS);
        return -3;
    }

    /* Calculate samples per symbol from the configured RF sampling rate */
    sps = (int)llround(cfg->rf_sample_rate_hz / cfg->symbol_rate_hz);
    if (sps < 8) {
        sps = 8;   /* Minimum 8x oversampling for Nyquist margin */
    }
    fs_hz = cfg->symbol_rate_hz * (double)sps;   /* Actual sampling frequency */
    nrf = nsym * (size_t)sps;                      /* Total RF samples */

    /* Allocate all working buffers */
    env = (Complex*)calloc(nrf, sizeof(Complex));
    rf_ref = (double*)calloc(nrf, sizeof(double));
    rf_sig = (double*)calloc(nrf, sizeof(double));
    bb_ref_stream = (Complex*)calloc(nrf, sizeof(Complex));
    bb_sig_stream = (Complex*)calloc(nrf, sizeof(Complex));
    ref_sym = (Complex*)calloc(nsym, sizeof(Complex));
    sig_sym = (Complex*)calloc(nsym, sizeof(Complex));

    if (!env || !rf_ref || !rf_sig || !bb_ref_stream || !bb_sig_stream || !ref_sym || !sig_sym) {
        /* Clean up on allocation failure */
        free(env); free(rf_ref); free(rf_sig);
        free(bb_ref_stream); free(bb_sig_stream);
        free(ref_sym); free(sig_sym);
        return -1;
    }

    /* Step 1: Upsample symbols to RF rate (rectangular pulses) */
    repeat_symbols_to_env(tx_symbols, nsym, sps, env);

    /* Step 2: IQ-modulate onto 24 GHz carrier → real RF waveform */
    env_to_rf_real(env, nrf, fs_hz, cfg->carrier_hz, rf_ref);
    copy_real(rf_sig, rf_ref, nrf);  /* Copy as received signal (will add noise next) */

    /* Step 3: Add AWGN to the RF signal */
    {
        const double ps = mean_power_real(rf_ref, nrf);
        /* 
         * FIX Physics: RF spans a much wider bandwidth (fs/2) than the baseband signal (symbol_rate).
         * To maintain the identical noise power density (and thus identical SNR after
         * downconversion), the total noise power in the RF band must be scaled up
         * by the ratio of (fs / symbol_rate). Since sps = fs / symbol_rate:
         */
        const double pn = (ps * (double)sps) / (2.0 * db_to_lin(cfg->input_snr_db));
        add_awgn_real(rf_sig, nrf, pn);
    }

    /* Step 4: Initialize the RF T0 noise tracker */
    {
        const double pn_input = mean_noise_power_real(rf_sig, rf_ref, nrf);
        if (cfg->antenna_temp_k > 1e-12) {
            pn_t0_track_rf = pn_input * (cfg->t0_k / cfg->antenna_temp_k);
        } else {
            pn_t0_track_rf = pn_input;
        }
    }

    /* Step 5: Record the INPUT RF trace metric */
    {
        metrics[m] = compute_metric_real("input_rf", "rf_real", rf_ref, rf_sig, nrf);
        write_trace_stage_artifacts(
            csv_run_dir, svg_run_dir, "rf", 0u, 1, "input_rf",
            &metrics[m], "RF", rf_ref, rf_sig, nrf, 6000u, fs_hz);
            
        {
            Complex* temp_bb_ref = (Complex*)calloc(nrf, sizeof(Complex));
            Complex* temp_bb_sig = (Complex*)calloc(nrf, sizeof(Complex));
            Complex* temp_ref_sym = (Complex*)calloc(nsym, sizeof(Complex));
            Complex* temp_sig_sym = (Complex*)calloc(nsym, sizeof(Complex));
            
            if (temp_bb_ref && temp_bb_sig && temp_ref_sym && temp_sig_sym) {
                const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
                mix_down_and_lowpass(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz, temp_bb_ref);
                mix_down_and_lowpass(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz, temp_bb_sig);

                synchronize_and_downsample(temp_bb_ref, nsym, sps, tx_symbols, temp_ref_sym);
                synchronize_and_downsample(temp_bb_sig, nsym, sps, tx_symbols, temp_sig_sym);

                StageMetric temp_metric = compute_metric_complex("input_rf", "rf_to_bb", temp_ref_sym, temp_sig_sym, nsym);
                metrics[m].evm_pct = temp_metric.evm_pct;
                metrics[m].snr_db = temp_metric.snr_db;

                write_constellation_stage_artifacts(
                    csv_run_dir, svg_run_dir, "rf", 0u, 1,
                    "input_rf", &metrics[m], "RF",
                    constellation_template, constellation_count,
                    temp_ref_sym, temp_sig_sym, nsym);
            }
            if (temp_bb_ref) free(temp_bb_ref);
            if (temp_bb_sig) free(temp_bb_sig);
            if (temp_ref_sym) free(temp_ref_sym);
            if (temp_sig_sym) free(temp_sig_sym);
        }
        
        ++m;
    }

    /* Step 6: Process through RF frontend stages */
    for (i = 0u; i < rf_stage_count; ++i) {
        StageModel stage = rf_stages[i];

        metrics[m] = apply_stage_real(&stage, rf_ref, rf_sig, nrf, "rf_real", &pn_t0_track_rf, fs_hz, cfg->carrier_hz);
        write_trace_stage_artifacts(
            csv_run_dir, svg_run_dir, "rf", i + 1u, 0, stage.name,
            &metrics[m], "RF", rf_ref, rf_sig, nrf, 6000u, fs_hz);
            
        /* Generate Constellation & EVM for all RF Stages */
        if (i < rf_stage_count) {
            Complex* temp_bb_ref = (Complex*)calloc(nrf, sizeof(Complex));
            Complex* temp_bb_sig = (Complex*)calloc(nrf, sizeof(Complex));
            Complex* temp_ref_sym = (Complex*)calloc(nsym, sizeof(Complex));
            Complex* temp_sig_sym = (Complex*)calloc(nsym, sizeof(Complex));
            
            if (temp_bb_ref && temp_bb_sig && temp_ref_sym && temp_sig_sym) {
                const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
                mix_down_and_lowpass(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz, temp_bb_ref);
                mix_down_and_lowpass(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz, temp_bb_sig);

                synchronize_and_downsample(temp_bb_ref, nsym, sps, tx_symbols, temp_ref_sym);
                synchronize_and_downsample(temp_bb_sig, nsym, sps, tx_symbols, temp_sig_sym);

                StageMetric temp_metric = compute_metric_complex(stage.name, "rf_to_bb", temp_ref_sym, temp_sig_sym, nsym);
                metrics[m].evm_pct = temp_metric.evm_pct;
                metrics[m].snr_db = temp_metric.snr_db;

                write_constellation_stage_artifacts(
                    csv_run_dir, svg_run_dir, "rf", i + 1u, 0,
                    stage.name, &metrics[m], "RF",
                    constellation_template, constellation_count,
                    temp_ref_sym, temp_sig_sym, nsym);
            }
            if (temp_bb_ref) free(temp_bb_ref);
            if (temp_bb_sig) free(temp_bb_sig);
            if (temp_ref_sym) free(temp_ref_sym);
            if (temp_sig_sym) free(temp_sig_sym);
        }
        
        ++m;
    }

    /* Record noise power before downconversion (for T0 tracker continuity) */
    pn_before_mix = mean_noise_power_real(rf_sig, rf_ref, nrf);

    /* Step 7: Downconvert RF → complex baseband */
    {
        /*
         * Low-pass cutoff = 0.75 × Nyquist bandwidth.
         * Nyquist BW = symbol_rate × (1 + rolloff).
         * The 0.75 factor provides margin to reject the image band while
         * keeping all in-band signal energy.
         */
        const double cutoff_hz = 5.0 * cfg->symbol_rate_hz; /* Wide enough to avoid ISI, matched filter provides noise rejection */
        mix_down_and_lowpass(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz, bb_ref_stream);
        mix_down_and_lowpass(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz, bb_sig_stream);
    }

    /* Step 8: Extract symbol-rate samples to record the immediate downconversion metric */
    synchronize_and_downsample(bb_ref_stream, nsym, sps, tx_symbols, ref_sym);
    synchronize_and_downsample(bb_sig_stream, nsym, sps, tx_symbols, sig_sym);

    /* Update the T0 tracker across the downconversion boundary */
    pn_after_mix = mean_noise_power_complex(sig_sym, ref_sym, nsym);
    pn_t0_track_post = pn_t0_track_rf;
    if (pn_before_mix > 0.0 && pn_after_mix >= 0.0) {
        pn_t0_track_post *= (pn_after_mix / pn_before_mix);
    }

    /* Step 9: Record the downconversion metric and constellation */
    metrics[m] = compute_metric_complex("MIX2_Downconv", "rf_to_bb", ref_sym, sig_sym, nsym);
    write_constellation_stage_artifacts(
        csv_run_dir, svg_run_dir, "rf", rf_stage_count + 1u, 0,
        metrics[m].stage, &metrics[m], "RF",
        constellation_template, constellation_count,
        ref_sym, sig_sym, nsym);
    write_complex_trace_stage_artifacts(
        svg_run_dir, "rf", rf_stage_count + 1u, 0,
        metrics[m].stage, &metrics[m],
        sig_sym, nsym, cfg->symbol_rate_hz);
    ++m;

    /* Step 10: Process through post-mixer BB stages (on the continuous oversampled stream!) */
    for (i = 0u; i < bb_stage_count; ++i) {
        StageModel stage = bb_stages[i];

        if (stage.auto_gain_to_vpp) {
            const double target_vpp = (stage.target_vpp > 0.0) ? stage.target_vpp : 1.0;
            /* Gain is calculated on the oversampled reference */
            stage.gain_db = gain_db_to_target_vpp(bb_ref_stream, nrf, target_vpp);
        }

        /* 
         * ARCHITECTURAL INVERSION:
         * We apply the hardware filters (Lowpass/Amp) to the CONTINUOUS, highly-oversampled 
         * waveform (bb_sig_stream), NOT the crushed 1-sps symbols.
         */
        apply_stage_complex(&stage, bb_ref_stream, bb_sig_stream, nrf, "rf_to_bb", &pn_t0_track_post);

        /* 
         * Now extract the symbol centers from the cleanly filtered waveform
         * to compute the metrics and generate the plots.
         */
        synchronize_and_downsample(bb_ref_stream, nsym, sps, tx_symbols, ref_sym);
        synchronize_and_downsample(bb_sig_stream, nsym, sps, tx_symbols, sig_sym);

        metrics[m] = compute_metric_complex(stage.name, "rf_to_bb", ref_sym, sig_sym, nsym);

        write_constellation_stage_artifacts(
            csv_run_dir, svg_run_dir, "rf", rf_stage_count + 2u + i, 0,
            stage.name, &metrics[m], "RF",
            constellation_template, constellation_count,
            ref_sym, sig_sym, nsym);
        write_complex_trace_stage_artifacts(
            svg_run_dir, "rf", rf_stage_count + 2u + i, 0,
            stage.name, &metrics[m],
            sig_sym, nsym, cfg->symbol_rate_hz);
        ++m;
    }



    /* Step 11: Report results */
    *final_vpp = complex_real_vpp(ref_sym, nsym);
    *metric_count = m;
    *used_sps = sps;
    *used_fs_hz = fs_hz;

    /* Clean up all working buffers */
    free(env);
    free(rf_ref);
    free(rf_sig);
    free(bb_ref_stream);
    free(bb_sig_stream);
    free(ref_sym);
    free(sig_sym);
    return 0;
}


/*
 * print_metrics — Display a formatted table of cumulative end-to-end stage metrics on the console
 *
 * What it does:
 *   Prints a table with columns: Stage, Domain, SNR(dB), EVM(%), NoisePow
 *   Each row shows the signal quality at one point in the receiver chain.
 *
 * Parameters:
 *   title   — Section heading (printed above the table)
 *   metrics — Array of StageMetric structs
 *   count   — Number of entries to print
 */
static void print_metrics(const char* title, const StageMetric* metrics, size_t count) {
    size_t i;

    printf("\n%s\n", title);
    printf("%-18s %-14s %12s %12s %12s\n", "Stage", "Domain", "SNR(dB)", "EVM(%)", "NoisePow");
    for (i = 0; i < count; ++i) {
        char stage_label[128];
        humanize_stage_name(metrics[i].stage, stage_label, sizeof(stage_label));
        {
            char numbered_label[256];
            snprintf(numbered_label, sizeof(numbered_label), "%zu. %s", i + 1u, stage_label);
            strncpy(stage_label, numbered_label, sizeof(stage_label));
            stage_label[sizeof(stage_label) - 1u] = '\0';
        }
        printf(
            "%-18s %-14s %12.3f %12.3f %12.4e\n",
            stage_label,
            metrics[i].domain,
            metrics[i].snr_db,
            metrics[i].evm_pct,
            metrics[i].noise_power);
    }
}


/* ============================================================================
 * COMMAND-LINE ARGUMENT PARSING
 * ============================================================================ */

/*
 * parse_u32 — Parse a string as an unsigned 32-bit integer with validation
 *
 * Returns: 0 on success, -1 if NULL, -2 if not a valid unsigned integer
 */
static int parse_u32(const char* s, unsigned int* out) {
    char* end = NULL;
    unsigned long v;

    if (!s || !out) return -1;

    errno = 0;
    v = strtoul(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0' || v > UINT_MAX) {
        return -2;
    }

    *out = (unsigned int)v;
    return 0;
}

/*
 * parse_i32 — Parse a string as a signed 32-bit integer with validation
 */
static int parse_i32(const char* s, int* out) {
    char* end = NULL;
    long v;

    if (!s || !out) return -1;

    errno = 0;
    v = strtol(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0' || v < INT_MIN || v > INT_MAX) {
        return -2;
    }

    *out = (int)v;
    return 0;
}

/*
 * parse_double — Parse a string as a double-precision float with validation
 */
static int parse_double(const char* s, double* out) {
    char* end = NULL;
    double v;

    if (!s || !out) return -1;

    errno = 0;
    v = strtod(s, &end);
    if (errno != 0 || end == s || *end != '\0') {
        return -2;
    }

    *out = v;
    return 0;
}


/*
 * resolve_stage_csv_path — Resolve a stage CSV path (file or directory)
 *
 * What it does:
 *   If the user provides a DIRECTORY path (e.g., "stage_models/"), this function
 *   resolves it to the specific "runtime_stage_models.csv" file inside that
 *   directory. If the user provides a FILE path, it's used as-is.
 *
 *   This exists because the stage_models/ directory may contain multiple CSV files:
 *   - stage_models.csv               — design reference / topology sketch
 *   - runtime_stage_models.csv       — simulator-ready canonical configuration
 *   Automatically choosing the runtime file prevents accidentally loading
 *   the design sketch (which may have incomplete data).
 *
 * Parameters:
 *   input_path    — User-provided path (file or directory)
 *   resolved_path — Output buffer for the resolved file path
 *   resolved_size — Size of the output buffer
 *
 * Returns:
 *   0 on success, negative on error
 */
static int resolve_stage_csv_path(const char* input_path, char* resolved_path, size_t resolved_size) {
    struct stat st;
    int written;
    size_t input_len;

    if (!input_path || !resolved_path || resolved_size == 0u) {
        return -1;
    }

    /* Check if the input is a directory */
    if (stat(input_path, &st) == 0 && S_ISDIR(st.st_mode)) {
        /* Strip trailing slashes from the directory path */
        input_len = strlen(input_path);
        while (input_len > 1u && input_path[input_len - 1u] == '/') {
            --input_len;
        }

        /* Append the runtime CSV filename */
        written = snprintf(resolved_path, resolved_size, "%.*s/runtime_stage_models.csv", (int)input_len, input_path);
        if (written < 0 || (size_t)written >= resolved_size) {
            return -2;   /* Path too long */
        }

        /* Verify the resolved file exists */
        if (stat(resolved_path, &st) != 0) {
            return -3;   /* File doesn't exist */
        }

        return 0;
    }

    /* Not a directory — use the path as-is */
    written = snprintf(resolved_path, resolved_size, "%s", input_path);
    if (written < 0 || (size_t)written >= resolved_size) {
        return -4;
    }

    return 0;
}


/*
 * print_usage — Display command-line usage help
 */
static void print_usage(const char* exe) {
    printf("Usage: %s [options]\n", exe);
    printf("Options:\n");
    printf("  --seed <int>          RNG seed (default: time-based)\n");
    printf("  --symbols <int>       Number of symbols (default: 256)\n");
    printf("  --symbol-rate <Hz>    Symbol rate in Hz (default: 1e7)\n");
    printf("  --rf-fs <Hz>          RF brute-force sample rate in Hz (default: 9.6e10)\n");
    printf("  --carrier <Hz>        Carrier frequency in Hz (default: 2.4e10)\n");
    printf("  --snr <dB>            Input SNR in dB (default: 20)\n");
    printf("  --stage-csv <path>    Stage-model CSV file or folder (default: stage_models/runtime_stage_models_target16.csv)\n");
    printf("  --topology-sim <1..4> Output simulation slot under out/topology_sim_N (default: 1)\n");
    printf("  --stage-sim <1..4>    Alias for --topology-sim\n");
}


/* ============================================================================
 * MAIN — Program Entry Point
 * ============================================================================ */

/*
 * main — Orchestrate the complete 64-APSK receiver simulation
 *
 * Execution sequence:
 *   1.  Parse command-line arguments (--seed, --symbols, --snr, etc.)
 *   2.  Initialize the PRNG with the seed
 *   3.  Create/verify output directory structure
 *   4.  Resolve the stage-model CSV path
 *   5.  Build the DVB-S2X 64-APSK constellation (64 fixed I/Q points)
 *   6.  Generate a random transmitted symbol sequence
 *   7.  Calculate the link budget (thermal noise power + signal power)
 *   8.  Write input budget CSV and SVG
 *   9.  Load receiver stage configuration from CSV
 *  10.  Run complex-baseband simulation
 *  11.  Run brute-force RF simulation
 *  12.  Write stage metrics CSV and SVG for both paths
 *  13.  Print console summary
 *  14.  Free all allocated memory and exit
 *
 * Exit codes:
 *   0 = Success
 *   1 = Usage error (unknown argument)
 *   2 = Argument parse error (invalid number for --seed, --snr, etc.)
 *   3 = Constellation build error
 *   4 = Memory allocation error
 *   5 = CSV load error or directory error
 *   6 = Complex-baseband simulation error
 *   7 = Brute-force RF simulation error
 *   8 = Output directory creation error
 */
int main(int argc, char** argv) {
    SimConfig cfg;                    /* Top-level simulation parameters */
    StageModelsConfig stage_cfg;      /* Loaded stage chain configuration */
    char stage_csv_path[512] = "stage_models/runtime_stage_models_target16.csv";  /* Default CSV path */
    char resolved_stage_csv_path[512]; /* Resolved path after directory resolution */
    char stage_err[256];              /* Error message buffer for CSV loading */
    Complex constellation[64];        /* The 64-APSK constellation (stack-allocated) */
    Complex* tx_symbols;              /* Transmitted symbol sequence (heap-allocated) */
    unsigned short* labels;           /* Transmitted symbol indices (for potential SER) */
    StageMetric metrics_bb[MAX_METRICS];  /* Baseband path metrics */
    StageMetric metrics_rf[MAX_METRICS];  /* RF path metrics */
    size_t count_bb = 0u;             /* Number of baseband metrics */
    size_t count_rf = 0u;             /* Number of RF metrics */
    double final_vpp_bb = 0.0;        /* Final Vpp after baseband path */
    double final_vpp_rf = 0.0;        /* Final Vpp after RF path */
    int rf_sps = 0;                   /* RF samples per symbol (output) */
    double rf_fs_used = 0.0;          /* RF sampling frequency used (output) */
    int i;                             /* Loop variable for CLI parsing */

    /* Link budget variables */
    double noise_bw_hz;               /* Noise bandwidth = symbol_rate × (1 + rolloff) */
    double noise_w;                   /* Thermal noise power in watts */
    double noise_dbm;                 /* Noise power in dBm */
    double signal_dbm;                /* Signal power in dBm */
    int topology_sim_id;              /* Output simulation slot (1–4) */

    /* --- Set default simulation parameters --- */
    cfg.carrier_hz = 24.0e9;          /* 24 GHz carrier (K-band satellite) */
    cfg.symbol_rate_hz = 10.0e6;      /* 10 MegaSymbols/sec */
    cfg.symbols = 2048;               /* 2048 symbols per run (denser constellation visualization) */
    cfg.rolloff = 0.2;               /* 20% Root-Raised-Cosine roll-off */
    cfg.input_snr_db = 20.0;         /* 20 dB input SNR based on assignment */
    cfg.antenna_temp_k = 150.0;      /* 150 K antenna temperature (typical satellite) */
    cfg.t0_k = 290.0;               /* 290 K reference temperature (room temp) */
    cfg.rf_sample_rate_hz = 96.0e9;  /* 96 GHz RF sampling → SPS ≈ 9600 */
    cfg.seed = (unsigned int)time(NULL);  /* Default seed: current time */
    topology_sim_id = 1;              /* Default output slot: topology_sim_1 */
    memset(&stage_cfg, 0, sizeof(stage_cfg));
    stage_err[0] = '\0';

    /* --- Parse command-line arguments --- */
    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            if (parse_u32(argv[++i], &cfg.seed) != 0) {
                fprintf(stderr, "Invalid --seed value\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--symbols") == 0 && i + 1 < argc) {
            if (parse_i32(argv[++i], &cfg.symbols) != 0 || cfg.symbols <= 0) {
                fprintf(stderr, "Invalid --symbols value\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--symbol-rate") == 0 && i + 1 < argc) {
            if (parse_double(argv[++i], &cfg.symbol_rate_hz) != 0 || cfg.symbol_rate_hz <= 0.0) {
                fprintf(stderr, "Invalid --symbol-rate value\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--rf-fs") == 0 && i + 1 < argc) {
            if (parse_double(argv[++i], &cfg.rf_sample_rate_hz) != 0 || cfg.rf_sample_rate_hz <= 0.0) {
                fprintf(stderr, "Invalid --rf-fs value\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--carrier") == 0 && i + 1 < argc) {
            if (parse_double(argv[++i], &cfg.carrier_hz) != 0 || cfg.carrier_hz <= 0.0) {
                fprintf(stderr, "Invalid --carrier value\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
            if (parse_double(argv[++i], &cfg.input_snr_db) != 0) {
                fprintf(stderr, "Invalid --snr value\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--stage-csv") == 0 && i + 1 < argc) {
            snprintf(stage_csv_path, sizeof(stage_csv_path), "%s", argv[++i]);
        } else if (strcmp(argv[i], "--topology-sim") == 0 && i + 1 < argc) {
            if (parse_i32(argv[++i], &topology_sim_id) != 0 || topology_sim_id < 1 || topology_sim_id > TOPOLOGY_SIM_COUNT) {
                fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n", TOPOLOGY_SIM_COUNT);
                return 2;
            }
        } else if (strcmp(argv[i], "--stage-sim") == 0 && i + 1 < argc) {
            if (parse_i32(argv[++i], &topology_sim_id) != 0 || topology_sim_id < 1 || topology_sim_id > TOPOLOGY_SIM_COUNT) {
                fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n", TOPOLOGY_SIM_COUNT);
                return 2;
            }
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    /* --- Initialize the PRNG --- */
    prng_seed(cfg.seed);

    /* --- Create output directories --- */
    if (ensure_output_dirs() != 0) {
        fprintf(stderr, "Failed to create output directories under ./out\n");
        return 8;
    }

    /* --- Resolve the stage-model CSV path --- */
    if (resolve_stage_csv_path(stage_csv_path, resolved_stage_csv_path, sizeof(resolved_stage_csv_path)) != 0) {
        fprintf(stderr, "Failed to resolve stage-model CSV path '%s'\n", stage_csv_path);
        return 5;
    }

    /* --- Build output directory paths --- */
    char baseband_dir[256];
    char rf_dir[256];
    if (snprintf(baseband_dir, sizeof(baseband_dir), OUTPUT_ROOT_DIR "/baseband") >= (int)sizeof(baseband_dir)
        || snprintf(rf_dir, sizeof(rf_dir), OUTPUT_ROOT_DIR "/rf") >= (int)sizeof(rf_dir)) {
        fprintf(stderr, "Failed to build output subdirectories\n");
        return 5;
    }

    /* Clear old output files from the selected slot */
    if (clear_directory_contents(baseband_dir) != 0 || clear_directory_contents(rf_dir) != 0) {
        fprintf(stderr, "Failed to clear output directories before writing new artifacts\n");
        return 5;
    }

    /* --- Build the 64-APSK constellation --- */
    if (build_64apsk_constellation_dvbs2(constellation) != 0) {
        fprintf(stderr, "Failed to build 64-APSK constellation\n");
        return 3;
    }

    /* --- Allocate and generate transmitted symbols --- */
    tx_symbols = (Complex*)calloc((size_t)cfg.symbols, sizeof(Complex));
    labels = (unsigned short*)calloc((size_t)cfg.symbols, sizeof(unsigned short));
    if (!tx_symbols || !labels) {
        fprintf(stderr, "Allocation failed for symbols\n");
        free(tx_symbols);
        free(labels);
        return 4;
    }

    generate_symbols(constellation, 64, tx_symbols, labels, (size_t)cfg.symbols);

    /* --- Calculate the link budget --- */
    /*
     * Noise bandwidth = symbol_rate × (1 + rolloff)
     * For 10 Msym/s with rolloff 0.2: BW = 12 MHz
     *
     * Thermal noise power: P = k_B × T × B
     * This is the fundamental noise floor set by physics.
     *
     * Convert to dBm: dBm = 10×log10(P_watts) + 30
     * (dBm = decibels relative to 1 milliwatt)
     *
     * Signal power: P_signal_dBm = P_noise_dBm + SNR_dB
     */
    noise_bw_hz = cfg.symbol_rate_hz * (1.0 + cfg.rolloff);
    noise_w = K_BOLTZMANN * cfg.antenna_temp_k * noise_bw_hz;
    noise_dbm = lin_to_db(noise_w) + 30.0;
    signal_dbm = noise_dbm + cfg.input_snr_db;

    /* Write input budget artifacts */
    /* Disabled per user request to only output the exact 4 requirements per stage */
    /* --- Load receiver stage configuration from CSV --- */
    if (stage_models_load_csv(resolved_stage_csv_path, &stage_cfg, stage_err, sizeof(stage_err)) != 0) {
        fprintf(stderr, "Failed to load stage-model CSV '%s': %s\n", resolved_stage_csv_path, stage_err);
        free(tx_symbols);
        free(labels);
        return 5;
    }

    /* --- Run Simulation 1: Complex Baseband --- */
    if (simulate_complex_baseband(
        &cfg, &stage_cfg, tx_symbols, constellation, 64u, (size_t)cfg.symbols,
        metrics_bb, &count_bb, &final_vpp_bb,
        baseband_dir, baseband_dir) != 0) {
        fprintf(stderr, "Complex-baseband simulation failed\n");
        stage_models_free(&stage_cfg);
        free(tx_symbols);
        free(labels);
        return 6;
    }

    /* --- Run Simulation 2: Brute-Force RF --- */
    if (simulate_bruteforce_rf(
        &cfg, &stage_cfg, tx_symbols, constellation, 64u, (size_t)cfg.symbols,
        metrics_rf, &count_rf, &final_vpp_rf,
        &rf_sps, &rf_fs_used,
        rf_dir, rf_dir) != 0) {
        fprintf(stderr, "Brute-force RF simulation failed\n");
        stage_models_free(&stage_cfg);
        free(tx_symbols);
        free(labels);
        return 7;
    }

    /* --- Print console summary --- */
    printf("seed=%u\n", cfg.seed);
    printf("Stage CSV: %s\n", resolved_stage_csv_path);
    printf("Input budget: Noise=%.4f dBm, Signal=%.4f dBm (SNR=%.2f dB)\n", noise_dbm, signal_dbm, cfg.input_snr_db);

    print_metrics("Baseband Stages Metrics (SNR/EVM after each stage)", metrics_bb, count_bb);
    print_metrics("RF Brute-Force Stages Metrics (SNR/EVM after each stage)", metrics_rf, count_rf);

    printf("\nOutputs written under ./out/\n");
    printf("Generated files strictly follow the assignment requirements (Trace, Constellation, SNR, EVM).\n");

    /* --- Cleanup --- */
    stage_models_free(&stage_cfg);
    free(tx_symbols);
    free(labels);
    return 0;   /* Success! */
}
