/*
 * ============================================================================
 * prng.c — Deterministic Pseudo-Random Number Generator (PRNG)
 * ============================================================================
 *
 * PURPOSE:
 *   This file provides a high-quality, deterministic random number generator
 *   that the simulator relies on for ALL randomness. That includes:
 *     - Picking which of the 64 constellation symbols to transmit
 *     - Generating Gaussian (bell-curve) noise added to signals
 *
 *   "Deterministic" means: if you give it the same starting seed number,
 *   it will always produce the EXACT SAME sequence of random numbers.
 *   This is critical for reproducibility — you can re-run a simulation
 *   with the same seed and get identical results, which is essential
 *   for debugging and scientific verification.
 *
 * CORE ALGORITHM — xoshiro256** (pronounced "zoh-SHEE-roh 256 star-star"):
 *   - This is one of the best-known non-cryptographic PRNGs in use today
 *   - It was designed by David Blackman and Sebastiano Vigna
 *   - It maintains 256 bits of internal state (four 64-bit numbers)
 *   - It has a period of 2^256 - 1, meaning it can produce that many numbers
 *     before repeating. This is an astronomically large number
 *   - It passes all major statistical randomness tests (BigCrush, PractRand)
 *   - It is very fast: roughly 10 CPU cycles per 64-bit random number
 *
 * SEEDING — SplitMix64:
 *   - The user provides a single 32-bit seed (like 42 or 12345)
 *   - But xoshiro256** needs 256 bits (four 64-bit words) of initial state
 *   - SplitMix64 is a simpler PRNG used ONLY during initialization to
 *     "expand" the 32-bit seed into four well-distributed 64-bit values
 *   - This ensures that even similar seeds (like 0 and 1) produce
 *     completely different random sequences
 *
 * RANDOM DISTRIBUTIONS PROVIDED:
 *   1. prng_uniform()  → Returns a random decimal number in [0.0, 1.0)
 *   2. prng_gauss()    → Returns a Gaussian (normal) random number with
 *                         mean=0 and standard deviation=1 (the "standard normal")
 *   3. prng_uint32()   → Returns a random 32-bit unsigned integer (0 to ~4 billion)
 *
 * THREAD SAFETY:
 *   This module uses file-scope static variables (global state).
 *   It is NOT thread-safe. If multiple threads call these functions
 *   simultaneously, the state will become corrupted. This is fine for
 *   this simulator, which runs single-threaded.
 *
 * ============================================================================
 */

#include "prng.h"   /* Public interface: prng_seed, prng_uniform, prng_gauss, prng_uint32 */

#include <math.h>   /* For sqrt(), log(), cos(), sin() — used in Box-Muller transform */

/*
 * M_PI might not be defined on all compilers/platforms (it's not part of the
 * C standard, only POSIX). So we define it ourselves if it's missing.
 * M_PI = π = 3.14159... — the ratio of a circle's circumference to its diameter.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * ============================================================================
 * INTERNAL STATE
 * ============================================================================
 *
 * s[4] — The four 64-bit words that make up the xoshiro256** state.
 *
 * These are initialized with arbitrary non-zero default values (shown below).
 * These defaults will be overwritten by prng_seed() before the simulation
 * actually starts. The defaults are only here to prevent an all-zero state
 * if someone accidentally calls prng_uniform() before prng_seed().
 *
 * An all-zero state is catastrophic for xoshiro256** because:
 *   XOR of zero with zero is always zero → the generator would be stuck
 *   producing 0 forever.
 */
static uint64_t s[4] = {
    0x180ec6d33cfd0abaULL,   /* Default state word 0 — arbitrary non-zero constant */
    0xd5a61266f0c9392cULL,   /* Default state word 1 — arbitrary non-zero constant */
    0xa9582618e03fc9aaULL,   /* Default state word 2 — arbitrary non-zero constant */
    0x39abdc4529b1661cULL    /* Default state word 3 — arbitrary non-zero constant */
};

/*
 * Box-Muller Gaussian cache variables:
 *
 * The Box-Muller transform (used in prng_gauss) converts two independent
 * uniform random numbers into two independent Gaussian random numbers.
 * Since we only need to return one at a time, we cache the second one
 * and return it on the next call. This effectively halves the cost.
 *
 * have_spare_gauss: 1 if spare_gauss contains a valid cached sample, 0 otherwise
 * spare_gauss:      the cached Gaussian sample from the previous Box-Muller call
 */
static int have_spare_gauss = 0;
static double spare_gauss = 0.0;


/* ============================================================================
 * INTERNAL HELPER FUNCTIONS
 * ============================================================================ */

/*
 * rotl — Bitwise Rotate Left
 *
 * What it does:
 *   Takes a 64-bit number and rotates its bits to the left by k positions.
 *   Bits that "fall off" the left end wrap around to the right end.
 *
 * Why it's needed:
 *   This is a fundamental building block of the xoshiro256** algorithm.
 *   Rotation mixes bits efficiently — it's faster than multiplication on
 *   many CPUs and creates complex bit-level dependencies that make the
 *   output appear random.
 *
 * Example:
 *   If x = 0b10110001 (8-bit for illustration) and k = 3:
 *   Result = 0b10001101 (the top 3 bits wrapped to the bottom)
 *
 * Parameters:
 *   x — the 64-bit value to rotate
 *   k — how many bit positions to rotate left (0 ≤ k < 64)
 *
 * Returns:
 *   The rotated 64-bit value
 *
 * Note: "static inline" means this function is private to this file
 * and the compiler should try to embed its code directly at each call site
 * (avoiding the overhead of a function call).
 */
static inline uint64_t rotl(const uint64_t x, int k) {
    /*
     * (x << k)          — shift left by k: bits move up, zeros fill from below
     * (x >> (64 - k))   — shift right by (64-k): captures the bits that
     *                      would have been lost by the left shift
     * OR them together  — combine to create the rotation effect
     */
    return (x << k) | (x >> (64 - k));
}


/*
 * next_u64 — Generate the next 64-bit pseudo-random number from xoshiro256**
 *
 * What it does:
 *   1. Computes the OUTPUT from the current state using a "scrambler":
 *      result = rotl(s[1] * 5, 7) * 9
 *      This specific formula (multiply by 5, rotate by 7, multiply by 9)
 *      is what makes this the "**" (starstar) variant of xoshiro256.
 *      It produces better statistical properties than the simpler variants.
 *
 *   2. Updates the 4-word state using XOR and shift operations:
 *      - These operations ensure that every state word influences every
 *        other state word over time, creating long-range correlations
 *        that make the output hard to predict.
 *      - The specific constants (shift of 17, rotation of 45) were chosen
 *        by the algorithm authors to maximize the period and quality.
 *
 * Why it's needed:
 *   This is the core of the PRNG. Every time we need a random number
 *   (for noise, for symbol selection, etc.), this function is called
 *   to advance the state and produce the next output.
 *
 * Returns:
 *   A 64-bit pseudo-random unsigned integer (all values 0..2^64-1 are possible)
 *
 * Period:
 *   2^256 - 1 (the generator cycles through this many states before repeating)
 */
static uint64_t next_u64(void) {
    /*
     * STEP 1: Compute the output value (scrambler).
     *
     * Take state word s[1], multiply by 5, rotate left by 7, then multiply by 9.
     * This scrambling operation maps the internal state to an output that has
     * better statistical properties than the raw state bits.
     *
     * Why s[1] and not s[0]?  The algorithm authors tested all combinations
     * and found s[1] gives the best results with these multiplier/rotation values.
     */
    const uint64_t result = rotl(s[1] * 5, 7) * 9;

    /*
     * STEP 2: Advance the state.
     *
     * t is a temporary value derived from s[1] shifted left by 17 bits.
     * This value will be XOR'd into s[2] near the end to mix state bits.
     */
    const uint64_t t = s[1] << 17;

    /*
     * STEP 3: Cross-mix all four state words via XOR operations.
     *
     * The order of these operations matters! Each line depends on the
     * previous state, and together they form a linear recurrence over GF(2)
     * (the field with two elements, 0 and 1 — basically binary arithmetic).
     *
     * s[2] ^= s[0] : Mix word 0 into word 2
     * s[3] ^= s[1] : Mix word 1 into word 3
     * s[1] ^= s[2] : Now word 2 has been updated, mix it back into word 1
     * s[0] ^= s[3] : Now word 3 has been updated, mix it back into word 0
     *
     * After these four XOR operations, every state word has been influenced
     * by at least one other word, creating the "avalanche effect" where
     * a single bit change propagates to affect many output bits.
     */
    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];

    /*
     * STEP 4: Apply the shift and rotation to complete the state update.
     *
     * s[2] ^= t      : XOR in the shifted value of the old s[1]
     * s[3] = rotl(45) : Rotate word 3 by 45 positions to break any
     *                    linear patterns that might form over many iterations
     */
    s[2] ^= t;
    s[3] = rotl(s[3], 45);

    return result;
}


/*
 * splitmix64 — Seeding helper that maps one 64-bit value to a well-distributed 64-bit hash
 *
 * What it does:
 *   Given a state variable (modified in-place), it:
 *   1. Adds the golden-ratio-derived constant 0x9e3779b97f4a7c15 to the state
 *      (this constant = floor(2^64 / φ) where φ = (1+√5)/2 is the golden ratio)
 *   2. Applies a series of XOR-shift and multiply operations that thoroughly
 *      "hash" the bits — even if the input changes by just 1 bit, the output
 *      will change dramatically (this is called the "avalanche" property)
 *
 * Why it's needed:
 *   When the user provides a small seed like 42, we need to convert it into
 *   four 64-bit state words that are wildly different from each other.
 *   SplitMix64 is called 4 times in sequence to produce s[0], s[1], s[2], s[3].
 *   Each call advances the internal counter and produces a new hash, so even
 *   though all 4 calls start from the same seed, they produce 4 different values.
 *
 * Parameters:
 *   x — pointer to the SplitMix64 state variable (read AND modified)
 *       On input: current counter value
 *       On output: counter advanced by one step
 *
 * Returns:
 *   A 64-bit pseudo-random value derived from the advanced counter
 *
 * Note: SplitMix64 was designed by Sebastiano Vigna. It's a simple but
 *       high-quality hash function specifically made for PRNG seeding.
 */
static uint64_t splitmix64(uint64_t* x) {
    /*
     * Step 1: Advance the counter by the golden-ratio constant.
     * The += modifies *x in place, and the new value is also stored in z.
     * This is written as a single expression: z = (*x += constant).
     */
    uint64_t z = (*x += 0x9e3779b97f4a7c15ULL);

    /*
     * Steps 2-4: Three rounds of XOR-shift-multiply hashing.
     * Each round:
     *   1. XOR z with itself shifted right (mixes high bits into low bits)
     *   2. Multiply by a large odd constant (creates complex bit dependencies)
     *
     * The specific shift amounts (30, 27, 31) and multiplier constants
     * were found by automated search to minimize statistical bias.
     */
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;   /* Round 1: shift 30, multiply */
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;   /* Round 2: shift 27, multiply */
    return z ^ (z >> 31);                            /* Round 3: final XOR-shift   */
}


/* ============================================================================
 * PUBLIC API FUNCTIONS
 * ============================================================================ */

/*
 * prng_seed — Initialize the PRNG with a user-provided 32-bit seed
 *
 * What it does:
 *   1. Takes the 32-bit seed and widens it to 64 bits (just zero-extends it)
 *   2. If the widened value is 0, changes it to 1 to prevent an all-zero state
 *      (an all-zero state would cause xoshiro256** to produce zeros forever)
 *   3. Calls splitmix64 four times to generate four well-distributed 64-bit
 *      values, which become the initial state of the xoshiro256** generator
 *   4. Resets the Gaussian cache (have_spare_gauss = 0) so that prng_gauss()
 *      doesn't return a stale cached value from a previous seed's sequence
 *
 * When to call:
 *   Call this ONCE at the start of the program, before any calls to
 *   prng_uniform(), prng_gauss(), or prng_uint32().
 *
 * Parameters:
 *   seed — any 32-bit value (0 through 4,294,967,295). The value 0 is
 *          automatically mapped to 1 internally, so all 32-bit values are valid.
 *
 * Determinism guarantee:
 *   Calling prng_seed(42) will ALWAYS set the same internal state,
 *   so subsequent calls to prng_uniform() etc. will produce the
 *   identical sequence of numbers every time.
 */
void prng_seed(uint32_t seed) {
    /* Widen the 32-bit seed to 64 bits. The upper 32 bits become zero. */
    uint64_t state = (uint64_t)seed;

    /* Protect against the all-zero state, which would make xoshiro stuck. */
    if (state == 0u) {
        state = 1u;
    }

    /*
     * Generate four 64-bit state words from the single seed.
     * Each call to splitmix64 advances `state` and returns a new hash.
     * After this, s[0..3] are initialized and ready for next_u64() calls.
     */
    s[0] = splitmix64(&state);
    s[1] = splitmix64(&state);
    s[2] = splitmix64(&state);
    s[3] = splitmix64(&state);

    /*
     * Reset the Gaussian cache. This is important because:
     * - If prng_gauss() was called under a previous seed, it may have
     *   cached a "spare" sample that belongs to the OLD sequence.
     * - After re-seeding, we must discard that stale sample.
     */
    have_spare_gauss = 0;
}


/*
 * prng_uniform — Generate a uniformly distributed random floating-point number in [0, 1)
 *
 * What it does:
 *   1. Calls next_u64() to get a fresh 64-bit random integer
 *   2. Right-shifts by 11 to keep only the upper 53 bits
 *   3. Multiplies by 2^(-53) to convert the 53-bit integer to a double
 *      in the range [0.0, 1.0)
 *
 * Why 53 bits?
 *   IEEE 754 double-precision floating-point numbers have a 52-bit mantissa
 *   (plus one implicit leading 1-bit), so 53 bits is the maximum precision.
 *   Using more bits would be wasted; using fewer would leave gaps in the
 *   distribution.
 *
 * What does [0, 1) mean?
 *   The result can be exactly 0.0, but it can NEVER be exactly 1.0.
 *   The largest possible value is 1.0 - 2^(-53) ≈ 0.999999999999999889.
 *   This is important for algorithms like Box-Muller that use log(u1) —
 *   log(0) is -infinity and log(1) is 0, and we want to avoid both extremes.
 *
 * Returns:
 *   A double-precision floating-point value uniformly distributed in [0.0, 1.0)
 *
 * Usage:
 *   double r = prng_uniform();  // r is a random number like 0.73821...
 */
double prng_uniform(void) {
    /*
     * (next_u64() >> 11)  — discard the lower 11 bits, keeping 53 random bits
     * 0x1.0p-53           — this is the C99 hexadecimal float literal for 2^(-53)
     *                        = 1.1102230246251565e-16
     * Multiplying a 53-bit integer by 2^(-53) gives a value in [0, 1)
     */
    return (next_u64() >> 11) * 0x1.0p-53;
}


/*
 * prng_gauss — Generate a standard normal (Gaussian) random variate N(0, 1)
 *
 * What it does:
 *   Uses the Box-Muller transform to convert two uniform random numbers
 *   into two independent standard normal random numbers.
 *
 * Background — What is a Gaussian/Normal distribution?
 *   A Gaussian distribution is the classic "bell curve". A standard normal
 *   distribution has:
 *     - Mean (center) = 0
 *     - Standard deviation (spread) = 1
 *   About 68% of values fall within ±1, 95% within ±2, 99.7% within ±3.
 *
 * The Box-Muller Algorithm:
 *   Given two independent uniform random numbers u1, u2 in (0, 1):
 *     magnitude = sqrt(-2 * ln(u1))         — "how far from the center"
 *     angle     = 2π * u2                    — "in which direction"
 *     sample1   = magnitude * cos(angle)     — first Gaussian sample
 *     sample2   = magnitude * sin(angle)     — second Gaussian sample
 *
 *   Both sample1 and sample2 are independent N(0,1) random variables.
 *   We return sample1 immediately and cache sample2 for the next call.
 *   This means on average we only need one uniform draw per Gaussian sample.
 *
 * Why we need Gaussian noise in this simulator:
 *   Real communication channels add thermal noise that follows a Gaussian
 *   distribution (by the Central Limit Theorem). AWGN (Additive White Gaussian
 *   Noise) is the standard model for this. Each noise sample is drawn from
 *   N(0, σ) where σ depends on the desired noise power.
 *
 * Edge case handling:
 *   - u1 = 0 would cause log(0) = -infinity, crashing the program.
 *     We clamp u1 to a minimum of 1e-15 to prevent this.
 *   - u1 very close to 0 would produce an extremely large Gaussian sample
 *     (like 8 or 9 standard deviations). This is fine — real Gaussians
 *     can produce extreme values too.
 *
 * Returns:
 *   A double sampled from the standard normal distribution N(0, 1).
 *   Typical values range from about -4 to +4, with most between -2 and +2.
 */
double prng_gauss(void) {
    double u1;     /* First uniform random sample — controls the magnitude */
    double u2;     /* Second uniform random sample — controls the angle */
    double mag;    /* Magnitude = sqrt(-2 * ln(u1)) */
    double phase;  /* Phase angle = 2π * u2 */

    /*
     * CACHING LOGIC:
     * Check if we have a cached ("spare") Gaussian sample from the previous call.
     * The Box-Muller transform produces TWO samples at once, but we can only
     * return one. So on even-numbered calls, we compute both and cache one.
     * On odd-numbered calls, we just return the cached one — no computation needed!
     */
    if (have_spare_gauss) {
        have_spare_gauss = 0;   /* Mark the cache as empty */
        return spare_gauss;     /* Return the previously cached sample */
    }

    /* --- No cached sample available; compute a fresh pair --- */

    /* Draw two independent uniform random samples from [0, 1) */
    u1 = prng_uniform();
    if (u1 < 1e-15) {
        u1 = 1e-15; /* Clamp to prevent log(0) = -infinity */
    }
    u2 = prng_uniform();

    /*
     * Box-Muller transform:
     *
     *   mag   = sqrt(-2 * ln(u1))
     *         The logarithm creates an exponential distribution, and the
     *         square root converts it to a Rayleigh distribution — which is
     *         the distribution of the distance from the origin in 2D Gaussian noise.
     *
     *   phase = 2π * u2
     *         A uniform angle in [0, 2π) — this picks a random direction
     *         on the unit circle.
     *
     * Together, (mag, phase) define a point in polar coordinates whose
     * Cartesian components (mag*cos, mag*sin) are two independent N(0,1) samples.
     */
    mag   = sqrt(-2.0 * log(u1));
    phase = 2.0 * M_PI * u2;

    /*
     * Produce TWO independent Gaussian samples:
     *   spare_gauss = mag * sin(phase)  — cache this for the next call
     *   return        mag * cos(phase)  — return this one now
     */
    spare_gauss      = mag * sin(phase);
    have_spare_gauss = 1;          /* Flag that the cache is valid */
    return mag * cos(phase);       /* Return the first Gaussian sample */
}


/*
 * prng_uint32 — Generate a uniformly distributed random 32-bit unsigned integer
 *
 * What it does:
 *   Calls next_u64() to get a 64-bit random number, then extracts the upper
 *   32 bits by right-shifting 32 positions. The upper bits of xoshiro256**
 *   have slightly better statistical properties than the lower bits.
 *
 * Usage in this simulator:
 *   Used for selecting random constellation symbol indices:
 *     idx = prng_uint32() % 64;   // pick one of 64 APSK symbols
 *   The modulo operation maps the random 32-bit value to the range [0, 63].
 *
 * Bias note:
 *   Modulo by 64 (a power of 2) introduces ZERO bias because 2^32 is
 *   exactly divisible by 64. For non-power-of-2 moduli, there would be
 *   a tiny bias, but it's negligible for simulation purposes.
 *
 * Returns:
 *   A 32-bit unsigned integer uniformly distributed over [0, 2^32 - 1]
 *   (i.e., from 0 to 4,294,967,295 inclusive).
 */
uint32_t prng_uint32(void) {
    /*
     * (next_u64() >> 32) — take the upper 32 bits of the 64-bit output.
     * Cast to uint32_t — narrow from 64 bits to 32 bits (discards upper zeros).
     */
    return (uint32_t)(next_u64() >> 32);
}
