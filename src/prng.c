#include "prng.h"
#include "math_utils.h"

#include <omp.h>
#include <math.h>

static inline uint64_t rotl(const uint64_t x, int k) {
    return (x << k) | (x >> (64 - k));
}

static uint64_t next_u64(PrngState *st) {
    const uint64_t result = rotl(st->s[1] * 5, 7) * 9;
    const uint64_t t = st->s[1] << 17;
    st->s[2] ^= st->s[0];
    st->s[3] ^= st->s[1];
    st->s[1] ^= st->s[2];
    st->s[0] ^= st->s[3];
    st->s[2] ^= t;
    st->s[3] = rotl(st->s[3], 45);
    return result;
}

static uint64_t splitmix64(uint64_t* x) {
    uint64_t z = (*x += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

void prng_init(PrngState *state, uint32_t seed) {
    uint64_t mix = (uint64_t)seed;
    if (mix == 0u) mix = 1u;
    state->s[0] = splitmix64(&mix);
    state->s[1] = splitmix64(&mix);
    state->s[2] = splitmix64(&mix);
    state->s[3] = splitmix64(&mix);
    state->have_spare_gauss = 0;
    state->spare_gauss = 0.0;
}

double prng_uniform(PrngState *state) {
    return (next_u64(state) >> 11) * 0x1.0p-53;
}

uint32_t prng_uint32(PrngState *state) {
    return (uint32_t)(next_u64(state) >> 32);
}

/* Gaussian sampling — exact Box-Muller transform.
 *
 * HISTORY / WHY NOT A ZIGGURAT ANYMORE: this module previously claimed to
 * implement a ziggurat sampler, but the hard-coded tables were not a valid
 * ziggurat construction (the f[] column matched the normal density at no
 * ladder point, most rows were zero padding, and box heights could never
 * cover the density peak), and the sampler additionally cached a
 * same-magnitude "+/-x" spare for every other call. Measured over millions
 * of draws the old generator produced mean +0.27, variance ~0.63, 75 %
 * positive samples and magnitude-correlated consecutive pairs — silently
 * skewing every noise-driven result in the simulator (AWGN, phase noise,
 * flicker noise, ADC jitter).
 *
 * The invalid tables were removed in favour of Box-Muller, which is exact
 * for any uniform source. Seeding behaviour and reproducibility per seed
 * are unchanged; only the produced stream values differ from the broken
 * predecessor.
 */
#define PRNG_TWO_PI 6.283185307179586476925286766559

static double gauss_from(PrngState *st) {
    double u1 = prng_uniform(st);
    /* prng_uniform may legally return exactly 0; guard log(0) = -inf. */
    while (u1 <= 0.0)
        u1 = prng_uniform(st);
    const double u2 = prng_uniform(st);
    return sqrt(-2.0 * log(u1)) * cos(PRNG_TWO_PI * u2);
}

double prng_gauss(PrngState *state) {
    return gauss_from(state);
}

void prng_init_parallel(PrngState *thread_states, uint32_t seed) {
    uint64_t mix = (uint64_t)seed;
    if (mix == 0u) mix = 1u;

    int nthreads = omp_get_max_threads();
    if (nthreads > PRNG_MAX_OMP_THREADS) nthreads = PRNG_MAX_OMP_THREADS;

    for (int t = 0; t < nthreads; t++) {
        thread_states[t].s[0] = splitmix64(&mix);
        thread_states[t].s[1] = splitmix64(&mix);
        thread_states[t].s[2] = splitmix64(&mix);
        thread_states[t].s[3] = splitmix64(&mix);
        thread_states[t].have_spare_gauss = 0;
        thread_states[t].spare_gauss = 0.0;
    }
}

double prng_gauss_parallel(PrngState *thread_states) {
    int tid = omp_get_thread_num();
    if (tid < 0 || tid >= PRNG_MAX_OMP_THREADS)
        tid = 0; /* defensive clamp: keeps indexing in bounds if the OpenMP
                    runtime ever reports more threads than initialized */
    return gauss_from(&thread_states[tid]);
}
