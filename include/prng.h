#ifndef PRNG_H
#define PRNG_H

#include <stdint.h>

/**
 * @brief Opaque PRNG state for xoshiro256** with ziggurat Gaussian sampling.
 *
 * Contains the 256-bit xoshiro256** state (four 64-bit words) plus
 * ziggurat spare-sample caching for Box-Muller compatibility.
 *
 * Create one per independent PRNG stream (e.g. one for serial,
 * one per OpenMP thread for parallel).
 */
typedef struct PrngState {
    uint64_t s[4];          /**< xoshiro256** internal state */
    int have_spare_gauss;   /**< 1 if spare_gauss holds a cached sample */
    double spare_gauss;     /**< cached spare Gaussian sample */
} PrngState;

/**
 * @brief Maximum number of OpenMP threads supported by the parallel PRNG.
 *
 * The caller must allocate at least this many PrngState entries when
 * using prng_init_parallel / prng_gauss_parallel.
 */
#define PRNG_MAX_OMP_THREADS 64

/* --- Serial (single-stream) interface --- */

void     prng_init(PrngState *state, uint32_t seed);
double   prng_uniform(PrngState *state);
double   prng_gauss(PrngState *state);
uint32_t prng_uint32(PrngState *state);

/* --- Parallel (multi-thread) interface --- */

void   prng_init_parallel(PrngState *thread_states, uint32_t seed);
double prng_gauss_parallel(PrngState *thread_states);

#endif
