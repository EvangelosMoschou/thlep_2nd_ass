#ifndef PRNG_H
#define PRNG_H

#include <stdint.h>

typedef struct PrngState {
    uint64_t s[4];
    int have_spare_gauss;
    double spare_gauss;
} PrngState;

#define PRNG_MAX_OMP_THREADS 64

void     prng_init(PrngState *state, uint32_t seed);
double   prng_uniform(PrngState *state);
double   prng_gauss(PrngState *state);
uint32_t prng_uint32(PrngState *state);

void   prng_init_parallel(PrngState *thread_states, uint32_t seed);
double prng_gauss_parallel(PrngState *thread_states);

#endif
