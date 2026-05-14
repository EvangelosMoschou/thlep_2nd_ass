#ifndef PRNG_H
#define PRNG_H

#include <stdint.h>
#include "sim_types.h"

void prng_init(PrngState* state, uint32_t seed);
void prng_init_parallel(PrngState* states, uint32_t seed);
double prng_next_norm(PrngState* state);

#endif
