#ifndef PRNG_H
#define PRNG_H

#include <stdint.h>

void prng_seed(uint32_t seed);
double prng_uniform(void);
double prng_gauss(void);
uint32_t prng_uint32(void);

void prng_init_parallel(uint32_t seed);
double prng_gauss_parallel(void);

#endif
