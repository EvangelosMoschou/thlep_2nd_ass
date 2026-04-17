/**
 * @file prng.h
 * @brief Pseudo-random number generator public interface.
 *
 * Provides a seeded, high-quality PRNG based on the xoshiro256** algorithm,
 * with Gaussian (Box–Muller) and uniform variate generation on top of it.
 *
 * All state is module-global and NOT thread-safe.  Seed once at program start
 * via prng_seed() and then call the generator functions freely.
 */

#ifndef PRNG_H
#define PRNG_H

#include <stdint.h>

/**
 * @brief Seed the PRNG with a 32-bit value.
 *
 * Internally expands the seed through SplitMix64 to initialise the four
 * 64-bit words of the xoshiro256** state, guaranteeing a well-distributed
 * starting state even for small seed values such as 0 or 1.
 *
 * @param seed  32-bit seed value (any value is valid).
 */
void prng_seed(uint32_t seed);

/**
 * @brief Draw a uniform random double in [0, 1).
 *
 * Uses the upper 53 bits of the next xoshiro256** output so that the result
 * maps exactly onto IEEE-754 double precision without bias.
 *
 * @return Double in [0.0, 1.0).
 */
double prng_uniform(void);

/**
 * @brief Draw a standard normal (Gaussian) variate N(0, 1).
 *
 * Implements the Box–Muller transform.  Two uniform variates produce two
 * independent Gaussian samples; the second sample is cached and returned on
 * the next call, so the cost is effectively one uniform draw per call on
 * average.
 *
 * @return A sample from N(0, 1).
 */
double prng_gauss(void);

/**
 * @brief Draw a uniform random 32-bit unsigned integer.
 *
 * Useful for integer-index sampling (e.g., selecting constellation points).
 * Takes the upper 32 bits of the next xoshiro256** output.
 *
 * @return Uniform random uint32_t.
 */
uint32_t prng_uint32(void);

#endif /* PRNG_H */
