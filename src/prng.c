#include "prng.h"
#include <math.h>

void prng_init(PrngState *state, uint32_t seed) {
    state->s[0] = seed;
    state->s[1] = 0x9e3779b97f4a7c15;
    state->s[2] = 0x3141592653589793;
    state->s[3] = 0xf3354a7e;
    state->have_spare_gauss = 0;
}

static inline uint64_t rotl(const uint64_t x, int k) {
    return (x << k) | (x >> (64 - k));
}

uint32_t prng_uint32(PrngState *state) {
    const uint64_t result = rotl(state->s[1] * 5, 7) * 9;
    const uint64_t t = state->s[1] << 17;
    state->s[2] ^= state->s[0];
    state->s[3] ^= state->s[1];
    state->s[1] ^= state->s[2];
    state->s[0] ^= state->s[3];
    state->s[2] ^= t;
    state->s[3] = rotl(state->s[3], 45);
    return (uint32_t)result;
}

double prng_uniform(PrngState *state) {
    return (double)prng_uint32(state) / 4294967296.0;
}

double prng_gauss(PrngState *state) {
    if (state->have_spare_gauss) {
        state->have_spare_gauss = 0;
        return state->spare_gauss;
    }
    double u1 = prng_uniform(state);
    double u2 = prng_uniform(state);
    double r = sqrt(-2.0 * log(u1 > 0 ? u1 : 1e-12));
    double theta = 2.0 * M_PI * u2;
    state->spare_gauss = r * sin(theta);
    state->have_spare_gauss = 1;
    return r * cos(theta);
}
