#include "signal_chain.h"
#include "prng.h"
#include <math.h>

void generate_symbols(const Complex *constellation, size_t constellation_size,
                      Complex *out_symbols, unsigned short *out_labels,
                      size_t nsym) {
    PrngState rng;
    prng_init(&rng, 1234);
    for (size_t i = 0; i < nsym; i++) {
        int idx = prng_uint32(&rng) % constellation_size;
        out_symbols[i] = constellation[idx];
        out_labels[i] = (unsigned short)idx;
    }
}

double calculate_evm(const Complex *ref, const Complex *sig, size_t nsym) {
    double sum_sq_err = 0;
    double sum_sq_ref = 0;
    for (size_t i = 0; i < nsym; i++) {
        double dr = sig[i].re - ref[i].re;
        double di = sig[i].im - ref[i].im;
        sum_sq_err += dr*dr + di*di;
        sum_sq_ref += ref[i].re*ref[i].re + ref[i].im*ref[i].im;
    }
    return sqrt(sum_sq_err / sum_sq_ref) * 100.0;
}
