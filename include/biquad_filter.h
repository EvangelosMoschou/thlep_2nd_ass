#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

#include <stddef.h>

typedef struct {
    double b[3];
    double a[3];
    double w[2]; /* Direct Form II delay elements */
} BiquadSection;

typedef struct {
    BiquadSection sections[4];
    size_t section_count;
} BiquadState;

typedef struct {
    double cutoff_hz;
    double fs_hz;
    int order;
} BiquadConfig;

int biquad_init(BiquadState* state, const BiquadConfig* cfg);
void biquad_process_soa(BiquadState* state, const double* in_re, const double* in_im, double* out_re, double* out_im, size_t n);

#endif
