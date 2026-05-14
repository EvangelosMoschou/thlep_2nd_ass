#ifndef CONSTELLATION_H
#define CONSTELLATION_H

#include "sim_types.h"

int build_dvbs2_64apsk_constellation(Complex* out, size_t n);
void generate_symbols(const Complex* constellation, size_t n_const, Complex* out_syms, unsigned short* out_labels, size_t n_syms);

#endif
