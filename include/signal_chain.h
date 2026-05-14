#ifndef SIGNAL_CHAIN_H
#define SIGNAL_CHAIN_H

#include <stddef.h>
#include "sim_types.h"
#include "stage_models.h"
#include "metrics.h"

/**
 * generate_symbols — Generate random 64-APSK symbols
 */
void generate_symbols(const Complex *constellation, size_t constellation_size,
                      Complex *out_symbols, unsigned short *out_labels,
                      size_t nsym);

/**
 * calculate_evm — Compute EVM between reference and impaired symbols
 */
double calculate_evm(const Complex *ref, const Complex *sig, size_t nsym);

#endif
