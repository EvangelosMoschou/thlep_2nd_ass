#ifndef SOA_UTILS_H
#define SOA_UTILS_H

#include <stddef.h>
#include "sim_types.h"

/**
 * @file soa_utils.h
 * @brief Utilities for converting and managing Structure-of-Arrays (SoA) layout.
 *
 * SoA layout stores real (re) and imaginary (im) parts of complex signals in separate,
 * contiguous double arrays. This layout is preferred for high-performance DSP loops:
 * 1. CPU prefetcher predicts sequential reads/writes perfectly.
 * 2. Unlocks SIMD auto-vectorization (AVX2/AVX-512) by removing interleaved stride.
 */

/*
 * pack_complex - Pack separate real and imaginary arrays (SoA) into interleaved Complex array (AoS).
 */
void pack_complex(const double *restrict re, const double *restrict im,
                  size_t n, Complex *restrict dst);

/*
 * unpack_complex - Unpack interleaved Complex array (AoS) into separate real and imaginary arrays (SoA).
 */
void unpack_complex(const Complex *restrict src, size_t n,
                    double *restrict re, double *restrict im);

/*
 * soa_copy - Copy a complex signal in SoA layout from src to dst.
 */
void soa_copy(const double *restrict src_re, const double *restrict src_im,
              size_t n, double *restrict dst_re, double *restrict dst_im);

/*
 * soa_scale - Scale SoA components by a scalar factor.
 */
void soa_scale(const double *restrict src_re, const double *restrict src_im,
               double factor, size_t n,
               double *restrict dst_re, double *restrict dst_im);

/*
 * soa_zero - Zero out real and imaginary arrays of size n.
 */
void soa_zero(double *restrict re, double *restrict im, size_t n);

#endif /* SOA_UTILS_H */
