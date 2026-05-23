#include "soa_utils.h"
#include <string.h>

void pack_complex(const double *restrict re, const double *restrict im,
                  size_t n, Complex *restrict dst) {
    if (!re || !im || !dst || n == 0) return;
    for (size_t i = 0; i < n; ++i) {
        dst[i].re = re[i];
        dst[i].im = im[i];
    }
}

void unpack_complex(const Complex *restrict src, size_t n,
                    double *restrict re, double *restrict im) {
    if (!src || !re || !im || n == 0) return;
    for (size_t i = 0; i < n; ++i) {
        re[i] = src[i].re;
        im[i] = src[i].im;
    }
}

void soa_copy(const double *restrict src_re, const double *restrict src_im,
              size_t n, double *restrict dst_re, double *restrict dst_im) {
    if (n == 0) return;
    if (src_re && dst_re) {
        memcpy(dst_re, src_re, n * sizeof(double));
    }
    if (src_im && dst_im) {
        memcpy(dst_im, src_im, n * sizeof(double));
    }
}

void soa_scale(const double *restrict src_re, const double *restrict src_im,
               double factor, size_t n,
               double *restrict dst_re, double *restrict dst_im) {
    if (n == 0) return;
    for (size_t i = 0; i < n; ++i) {
        if (src_re && dst_re) dst_re[i] = src_re[i] * factor;
        if (src_im && dst_im) dst_im[i] = src_im[i] * factor;
    }
}

void soa_zero(double *restrict re, double *restrict im, size_t n) {
    if (n == 0) return;
    if (re) memset(re, 0, n * sizeof(double));
    if (im) memset(im, 0, n * sizeof(double));
}
