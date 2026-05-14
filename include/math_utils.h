#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>
#include "sim_types.h"

static inline double db_to_lin_power(double db) {
    return pow(10.0, db / 10.0);
}

static inline double lin_to_db(double lin) {
    return 10.0 * log10(lin + 1e-25);
}

static inline double complex_abs_sq(Complex c) {
    return c.re * c.re + c.im * c.im;
}

#endif
