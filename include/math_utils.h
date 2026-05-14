#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>

/**
 * @brief Linear power to dB.
 */
static inline double lin_to_db(double lin) {
    if (lin <= 0.0) return -1000.0;
    return 10.0 * log10(lin);
}

/**
 * @brief dB to linear power.
 */
static inline double db_to_lin(double db) {
    return pow(10.0, db / 10.0);
}

#endif
