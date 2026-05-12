#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#define _USE_MATH_DEFINES
#include <math.h>

/*
 * M_PI and M_LN2 — Portable fallback definitions
 *
 * M_PI and M_LN2 are POSIX extensions not guaranteed by the C standard.
 * On MSVC, _USE_MATH_DEFINES (above) enables them; on POSIX systems they
 * are available by default.  These guards ensure availability everywhere.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_LN2
#define M_LN2 0.69314718055994530942
#endif

/*
 * db_to_lin_power — Convert dB to linear power ratio
 *
 * Formula:  linear = 10^(dB / 10)
 *
 * Examples:
 *   db_to_lin_power(0)   → 1.0        (0 dB = no change)
 *   db_to_lin_power(10)  → 10.0       (10 dB = 10× power amplification)
 *   db_to_lin_power(20)  → 100.0      (20 dB = 100× power amplification)
 *   db_to_lin_power(-3)  → ~0.5       (-3 dB ≈ half power)
 *   db_to_lin_power(-10) → 0.1        (-10 dB = 10× attenuation)
 *
 * Use this for power ratios (e.g., SNR, noise figure, gain in power).
 */
static inline double db_to_lin_power(double x_db) {
    return pow(10.0, x_db / 10.0);
}

/*
 * db_to_lin_amplitude — Convert dB to linear amplitude/voltage ratio
 *
 * Formula:  linear = 10^(dB / 20)
 *
 * Examples:
 *   db_to_lin_amplitude(0)  → 1.0       (0 dB = no change)
 *   db_to_lin_amplitude(6)  → ~2.0      (6 dB ≈ 2× voltage amplification)
 *   db_to_lin_amplitude(20) → 10.0      (20 dB = 10× voltage amplification)
 *
 * Use this for amplitude/voltage ratios (e.g., I/Q gain mismatch).
 */
static inline double db_to_lin_amplitude(double x_db) {
    return pow(10.0, x_db / 20.0);
}

/*
 * lin_to_db — Convert a linear value to decibels (power ratio)
 *
 * Formula:  dB = 10 × log10(linear)
 *
 * Special case: if the linear value is ≤ 0, returns -infinity
 * (you can't take the log of zero or negative numbers).
 */
static inline double lin_to_db(double x_lin) {
    if (x_lin <= 0.0) {
        return -INFINITY;
    }
    return 10.0 * log10(x_lin);
}

#endif /* MATH_UTILS_H */
