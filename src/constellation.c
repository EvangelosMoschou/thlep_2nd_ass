/*
 * constellation.c — DVB-S2X 64-APSK constellation generation
 *
 * Implements the DVB-S2X 64-APSK constellation with 4 concentric rings:
 *    Ring 0:  4 points at radius  1.00
 *    Ring 1: 12 points at radius  2.73
 *    Ring 2: 20 points at radius  4.52
 *    Ring 3: 28 points at radius  6.15
 *
 * Points within each ring are evenly distributed. The whole set is then
 * normalised to unit average symbol energy (Es = 1).
 */

#include <stddef.h>

#include "math_utils.h"
#include "sim_types.h"
#include "constellation.h"

/* ------------------------------------------------------------------ */
/*  Normalise constellation to unit average power                     */
/* ------------------------------------------------------------------ */

/*
 * Normalise all constellation points so their mean squared magnitude
 * (average symbol energy) equals exactly 1.0.
 *
 *   c — Array of m complex points (modified in-place)
 *   m — Number of points
 *
 * Returns 0 on success, -1 if c is NULL or m <= 0, -2 if all points are zero.
 */
static int normalize_constellation(Complex *c, int m) {
  double p = 0.0;
  int i;

  if (!c || m <= 0) {
    return -1;
  }

  /* Sum of squared magnitudes */
  for (i = 0; i < m; ++i) {
    p += c[i].re * c[i].re + c[i].im * c[i].im;
  }
  p /= (double)m;

  if (p <= 0.0) {
    return -2;       /* degenerate — physically meaningless */
  }

  { /* Scale each point by 1/sqrt(mean power) */
    const double s = 1.0 / sqrt(p);
    for (i = 0; i < m; ++i) {
      c[i].re *= s;
      c[i].im *= s;
    }
  }

  return 0;
}

/* ------------------------------------------------------------------ */
/*  Build the DVB-S2X 64-APSK constellation                           */
/* ------------------------------------------------------------------ */

int build_dvbs2_64apsk_constellation(Complex *table, size_t count) {
  if (!table)
    return -1;
  if (count < 64)
    return -2;

  /*
   * DVB-S2 64-APSK: 4 rings of 4, 12, 20, 28 points.
   * Ring radii from the DVB-S2X standard (pre-normalisation).
   */
  const int    n_points[4]    = { 4, 12, 20, 28 };
  const double ring_radii[4]  = { 1.0, 2.73, 4.52, 6.15 };

  int idx = 0;
  for (int i = 0; i < 4; ++i) {
    const int    N      = n_points[i];
    const double radius = ring_radii[i];
    for (int k = 0; k < N; ++k) {
      const double phase = (double)k * (2.0 * M_PI / N) + (M_PI / N);
      table[idx].re = radius * cos(phase);
      table[idx].im = radius * sin(phase);
      ++idx;
    }
  }

  const int ret = normalize_constellation(table, 64);
  if (ret != 0)
    return -3;       /* normalisation failed */

  return 0;
}
