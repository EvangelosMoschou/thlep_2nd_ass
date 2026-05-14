#ifndef CONSTELLATION_H
#define CONSTELLATION_H

#include <stddef.h>   /* size_t */
#include "sim_types.h" /* Complex */

/*
 * build_dvbs2_64apsk_constellation — Generate the DVB-S2X 64-APSK constellation
 *
 * Constructs all 64 complex constellation points according to the DVB-S2X
 * standard specification. The constellation has 4 concentric rings with
 * non-uniform radii and non-uniform numbers of points:
 *
 *   Ring 0: radius 1.00 — innermost ring ( 4 points)
 *   Ring 1: radius 2.73 —              (12 points)
 *   Ring 2: radius 4.52 —              (20 points)
 *   Ring 3: radius 6.15 — outermost    (28 points)
 *
 * After constructing all 64 points the constellation is normalized so that
 * the average symbol energy is 1.0 (Es = 1).
 *
 * Parameters:
 *   table — Pre-allocated array of count Complex elements
 *   count — Number of elements available (must be >= 64)
 *
 * Returns:
 *   0  on success
 *  -1  if table is NULL
 *  -2  if count < 64
 *  -3  if the constellation is degenerate (all-zero after construction)
 */
int build_dvbs2_64apsk_constellation(Complex *table, size_t count);

#endif /* CONSTELLATION_H */
