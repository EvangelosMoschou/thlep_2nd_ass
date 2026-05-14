#include "constellation.h"
#include <math.h>

int build_dvbs2_64apsk_constellation(Complex *table, size_t count) {
    if (!table) return -1;
    if (count < 64) return -2;

    const double R1 = 1.0;
    const double R2 = 2.73;
    const double R3 = 4.52;
    const double R4 = 6.15;

    int idx = 0;
    /* Ring 1: 4 points */
    for (int i = 0; i < 4; i++) {
        double phi = M_PI/4.0 + i*M_PI/2.0;
        table[idx].re = R1 * cos(phi);
        table[idx].im = R1 * sin(phi);
        idx++;
    }
    /* Ring 2: 12 points */
    for (int i = 0; i < 12; i++) {
        double phi = M_PI/12.0 + i*M_PI/6.0;
        table[idx].re = R2 * cos(phi);
        table[idx].im = R2 * sin(phi);
        idx++;
    }
    /* Ring 3: 20 points */
    for (int i = 0; i < 20; i++) {
        double phi = M_PI/20.0 + i*M_PI/10.0;
        table[idx].re = R3 * cos(phi);
        table[idx].im = R3 * sin(phi);
        idx++;
    }
    /* Ring 4: 28 points */
    for (int i = 0; i < 28; i++) {
        double phi = M_PI/28.0 + i*M_PI/14.0;
        table[idx].re = R4 * cos(phi);
        table[idx].im = R4 * sin(phi);
        idx++;
    }

    /* Normalize Es = 1 */
    double total_energy = 0;
    for (int i = 0; i < 64; i++) {
        total_energy += table[i].re*table[i].re + table[i].im*table[i].im;
    }
    double scale = sqrt(64.0 / total_energy);
    for (int i = 0; i < 64; i++) {
        table[i].re *= scale;
        table[i].im *= scale;
    }

    return 0;
}
