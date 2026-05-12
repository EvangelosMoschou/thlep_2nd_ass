#ifndef PHYSICS_H
#define PHYSICS_H

/*
 * K_BOLTZMANN — Boltzmann constant [J/K]
 *
 * The fundamental physical constant k_B = 1.380649e-23 J/K relating
 * temperature to thermal energy. Used in noise power calculations:
 *   N = k_B × T × B
 * where T is system temperature in Kelvin and B is noise bandwidth in Hz.
 * This gives the minimum possible noise power — the fundamental limit
 * set by thermodynamics (Johnson–Nyquist noise).
 */
#define K_BOLTZMANN 1.380649e-23

/*
 * B_NOISE_HZ — Equivalent receiver noise bandwidth [Hz]
 *
 * The effective bandwidth used for thermal noise power integration
 * throughout the simulation. Chosen to match the MATLAB reference
 * (sim_receiver_matlab.m). Combined with K_BOLTZMANN and system
 * temperature to compute the noise floor.
 */
#define B_NOISE_HZ 200.0e6

/*
 * R_LOAD_OHM — Reference system impedance [Ω]
 *
 * Standard 50 Ω impedance for RF power ↔ voltage conversion.
 * Used to transform power quantities (Watts) into voltage-squared
 * domain (V²) for signal chain calculations:
 *   V² = P × R
 */
#define R_LOAD_OHM 50.0

#endif /* PHYSICS_H */
