/**
 * @file iq_imbalance.h
 * @brief I/Q imbalance model for gain and phase errors in quadrature receivers.
 *
 * Models static (frequency-independent) I/Q imbalance caused by:
 *   - Gain mismatch between I and Q branches (typical: 0.3 dB)
 *   - Phase deviation from ideal 90-degree quadrature (typical: 1.5 degrees)
 *
 * Signal model for complex baseband:
 *   Q_out = Q_in * 10^(-gain_error_db/20) * cos(phase_error_deg)
 *           - I_in * sin(phase_error_deg)
 *   I_out = I_in  (unchanged; imbalance is applied to Q branch)
 *
 * A real-signal variant is also provided for RF-path modeling where
 * I and Q are carried on separate real-valued waveforms.
 *
 * Typical usage:
 * @code
 *   IQImbalanceConfig cfg = {0};
 *   cfg.gain_error_db = 0.3;
 *   cfg.phase_error_deg = 1.5;
 *
 *   Complex sample = {1.0, 0.5};
 *   iq_imbalance_apply(&sample, &cfg);
 *   // sample now contains the imbalanced I/Q values
 * @endcode
 */

#ifndef IQ_IMBALANCE_H
#define IQ_IMBALANCE_H

#include "sim_types.h"

/**
 * @brief Configuration for I/Q imbalance.
 *
 * All imbalance values are specified in engineering units (dB, degrees).
 * Typical values for a moderate-quality receiver:
 *   gain_error_db   = 0.3 dB
 *   phase_error_deg = 1.5 degrees
 */
typedef struct IQImbalanceConfig {
    /** Gain mismatch between I and Q branches in dB.
     *  Positive means Q branch is attenuated relative to I.
     *  Typical: 0.3 dB.  Zero means no gain error. */
    double gain_error_db;

    /** Phase deviation from ideal 90-degree quadrature in degrees.
     *  Typical: 1.5 degrees.  Zero means perfect quadrature. */
    double phase_error_deg;
} IQImbalanceConfig;

/**
 * @brief Apply I/Q imbalance to a complex baseband sample in-place.
 *
 * The imbalance model modifies the Q component as:
 *   Q_out = Q_in * 10^(-gain_error_db/20) * cos(phase_error_deg)
 *           - I_in * sin(phase_error_deg)
 * The I component passes through unchanged.
 *
 * @param s     Pointer to the complex sample (modified in-place).
 * @param cfg   Pointer to the imbalance configuration.
 */
void iq_imbalance_apply(Complex *s, const IQImbalanceConfig *cfg);

/**
 * @brief Apply I/Q imbalance to separate real I and Q signals in-place.
 *
 * This is the real-signal variant for RF-path modeling where I and Q
 * are carried on separate real-valued buffers (e.g., after downconversion
 * but before recombination into Complex form).
 *
 * The same model as iq_imbalance_apply() is used:
 *   q_out = q_in * 10^(-gain_error_db/20) * cos(phase_error_deg)
 *           - i_in * sin(phase_error_deg)
 *
 * @param i     Pointer to the I (real) sample (read-only for computation,
 *              not modified by this function).
 * @param q     Pointer to the Q (real) sample (modified in-place).
 * @param cfg   Pointer to the imbalance configuration.
 */
void iq_imbalance_apply_real(double *i, double *q, const IQImbalanceConfig *cfg);

#endif /* IQ_IMBALANCE_H */
