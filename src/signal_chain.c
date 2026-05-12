#include "signal_chain.h"

#include <string.h>

#include "math_utils.h"
#include "metrics.h"
#include "physics.h"

/* --- Noise injection (reentrant) --- */

static void add_awgn_complex_re(Complex *x, size_t n, double noise_power,
                                PrngState *rng) {
  size_t i;
  if (!x || n == 0u || noise_power <= 0.0) return;
  double sigma = sqrt(noise_power / 2.0);
  for (i = 0; i < n; ++i) {
    x[i].re += sigma * prng_gauss(rng);
    x[i].im += sigma * prng_gauss(rng);
  }
}

static void add_awgn_real_re(double *x, size_t n, double noise_power,
                             PrngState *rng) {
  size_t i;
  if (!x || n == 0u || noise_power <= 0.0) return;
  double sigma = sqrt(noise_power);
  for (i = 0; i < n; ++i)
    x[i] += sigma * prng_gauss(rng);
}

/* --- Vector scale --- */

static void scale_complex(Complex *x, size_t n, double amp) {
  size_t i;
  if (!x || n == 0u) return;
#pragma omp parallel for
  for (i = 0; i < n; ++i) {
    x[i].re *= amp;
    x[i].im *= amp;
  }
}

static void scale_real(double *x, size_t n, double amp) {
  size_t i;
  if (!x || n == 0u) return;
#pragma omp parallel for
  for (i = 0; i < n; ++i)
    x[i] *= amp;
}

/* --- Metric computation --- */

StageMetric compute_metric_complex(const char *stage, const char *domain,
                                   const Complex *restrict ref,
                                   const Complex *restrict sig, size_t n) {
  StageMetric m;
  m.stage = stage;
  m.domain = domain;
  m.gain_db = NAN;
  m.nf_db = NAN;
  m.filter_len = 0;
  m.is_limiter = 0;
  m.signal_power = mean_power_complex(ref, n);
  m.noise_power = mean_noise_power_complex(sig, ref, n);

  if (m.signal_power > 0.0 && m.noise_power > 0.0) {
    m.snr_db = lin_to_db(m.signal_power / m.noise_power);
    m.evm_pct = sqrt(m.noise_power / m.signal_power) * 100.0;
  } else if (m.signal_power > 0.0 && m.noise_power == 0.0) {
    m.snr_db = INFINITY;
    m.evm_pct = 0.0;
  } else {
    m.snr_db = -INFINITY;
    m.evm_pct = NAN;
  }
  return m;
}

StageMetric compute_metric_real(const char *stage, const char *domain,
                                const double *restrict ref,
                                const double *restrict sig, size_t n) {
  StageMetric m;
  m.stage = stage;
  m.domain = domain;
  m.gain_db = NAN;
  m.nf_db = NAN;
  m.filter_len = 0;
  m.is_limiter = 0;
  m.signal_power = mean_power_real(ref, n);
  m.noise_power = mean_noise_power_real(sig, ref, n);

  if (m.signal_power > 0.0 && m.noise_power > 0.0) {
    m.snr_db = lin_to_db(m.signal_power / m.noise_power);
  } else if (m.signal_power > 0.0 && m.noise_power == 0.0) {
    m.snr_db = INFINITY;
  } else {
    m.snr_db = -INFINITY;
  }
  m.evm_pct = NAN;
  return m;
}

/* ============================================================================
 * apply_stage_complex — Complex baseband stage processor
 *
 * Friis formula noise injection:
 *   F = 10^(NF_dB / 10)       — noise figure in linear scale
 *   pn_add = N_t0_v2 × g_lin × (F - 1)  — noise power in V² domain
 *   where N_t0_v2 = N_t0_W × R_LOAD_OHM converts W → V²
 * ============================================================================ */

StageMetric apply_stage_complex(const StageModel *stg,
                                Complex *restrict ref,
                                Complex *restrict sig, size_t n,
                                const char *domain, double N_t0_W,
                                double *N_current, double *Gain_total,
                                double P_sig_in_W, PrngState *rng) {
  double g_lin, F, pn_add, N_t0_v2;
  size_t i;

  N_t0_v2 = N_t0_W * R_LOAD_OHM;

  /* STEP 0: NONLINEARITY (IP3 and P1dB) */
  if (stg->ip3_dbm != INFINITY || stg->p1db_dbm != INFINITY) {
    double iip3_v2 = 0, ip1db_v2 = 0;
    if (stg->ip3_dbm != INFINITY)
      iip3_v2 = pow(10.0, (stg->ip3_dbm - 30.0) / 10.0) * R_LOAD_OHM;
    if (stg->p1db_dbm != INFINITY)
      ip1db_v2 = pow(10.0, (stg->p1db_dbm - 30.0) / 10.0) * R_LOAD_OHM;

#pragma omp parallel for
    for (i = 0; i < n; i++) {
      double p_in = ref[i].re * ref[i].re + ref[i].im * ref[i].im;
      double s = 1.0;
      if (stg->ip3_dbm != INFINITY) {
        double comp = (1.0 / 3.0) * (p_in / iip3_v2);
        if (comp > 0.9) comp = 0.9;
        s *= (1.0 - comp);
      }
      if (stg->p1db_dbm != INFINITY && p_in > (ip1db_v2 * 0.794))
        s *= sqrt(ip1db_v2 / p_in);
      ref[i].re *= s;
      ref[i].im *= s;
    }

#pragma omp parallel for
    for (i = 0; i < n; i++) {
      double p_in = sig[i].re * sig[i].re + sig[i].im * sig[i].im;
      double s = 1.0;
      if (stg->ip3_dbm != INFINITY) {
        double comp = (1.0 / 3.0) * (p_in / iip3_v2);
        if (comp > 0.9) comp = 0.9;
        s *= (1.0 - comp);
      }
      if (stg->p1db_dbm != INFINITY && p_in > (ip1db_v2 * 0.794))
        s *= sqrt(ip1db_v2 / p_in);
      sig[i].re *= s;
      sig[i].im *= s;
    }
  }

  /* STEP 1: GAIN — voltage gain = sqrt(power_gain_linear) */
  g_lin = db_to_lin_power(stg->gain_db);
  double amp_gain = sqrt(g_lin);
  scale_complex(ref, n, amp_gain);
  scale_complex(sig, n, amp_gain);

  /* STEP 2: NOISE INJECTION (Friis formula) */
  F = db_to_lin_power(stg->nf_db);
  if (F < 1.0) F = 1.0;

  /* pn_add in V² domain; only add noise to signal, not reference */
  pn_add = N_t0_v2 * g_lin * (F - 1.0);
  if (pn_add > 0.0)
    add_awgn_complex_re(sig, n, pn_add, rng);

  /* STEP 3: Friis analytic tracker */
  {
    double P_added_W = N_t0_W * g_lin * (F - 1.0);
    *Gain_total *= g_lin;
    *N_current = (*N_current) * g_lin + P_added_W;
  }

  /* STEP 4: LIMITER */
  if (stg->is_limiter) {
    const double max_amp = 10.0;
    for (i = 0; i < n; ++i) {
      double mag_ref = sqrt(ref[i].re * ref[i].re + ref[i].im * ref[i].im);
      double mag_sig = sqrt(sig[i].re * sig[i].re + sig[i].im * sig[i].im);
      if (mag_ref > max_amp && mag_ref > 0.0) {
        ref[i].re = (ref[i].re / mag_ref) * max_amp;
        ref[i].im = (ref[i].im / mag_ref) * max_amp;
      }
      if (mag_sig > max_amp && mag_sig > 0.0) {
        sig[i].re = (sig[i].re / mag_sig) * max_amp;
        sig[i].im = (sig[i].im / mag_sig) * max_amp;
      }
    }
  }

  /* Measure EVM and compute analytic Friis SNR */
  {
    StageMetric m_out = compute_metric_complex(stg->name, domain, ref, sig, n);
    m_out.gain_db = stg->gain_db;
    m_out.nf_db = stg->nf_db;
    m_out.filter_len = stg->filter_len;
    m_out.is_limiter = stg->is_limiter;

    if (N_current && Gain_total && *N_current > 0.0) {
      double P_sig_curr = P_sig_in_W * (*Gain_total);
      m_out.snr_db = lin_to_db(P_sig_curr / (*N_current));
    }
    return m_out;
  }
}

/* ============================================================================
 * apply_stage_realistic — Real RF stage processor with bandwidth correction
 *
 * Same Friis formula as apply_stage_complex, but the noise power includes
 * an RF bandwidth correction factor:
 *   pn_add = N_t0_v2 × g_lin × (F - 1) × (fs_hz / (2 × B_NOISE_HZ))
 *
 * This spreads the noise across the full RF bandwidth [0, fs/2] so that
 * after downconversion (selecting ~B_NOISE bandwidth), the correct
 * baseband noise power is recovered.
 * ============================================================================ */

StageMetric apply_stage_realistic(const StageModel *stg,
                                  double *restrict ref,
                                  double *restrict sig,
                                  size_t n, const char *domain,
                                  double N_t0_W, double fs_hz,
                                  double __attribute__((unused)) fc_hz,
                                  double *N_current, double *Gain_total,
                                  double P_sig_in_W, PrngState *rng) {
  double g_lin, F, pn_add, N_t0_v2;
  size_t i;

  N_t0_v2 = N_t0_W * R_LOAD_OHM;

  /* STEP 0: NONLINEARITY (IP3 and P1dB) */
  if (stg->ip3_dbm != INFINITY || stg->p1db_dbm != INFINITY) {
    double iip3_v2 = 0, ip1db_v2 = 0;
    if (stg->ip3_dbm != INFINITY)
      iip3_v2 = pow(10.0, (stg->ip3_dbm - 30.0) / 10.0) * R_LOAD_OHM;
    if (stg->p1db_dbm != INFINITY)
      ip1db_v2 = pow(10.0, (stg->p1db_dbm - 30.0) / 10.0) * R_LOAD_OHM;

#pragma omp parallel for
    for (i = 0; i < n; i++) {
      double p_in = sig[i] * sig[i];
      double s = 1.0;
      if (stg->ip3_dbm != INFINITY) {
        double comp = (1.0 / 3.0) * (p_in / iip3_v2);
        if (comp > 0.9) comp = 0.9;
        s *= (1.0 - comp);
      }
      if (stg->p1db_dbm != INFINITY && p_in > (ip1db_v2 * 0.794))
        s *= sqrt(ip1db_v2 / p_in);
      sig[i] *= s;
    }
  }

  /* STEP 1: GAIN */
  g_lin = db_to_lin_power(stg->gain_db);
  double amp_gain = sqrt(g_lin);
  scale_real(ref, n, amp_gain);
  scale_real(sig, n, amp_gain);

  /* STEP 2: NOISE INJECTION (Friis formula + RF bandwidth correction) */
  F = db_to_lin_power(stg->nf_db);
  if (F < 1.0) F = 1.0;

  pn_add = N_t0_v2 * g_lin * (F - 1.0) * (fs_hz / (2.0 * B_NOISE_HZ));
  if (pn_add > 0.0)
    add_awgn_real_re(sig, n, pn_add, rng);

  /* STEP 3: Friis analytic tracker */
  {
    double P_added_W = N_t0_W * g_lin * (F - 1.0);
    *Gain_total *= g_lin;
    *N_current = (*N_current) * g_lin + P_added_W;
  }

  /* STEP 4: LIMITER */
  if (stg->is_limiter) {
    const double max_amp = 10.0;
    for (i = 0; i < n; ++i) {
      if (ref[i] > max_amp) ref[i] = max_amp;
      else if (ref[i] < -max_amp) ref[i] = -max_amp;
      if (sig[i] > max_amp) sig[i] = max_amp;
      else if (sig[i] < -max_amp) sig[i] = -max_amp;
    }
  }

  {
    StageMetric m_out = compute_metric_real(stg->name, domain, ref, sig, n);
    m_out.gain_db = stg->gain_db;
    m_out.nf_db = stg->nf_db;
    m_out.filter_len = stg->filter_len;
    m_out.is_limiter = stg->is_limiter;

    if (N_current && Gain_total && *N_current > 0.0) {
      double P_sig_curr = P_sig_in_W * (*Gain_total);
      m_out.snr_db = lin_to_db(P_sig_curr / (*N_current));
    }
    return m_out;
  }
}
