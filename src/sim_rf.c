#define _USE_MATH_DEFINES
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <omp.h>

#include "sim_rf.h"
#include "rf_setup.h"
#include "physics.h"
#include "math_utils.h"
#include "metrics.h"
#include "stage_artifacts.h"
#include "fft.h"
#include "soa_utils.h"
#include "spectrum.h"
#include "prng.h"

/* Per-thread PRNG for RF simulation */
static PrngState rng_threads[PRNG_MAX_OMP_THREADS];

/* ============================================================================
 * ALIGNED ALLOCATION HELPER
 * ============================================================================ */

#define ALIGNMENT 64u

static void *alloc_aligned(size_t elem_size, size_t count) {
  size_t raw = elem_size * count;
  size_t aligned = (raw + ALIGNMENT - 1u) & ~(ALIGNMENT - 1u);
  void *p = aligned_alloc(ALIGNMENT, aligned);
  if (p) memset(p, 0, raw);
  return p;
}

#define ALLOC_ALIGNED_D(count) ((double *)alloc_aligned(sizeof(double), (count)))

/* ============================================================================
 * AUTO-GAIN HELPERS
 * ============================================================================ */

double complex_real_vpp(const Complex *x, size_t n) {
  size_t i;
  double mn; /* Minimum I value found */
  double mx; /* Maximum I value found */

  if (!x || n == 0u) {
    return 0.0;
  }

  mn = x[0].re;
  mx = x[0].re;
  for (i = 1u; i < n; ++i) {
    if (x[i].re < mn)
      mn = x[i].re;
    if (x[i].re > mx)
      mx = x[i].re;
  }

  return mx - mn;
}

/* ============================================================================
 * RF SIGNAL PROCESSING FUNCTIONS
 * ============================================================================ */

void sim_rf_upconvert(const double *env_re, const double *env_im, size_t n,
                      double fs_hz, double fc_hz, double *rf_out) {
  const double dtheta = 2.0 * M_PI * fc_hz / fs_hz;
  size_t i;
#pragma omp parallel for schedule(static)
  for (i = 0; i < n; ++i) {
    double theta = (double)i * dtheta;
    rf_out[i] = env_re[i] * cos(theta) - env_im[i] * sin(theta);
  }
}

size_t sim_rf_downconvert(const double *rf, size_t n, double fs_hz,
                          double fc_hz, double cutoff_hz, size_t dec_factor,
                          double *bb_re, double *bb_im,
                          double *i_raw, double *q_raw) {
  const double dtheta = 2.0 * M_PI * fc_hz / fs_hz;
  double alpha;
  size_t i, out_idx = 0;
  double i_state = 0.0, q_state = 0.0;

  if (cutoff_hz <= 0.0) cutoff_hz = fs_hz / 16.0;
  alpha = exp(-2.0 * M_PI * cutoff_hz / fs_hz);

#pragma omp parallel for schedule(static)
  for (i = 0; i < n; ++i) {
    double theta = (double)i * dtheta;
    i_raw[i] = 2.0 * rf[i] * cos(theta);
    q_raw[i] = -2.0 * rf[i] * sin(theta);
  }

  for (i = 0; i < n; ++i) {
    i_state = (1.0 - alpha) * i_raw[i] + alpha * i_state;
    q_state = (1.0 - alpha) * q_raw[i] + alpha * q_state;
    if (i % dec_factor == 0) {
      bb_re[out_idx] = i_state;
      bb_im[out_idx] = q_state;
      out_idx++;
    }
  }
  return out_idx;
}

/* ============================================================================
 * STRUCTURE-OF-ARRAYS (SoA) SIGNAL PROCESSING HELPERS
 * ============================================================================ */

void add_awgn_soa(double *restrict re, double *restrict im, size_t n,
                  double noise_power) {
  size_t i;
  double sigma;
  if (!re || !im || n == 0u || noise_power <= 0.0)
    return;
  sigma = sqrt(noise_power / 2.0);
#pragma omp parallel for schedule(static)
  for (i = 0; i < n; ++i) {
    re[i] += sigma * prng_gauss_parallel(rng_threads);
    im[i] += sigma * prng_gauss_parallel(rng_threads);
  }
}

void scale_soa(double *restrict re, double *restrict im, size_t n,
                double amp) {
  size_t i;
  if (!re || !im || n == 0u)
    return;
#pragma omp parallel for
  for (i = 0; i < n; ++i) {
    re[i] *= amp;
    im[i] *= amp;
  }
}

double mean_power_soa(const double *restrict re,
                       const double *restrict im, size_t n) {
  size_t i;
  double p = 0.0;
  if (!re || !im || n == 0u)
    return 0.0;
#pragma omp parallel for reduction(+ : p)
  for (i = 0; i < n; ++i) {
    p += re[i] * re[i] + im[i] * im[i];
  }
  return p / (double)n;
}

double mean_noise_power_soa(const double *restrict sig_re,
                             const double *restrict sig_im,
                             const double *restrict ref_re,
                             const double *restrict ref_im, size_t n) {
  size_t i;
  double p = 0.0;
  if (!sig_re || !sig_im || !ref_re || !ref_im || n == 0u)
    return 0.0;
#pragma omp parallel for reduction(+ : p)
  for (i = 0; i < n; ++i) {
    double dr = sig_re[i] - ref_re[i];
    double di = sig_im[i] - ref_im[i];
    p += dr * dr + di * di;
  }
  return p / (double)n;
}

StageMetric apply_stage_soa(
    const StageModel *stg, double *restrict ref_re, double *restrict ref_im,
    double *restrict sig_re, double *restrict sig_im, size_t n,
    const char *domain, double N_t0_W, double *N_current, double *Gain_total,
    double P_sig_in_W) {
  double amp_gain, g_lin, F, pn_add, N_t0_v2;
  size_t i;

  N_t0_v2 = N_t0_W * R_LOAD_OHM;

  /* --- STEP 0: NONLINEARITY --- */
  if (stg->ip3_dbm != INFINITY || stg->p1db_dbm != INFINITY) {
    double iip3_v2 = 0, ip1db_v2 = 0;
    if (stg->ip3_dbm != INFINITY) {
      double iip3_w = pow(10.0, (stg->ip3_dbm - 30.0) / 10.0);
      iip3_v2 = iip3_w * R_LOAD_OHM;
    }
    if (stg->p1db_dbm != INFINITY) {
      double ip1db_w = pow(10.0, (stg->p1db_dbm - 30.0) / 10.0);
      ip1db_v2 = ip1db_w * R_LOAD_OHM;
    }

#pragma omp parallel for schedule(static)
    for (i = 0; i < n; i++) {
      double p_in_inst = ref_re[i] * ref_re[i] + ref_im[i] * ref_im[i];
      double s = 1.0;
      if (stg->ip3_dbm != INFINITY) {
        double comp = (1.0 / 3.0) * (p_in_inst / iip3_v2);
        if (comp > 0.9) comp = 0.9;
        s *= (1.0 - comp);
      }
      if (stg->p1db_dbm != INFINITY) {
        if (p_in_inst > (ip1db_v2 * 0.794))
          s *= sqrt(ip1db_v2 / p_in_inst);
      }
      ref_re[i] *= s;
      ref_im[i] *= s;
    }

#pragma omp parallel for schedule(static)
    for (i = 0; i < n; i++) {
      double p_in_inst = sig_re[i] * sig_re[i] + sig_im[i] * sig_im[i];
      double s = 1.0;
      if (stg->ip3_dbm != INFINITY) {
        double comp = (1.0 / 3.0) * (p_in_inst / iip3_v2);
        if (comp > 0.9) comp = 0.9;
        s *= (1.0 - comp);
      }
      if (stg->p1db_dbm != INFINITY) {
        if (p_in_inst > (ip1db_v2 * 0.794))
          s *= sqrt(ip1db_v2 / p_in_inst);
      }
      sig_re[i] *= s;
      sig_im[i] *= s;
    }
  }

  /* --- STEP 1: GAIN --- */
  g_lin = db_to_lin_power(stg->gain_db);
  amp_gain = sqrt(g_lin);
  scale_soa(ref_re, ref_im, n, amp_gain);
  scale_soa(sig_re, sig_im, n, amp_gain);

  /* --- STEP 2: NOISE INJECTION --- */
  F = db_to_lin_power(stg->nf_db);
  if (F < 1.0) F = 1.0;
  pn_add = N_t0_v2 * g_lin * (F - 1.0);
  if (pn_add > 0.0) {
    add_awgn_soa(sig_re, sig_im, n, pn_add);
  }

  /* --- STEP 3: Friis tracker --- */
  {
    double P_added_W = N_t0_W * g_lin * (F - 1.0);
    *Gain_total *= g_lin;
    *N_current = (*N_current) * g_lin + P_added_W;
  }

  /* --- STEP 4: LIMITER --- */
  if (stg->is_limiter) {
    const double max_amp = 10.0;
    for (i = 0; i < n; ++i) {
      double mag_r = sqrt(ref_re[i] * ref_re[i] + ref_im[i] * ref_im[i]);
      double mag_s = sqrt(sig_re[i] * sig_re[i] + sig_im[i] * sig_im[i]);
      if (mag_r > max_amp && mag_r > 0.0) {
        ref_re[i] = (ref_re[i] / mag_r) * max_amp;
        ref_im[i] = (ref_im[i] / mag_r) * max_amp;
      }
      if (mag_s > max_amp && mag_s > 0.0) {
        sig_re[i] = (sig_re[i] / mag_s) * max_amp;
        sig_im[i] = (sig_im[i] / mag_s) * max_amp;
      }
    }
  }

  /* Compute metric using SoA */
  {
    StageMetric m_out;
    m_out.stage = stg->name;
    m_out.domain = domain;
    m_out.gain_db = stg->gain_db;
    m_out.nf_db = stg->nf_db;
    m_out.filter_len = stg->filter_len;
    m_out.is_limiter = stg->is_limiter;
    m_out.signal_power = mean_power_soa(ref_re, ref_im, n);
    m_out.noise_power = mean_noise_power_soa(sig_re, sig_im, ref_re, ref_im, n);

    if (m_out.signal_power > 0.0 && m_out.noise_power > 0.0) {
      m_out.snr_db = lin_to_db(m_out.signal_power / m_out.noise_power);
      m_out.evm_pct = sqrt(m_out.noise_power / m_out.signal_power) * 100.0;
    } else if (m_out.signal_power > 0.0 && m_out.noise_power == 0.0) {
      m_out.snr_db = INFINITY;
      m_out.evm_pct = 0.0;
    } else {
      m_out.snr_db = -INFINITY;
      m_out.evm_pct = NAN;
    }

    if (N_current && Gain_total && *N_current > 0.0) {
      double P_sig_curr = P_sig_in_W * (*Gain_total);
      m_out.snr_db = lin_to_db(P_sig_curr / (*N_current));
    }
    return m_out;
  }
}

StageMetric apply_stage_real_fused(const StageModel *stg,
                                   double *restrict ref, double *restrict sig,
                                   size_t n, const char *domain,
                                   double N_t0_W, double fs_hz,
                                   double fc_hz,
                                   double *N_current, double *Gain_total,
                                   double P_sig_in_W) {
  double g_lin, F, pn_add, N_t0_v2;
  size_t i;

  (void)fc_hz;
  N_t0_v2 = N_t0_W * R_LOAD_OHM;

  g_lin = db_to_lin_power(stg->gain_db);
  double amp_gain = sqrt(g_lin);
  F = db_to_lin_power(stg->nf_db);
  if (F < 1.0) F = 1.0;
  pn_add = N_t0_v2 * g_lin * (F - 1.0) * (fs_hz / (2.0 * B_NOISE_HZ));

  int has_ip3 = (stg->ip3_dbm != INFINITY);
  int has_p1db = (stg->p1db_dbm != INFINITY);
  double iip3_v2 = 0, ip1db_v2 = 0;
  if (has_ip3) {
    iip3_v2 = pow(10.0, (stg->ip3_dbm - 30.0) / 10.0) * R_LOAD_OHM;
  }
  if (has_p1db) {
    ip1db_v2 = pow(10.0, (stg->p1db_dbm - 30.0) / 10.0) * R_LOAD_OHM;
  }

  int has_am_pm = (stg->am_pm_coeff != 0.0);
  double p1db_threshold_v2 = has_p1db ? ip1db_v2 : 0.0;

  double sigma = (pn_add > 0.0) ? sqrt(pn_add) : 0.0;
  int has_limiter = stg->is_limiter;
  const double max_amp = 10.0;

#pragma omp parallel for schedule(static)
  for (i = 0; i < n; i++) {
    double s_scale = 1.0;
    if (has_ip3 || has_p1db) {
      double p_in = sig[i] * sig[i];
      if (has_ip3) {
        double comp = (1.0 / 3.0) * (p_in / iip3_v2);
        if (comp > 0.9) comp = 0.9;
        s_scale *= (1.0 - comp);
      }
      if (has_p1db) {
        if (p_in > (ip1db_v2 * 0.794)) {
          s_scale *= sqrt(ip1db_v2 / p_in);
        }
      }
    }
    if (has_am_pm) {
      double p_in = sig[i] * sig[i];
      if (p_in > p1db_threshold_v2) {
        double p_in_dBm = 10.0 * log10(p_in / R_LOAD_OHM) + 30.0;
        double p1db_dBm = stg->p1db_dbm;
        double phase_shift_deg = stg->am_pm_coeff * (p_in_dBm - p1db_dBm);
        double phi = phase_shift_deg * (3.14159265358979323846 / 180.0);
        s_scale *= cos(phi);
      }
    }

    double r = ref[i] * amp_gain;
    double s = sig[i] * s_scale * amp_gain;

    if (sigma > 0.0) {
      s += sigma * prng_gauss_parallel(rng_threads);
    }

    if (has_limiter) {
      if (r > max_amp) r = max_amp;
      else if (r < -max_amp) r = -max_amp;
      if (s > max_amp) s = max_amp;
      else if (s < -max_amp) s = -max_amp;
    }

    ref[i] = r;
    sig[i] = s;
  }

  {
    double P_added_W = N_t0_W * g_lin * (F - 1.0);
    *Gain_total *= g_lin;
    *N_current = (*N_current) * g_lin + P_added_W;
  }

  {
    StageMetric m_out;
    m_out.stage = stg->name;
    m_out.domain = domain;
    m_out.gain_db = stg->gain_db;
    m_out.nf_db = stg->nf_db;
    m_out.filter_len = stg->filter_len;
    m_out.is_limiter = stg->is_limiter;

    double p_ref = 0.0, p_diff = 0.0;
#pragma omp parallel for reduction(+:p_ref, p_diff)
    for (i = 0; i < n; i++) {
      p_ref += ref[i] * ref[i];
      p_diff += (sig[i] - ref[i]) * (sig[i] - ref[i]);
    }
    m_out.signal_power = p_ref / (double)n;
    m_out.noise_power = p_diff / (double)n;

    if (m_out.signal_power > 0.0 && m_out.noise_power > 0.0) {
      m_out.snr_db = lin_to_db(m_out.signal_power / m_out.noise_power);
      m_out.evm_pct = sqrt(m_out.noise_power / m_out.signal_power) * 100.0;
    } else if (m_out.signal_power > 0.0 && m_out.noise_power == 0.0) {
      m_out.snr_db = INFINITY;
      m_out.evm_pct = 0.0;
    } else {
      m_out.snr_db = -INFINITY;
      m_out.evm_pct = NAN;
    }

    if (N_current && Gain_total && *N_current > 0.0) {
      double P_sig_curr = P_sig_in_W * (*Gain_total);
      m_out.snr_db = lin_to_db(P_sig_curr / (*N_current));
    }
    return m_out;
  }
}

size_t synchronize_and_downsample(const Complex *bb_stream, size_t n_in,
                                         size_t nsym, int sps, double rolloff,
                                         const Complex *tx, Complex *out) {
  size_t i;
  size_t sym_count = 0u;
  size_t rx_probe_n;
  size_t tx_probe_n;
  int best_lag = 0;
  double best_lag_corr = -1.0;

  size_t n_tot = n_in;
  size_t pulse_len = 0u;
  Complex *matched = NULL;
  Complex *sym_stream = NULL;

  if (rolloff > 0.0) {
    const int span = 20;
    pulse_len = (size_t)span * (size_t)sps + 1u;
  } else {
    pulse_len = (size_t)sps;
  }

  matched = (Complex *)calloc(n_tot + pulse_len - 1u, sizeof(Complex));
  if (!matched)
    return 0u;

  /* 1. RRC matched filter (fallback to rectangular only when rolloff <= 0) */
  if (rolloff > 0.0) {
    double *pulse = (double *)calloc(pulse_len, sizeof(double));
    size_t k;
    double norm_sq = 0.0;

    if (!pulse) {
      free(matched);
      return 0u;
    }

    for (k = 0u; k < pulse_len; ++k) {
      const double t =
          ((double)k - ((double)pulse_len - 1.0) / 2.0) / (double)sps;
      const double value = rrc_tap_value(t, rolloff);
      pulse[k] = value;
      norm_sq += value * value;
    }

    if (norm_sq > 0.0) {
      const double norm = sqrt(norm_sq);
      for (k = 0u; k < pulse_len; ++k) {
        pulse[k] /= norm;
      }
    }

    /* FFT-based convolution (optimization: O(n log n) vs O(n × pulse_len)) */
    fft_convolve_complex(bb_stream, n_tot, pulse, pulse_len,
                         matched, n_tot + pulse_len - 1u);

    free(pulse);
  } else {
    for (i = 0u; i < n_tot + pulse_len - 1u; ++i) {
      size_t kmax = (i + 1u < pulse_len) ? (i + 1u) : pulse_len;
      size_t k;

      for (k = 0u; k < kmax; ++k) {
        if (i >= k && (i - k) < n_tot) {
          matched[i].re += bb_stream[i - k].re;
          matched[i].im += bb_stream[i - k].im;
        }
      }
    }
  }

  /* 2. Build a symbol-rate stream using the same fixed decimation as MATLAB. */
  {
    size_t sym_capacity = 0u;

    if (n_tot + pulse_len > 0u) {
      sym_capacity = (n_tot + pulse_len - 1u + (size_t)sps - 1u) / (size_t)sps;
    }

    sym_stream = (Complex *)calloc(sym_capacity > 0u ? sym_capacity : 1u,
                                   sizeof(Complex));
    if (!sym_stream) {
      free(matched);
      return 0u;
    }

    for (i = 0u;; ++i) {
      size_t idx = i * (size_t)sps;
      if (idx >= n_tot + pulse_len - 1u) {
        break;
      }
      sym_stream[sym_count++] = matched[idx];
    }
  }

  /* 4. Search for the MATLAB-style delay on a short probe window. */
  rx_probe_n = (sym_count < 2000u) ? sym_count : 2000u;
  tx_probe_n = (nsym < 500u) ? nsym : 500u;
  if (rx_probe_n > 0u && tx_probe_n > 0u) {
    const int min_lag = -(int)(tx_probe_n - 1u);
    const int max_lag = (int)(rx_probe_n - 1u);
    int lag;

    for (lag = min_lag; lag <= max_lag; ++lag) {
      const size_t rx_start = (lag > 0) ? (size_t)lag : 0u;
      const size_t tx_start = (lag < 0) ? (size_t)(-lag) : 0u;
      size_t overlap = rx_probe_n - rx_start;
      const size_t tx_overlap = tx_probe_n - tx_start;
      double corr_re = 0.0;
      double corr_im = 0.0;
      size_t k;

      if (tx_overlap < overlap) {
        overlap = tx_overlap;
      }

      if (overlap == 0u) {
        continue;
      }

      for (k = 0u; k < overlap; ++k) {
        const Complex rx = sym_stream[rx_start + k];
        const Complex txv = tx[tx_start + k];

        /* conj(tx) * rx */
        corr_re += txv.re * rx.re + txv.im * rx.im;
        corr_im += txv.re * rx.im - txv.im * rx.re;
      }

      {
        const double corr_mag = sqrt(corr_re * corr_re + corr_im * corr_im);
        if (corr_mag > best_lag_corr) {
          best_lag_corr = corr_mag;
          best_lag = lag;
        }
      }
    }
  }

  /* MATLAB fallback: keep the default span when the estimated lag is negative.
   */
  {
    size_t opt_delay = (best_lag < 0) ? 20u : (size_t)best_lag;
    size_t eval_len;

    if (opt_delay > sym_count) {
      opt_delay = sym_count;
    }

    eval_len = (sym_count > opt_delay) ? (sym_count - opt_delay) : 0u;
    if (eval_len > nsym) {
      eval_len = nsym;
    }

    if (eval_len > 0u) {
      double num_re = 0.0;
      double num_im = 0.0;
      double den = 0.0;
      size_t k;

      for (k = 0u; k < eval_len; ++k) {
        const Complex rx = sym_stream[opt_delay + k];
        const Complex txv = tx[k];

        /* Least-squares channel estimate: alpha = (tx' * rx) / (tx' * tx) */
        num_re += txv.re * rx.re + txv.im * rx.im;
        num_im += txv.re * rx.im - txv.im * rx.re;
        den += txv.re * txv.re + txv.im * txv.im;
      }

      if (den > 0.0) {
        const double alpha_re = num_re / den;
        const double alpha_im = num_im / den;

        /* Complex reciprocal: 1/alpha = conj(alpha) / |alpha|² */
        const double alpha_mag_sq = alpha_re * alpha_re + alpha_im * alpha_im;
        const double inv_re = alpha_re / alpha_mag_sq;
        const double inv_im = -alpha_im / alpha_mag_sq;

        for (k = 0u; k < eval_len; ++k) {
          const Complex rx = sym_stream[opt_delay + k];
          out[k].re = rx.re * inv_re - rx.im * inv_im;
          out[k].im = rx.re * inv_im + rx.im * inv_re;
        }
      } else {
        eval_len = 0u;
      }
    }

    for (i = eval_len; i < nsym; ++i) {
      out[i].re = 0.0;
      out[i].im = 0.0;
    }

    free(sym_stream);
    free(matched);
    return eval_len;
  }
}

/* ============================================================================
 * BRUTEFORCE RF SIMULATION
 * ============================================================================ */

int simulate_bruteforce_rf(
    const SimConfig *cfg, const StageModelsConfig *stage_cfg,
    const Complex *tx_symbols, const Complex *constellation_template,
    size_t constellation_count, size_t nsym, StageMetric *metrics,
    size_t *metric_count, double *final_vpp, int *used_sps, double *used_fs_hz,
    const char *csv_dir, const char *const_dir, const char *trace_dir,
    const char *spectrum_dir) {

  RfSimBuffers bufs;
  RfSimParams params;

  if (rf_setup_compute_params(&params, cfg, stage_cfg, nsym) != 0) {
    return -1;
  }

  if (!params.rf_stages || !params.bb_stages || params.rf_stage_count == 0u ||
      params.bb_stage_count == 0u || !csv_dir || !const_dir || !trace_dir) {
    return -2;
  }

  /* Check metric buffer capacity */
  if (params.rf_stage_count + params.bb_stage_count + 2u > (size_t)MAX_RF_METRICS) {
    fprintf(
        stderr,
        "RF chain has %zu+%zu stages; increase MAX_METRICS (currently %d)\n",
        params.rf_stage_count, params.bb_stage_count, MAX_RF_METRICS);
    return -3;
  }

  if (rf_setup_allocate(&bufs, cfg, stage_cfg, nsym) != 0) {
    return -1;
  }

  double *env_re = bufs.env_re;
  double *env_im = bufs.env_im;
  double *rf_ref = bufs.rf_ref;
  double *rf_sig = bufs.rf_sig;
  double *bb_ref_re = bufs.bb_ref_re;
  double *bb_ref_im = bufs.bb_ref_im;
  double *bb_sig_re = bufs.bb_sig_re;
  double *bb_sig_im = bufs.bb_sig_im;
  Complex *ref_sym = bufs.ref_sym;
  Complex *sig_sym = bufs.sig_sym;
  double *temp_bb_ref_re = bufs.temp_bb_ref_re;
  double *temp_bb_ref_im = bufs.temp_bb_ref_im;
  double *temp_bb_sig_re = bufs.temp_bb_sig_re;
  double *temp_bb_sig_im = bufs.temp_bb_sig_im;
  Complex *temp_ref_sym = bufs.temp_ref_sym;
  Complex *temp_sig_sym = bufs.temp_sig_sym;
  Complex *temp_complex_buf = bufs.temp_complex_buf;
  double *i_raw = bufs.i_raw;
  double *q_raw = bufs.q_raw;

  int sps = params.sps;
  double fs_hz = params.fs_hz;
  size_t nrf = params.nrf;
  size_t nbb = params.nbb;
  size_t bb_sps = params.bb_sps;
  size_t dec_factor = params.dec_factor;
  const StageModel *rf_stages = params.rf_stages;
  const StageModel *bb_stages = params.bb_stages;
  size_t rf_stage_count = params.rf_stage_count;
  size_t bb_stage_count = params.bb_stage_count;

  size_t m = 0u; /* Metric counter */
  size_t i;

  /* MATLAB-exact physical noise constant: N_t0_W = k_B * T0 * B_noise (Watts) */
  const double N_t0_W = K_BOLTZMANN * cfg->t0_k * B_NOISE_HZ;
  /* Friis trackers */
  double N_current = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
  double Gain_total = 1.0;
  const double P_sig_in_W = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ *
                            db_to_lin_power(cfg->input_snr_db);

  prng_init_parallel(rng_threads, (uint32_t)cfg->seed);

  double t0 = omp_get_wtime();
  double t_pulse = 0, t_rf_stages = 0, t_downconv = 0, t_bb_stages = 0;

  if (rf_setup_build_envelope(&bufs, tx_symbols, nsym, cfg, &params) != 0) {
    rf_setup_free(&bufs);
    return -1;
  }

  /* Step 2: Add antenna noise to envelope, then upconvert — SoA */
  {
    double *env_noisy_re = ALLOC_ALIGNED_D(nrf);
    double *env_noisy_im = ALLOC_ALIGNED_D(nrf);
    if (!env_noisy_re || !env_noisy_im) {
      rf_setup_free(&bufs);
      free(env_noisy_re); free(env_noisy_im);
      return -1;
    }
    memcpy(env_noisy_re, env_re, nrf * sizeof(double));
    memcpy(env_noisy_im, env_im, nrf * sizeof(double));

    {
      const double P_noise_v2 =
          K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ * R_LOAD_OHM;
      add_awgn_soa(env_noisy_re, env_noisy_im, nrf, P_noise_v2);
    }

    /* Step 3: IQ-modulate both clean and noisy envelopes — SoA */
    sim_rf_upconvert(env_re, env_im, nrf, fs_hz, cfg->carrier_hz, rf_ref);
    sim_rf_upconvert(env_noisy_re, env_noisy_im, nrf, fs_hz, cfg->carrier_hz, rf_sig);
    free(env_noisy_re);
    free(env_noisy_im);
  }

  /* Step 5: Record the INPUT RF trace metric */
  {
    metrics[m] =
        compute_stage_metric_real("input_rf", "rf_real", rf_ref, rf_sig, nrf);
    metrics[m].signal_power = P_sig_in_W;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db(P_sig_in_W / N_current);
    }
    write_trace_stage_artifacts(csv_dir, trace_dir, "traces", 0u, 1,
                                "input_rf", &metrics[m], "RF", rf_ref, rf_sig,
                                nrf, 24000u, fs_hz, cfg->carrier_hz);

    {
      const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      sim_rf_downconvert(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      sim_rf_downconvert(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                   dec_factor, temp_bb_sig_re, temp_bb_sig_im, i_raw, q_raw);

      /* Pack SoA → Complex for synchronize_and_downsample */
      pack_complex(temp_bb_ref_re, temp_bb_ref_im, nbb, temp_complex_buf);
      size_t temp_ref_n =
          synchronize_and_downsample(temp_complex_buf, nbb, nsym, bb_sps,
                                     cfg->rolloff, tx_symbols, temp_ref_sym);
      pack_complex(temp_bb_sig_re, temp_bb_sig_im, nbb, temp_complex_buf);
      size_t temp_sig_n =
          synchronize_and_downsample(temp_complex_buf, nbb, nsym, bb_sps,
                                     cfg->rolloff, tx_symbols, temp_sig_sym);
      size_t temp_eval_n =
          (temp_ref_n < temp_sig_n) ? temp_ref_n : temp_sig_n;

      StageMetric temp_metric = compute_stage_metric_complex(
          "input_rf", "rf_to_bb", temp_ref_sym, temp_sig_sym, temp_eval_n);
      metrics[m].evm_pct = temp_metric.evm_pct;

      write_constellation_stage_artifacts(
          csv_dir, const_dir, "receiver", 0u, 1, "input_rf", &metrics[m],
          "RF", constellation_template, constellation_count, tx_symbols,
          temp_sig_sym, temp_eval_n);
    }

    ++m;
  }

  t_pulse = omp_get_wtime();

  /* Step 6: Process through RF frontend stages */
  double current_center_hz = cfg->carrier_hz;
  for (i = 0u; i < rf_stage_count; ++i) {
    StageModel stage = rf_stages[i];

    metrics[m] =
        apply_stage_real_fused(&stage, rf_ref, rf_sig, nrf, "rf_real", N_t0_W, fs_hz,
                         cfg->carrier_hz, &N_current, &Gain_total, P_sig_in_W);
    if (stage.lo_hz > 0.0) {
      current_center_hz = fabs(current_center_hz - stage.lo_hz);
    }
    metrics[m].signal_power =
        P_sig_in_W * Gain_total; /* Align with Friis model */
    write_trace_stage_artifacts(csv_dir, trace_dir, "traces", i + 1u, 0,
                                stage.name, &metrics[m], "RF", rf_ref, rf_sig,
                                nrf, 6000u, fs_hz, current_center_hz);

    {
      double spectrum_span = 4.0e9;
      if (stage.lo_hz > 0.0 && current_center_hz < 1e9) {
        spectrum_span = 10.0e6;
      }
      write_stage_spectrum(spectrum_dir, "receiver", i + 1u, stage.name,
                           rf_ref, nrf, fs_hz,
                           current_center_hz, spectrum_span);
    }
    /* Generate Constellation & EVM for all RF Stages (SoA) -- baseline */
    {
      const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      sim_rf_downconvert(rf_ref, nrf, fs_hz, current_center_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      sim_rf_downconvert(rf_sig, nrf, fs_hz, current_center_hz, cutoff_hz,
                   dec_factor, temp_bb_sig_re, temp_bb_sig_im, i_raw, q_raw);

      pack_complex(temp_bb_ref_re, temp_bb_ref_im, nbb, temp_complex_buf);
      size_t temp_ref_n =
          synchronize_and_downsample(temp_complex_buf, nbb, nsym, bb_sps,
                                     cfg->rolloff, tx_symbols, temp_ref_sym);
      pack_complex(temp_bb_sig_re, temp_bb_sig_im, nbb, temp_complex_buf);
      size_t temp_sig_n =
          synchronize_and_downsample(temp_complex_buf, nbb, nsym, bb_sps,
                                     cfg->rolloff, tx_symbols, temp_sig_sym);
      size_t temp_eval_n =
          (temp_ref_n < temp_sig_n) ? temp_ref_n : temp_sig_n;

      StageMetric temp_metric = compute_stage_metric_complex(
          stage.name, "rf_to_bb", tx_symbols, temp_sig_sym, temp_eval_n);
      metrics[m].evm_pct = temp_metric.evm_pct;

      write_constellation_stage_artifacts(
          csv_dir, const_dir, "receiver", i + 1u, 0, stage.name, &metrics[m],
          "RF", constellation_template, constellation_count, tx_symbols,
          temp_sig_sym, temp_eval_n);
    }

    ++m;
  }

  t_rf_stages = omp_get_wtime();

  /* Step 7: Downconvert RF → complex baseband — SoA */
  {
    const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
    sim_rf_downconvert(rf_ref, nrf, fs_hz, current_center_hz, cutoff_hz,
                 dec_factor, bb_ref_re, bb_ref_im, i_raw, q_raw);
    sim_rf_downconvert(rf_sig, nrf, fs_hz, current_center_hz, cutoff_hz,
                 dec_factor, bb_sig_re, bb_sig_im, i_raw, q_raw);
  }

  /* IF Spectrum: complex FFT of real IF signal (imag=0)
   * rf_sig is unmodified by mix_down_soa (const param).
   * Complex FFT of real signal produces conjugate-symmetric output:
   * dual peaks at ±IF from the heterodyne mixer. */
  if (current_center_hz != cfg->carrier_hz) {
    size_t nfft = 1u; while (nfft * 2u <= nrf) nfft *= 2u;
    size_t m_fft = nfft + 2u;
    double *freq = malloc(m_fft * sizeof(double));
    double *mag = malloc(m_fft * sizeof(double));
    double *zero_im = calloc(nfft, sizeof(double));
    if (freq && mag && zero_im) {
      int nb = fft_complex_spectrum_dB(rf_sig, zero_im, nfft, fs_hz, freq, mag, m_fft);
      if (nb > 0) {
        /* IF Spectrum — medium zoom: filter original FFT to ±100 MHz */
        {
          double zoom_fl = -50.0e6, zoom_fh = 50.0e6;
          size_t nz = 0u;
          for (int ri = 0; ri < nb; ri++)
            if (freq[ri] >= zoom_fl && freq[ri] <= zoom_fh) nz++;
          if (nz > 1u) {
            double *zf = malloc(nz * sizeof(double));
            double *zm = malloc(nz * sizeof(double));
            if (zf && zm) {
              size_t wi = 0u;
              for (int ri = 0; ri < nb; ri++)
                if (freq[ri] >= zoom_fl && freq[ri] <= zoom_fh) {
                  zf[wi] = freq[ri]; zm[wi] = mag[ri]; wi++;
                }
              size_t max_d = 1000u;
              if (nz > max_d) {
                size_t stride = nz / max_d;
                for (size_t di = 0u; di < max_d; di++) {
                  double sa = 0.0;
                  for (size_t sj = 0u; sj < stride && di*stride+sj < nz; sj++)
                    sa += zm[di*stride+sj];
                  zm[di] = sa / (double)stride;
                  zf[di] = zf[di*stride + stride/2u];
                }
                nz = max_d;
              }
              char fpath[512];
              snprintf(fpath, sizeof(fpath), "%s/receiver_stage_07_full.svg", spectrum_dir);
              write_spectrum_svg(fpath, zf, zm, nz, 0.0, "IF Spectrum (full)");
            }
            free(zf); free(zm);
          }
        }
        double zoom_lo = -2.0e6, zoom_hi = 2.0e6;
        size_t wi = 0u, ri; size_t nc = (size_t)nb;
        for (ri = 0u; ri < nc; ri++) {
          if (freq[ri] >= zoom_lo && freq[ri] <= zoom_hi) {
            freq[wi] = freq[ri]; mag[wi] = mag[ri]; wi++;
          }
        }
        if (wi > 1u) {
          size_t max_d = 1000u, di; nc = wi;
          if (nc > max_d) {
            size_t stride = nc / max_d;
            for (di = 0u; di < max_d; di++) {
              double sa = 0.0; size_t sj, start = di * stride;
              for (sj = 0u; sj < stride && start + sj < nc; sj++)
                sa += mag[start + sj];
              mag[di] = sa / (double)stride;
              freq[di] = freq[start + stride/2u];
            }
            nc = max_d;
          }
          char path[512];
          snprintf(path, sizeof(path), "%s/receiver_stage_07_spectrum.svg", spectrum_dir);
          write_spectrum_svg(path, freq, mag, nc, 0.0, "IF Spectrum (dual sidebands)");

          {
            double f_min = freq[0], f_max = freq[nc - 1u], f_span = f_max - f_min;
            if (f_span > 0.0) {
              double neg_peak_mag = -INFINITY, pos_peak_mag = -INFINITY;
              double neg_peak_freq = 0.0, pos_peak_freq = 0.0;
              size_t ki;
              for (ki = 0u; ki < nc; ki++) {
                if (freq[ki] < 0.0 && mag[ki] > neg_peak_mag) {
                  neg_peak_mag = mag[ki];
                  neg_peak_freq = freq[ki];
                }
                if (freq[ki] > 0.0 && mag[ki] > pos_peak_mag) {
                  pos_peak_mag = mag[ki];
                  pos_peak_freq = freq[ki];
                }
              }
              const int ml = 80, mt = 50, plot_h = 630;
              const double px_per_hz = 870.0 / f_span;
              FILE *f = fopen(path, "r");
              if (f) {
                fseek(f, 0, SEEK_END);
                long fsz = ftell(f);
                fseek(f, 0, SEEK_SET);
                char *svg_buf = (char *)malloc((size_t)fsz + 1u);
                size_t nrd = 0;
                if (svg_buf) nrd = fread(svg_buf, 1, (size_t)fsz, f);
                fclose(f);
                if (svg_buf) {
                  svg_buf[nrd] = '\0';
                  char *closing = strstr(svg_buf, "</svg>");
                  if (closing) {
                    size_t prefix = (size_t)(closing - svg_buf);
                    f = fopen(path, "w");
                    if (f) {
                      fwrite(svg_buf, 1, prefix, f);
                      double xl = (double)ml + (neg_peak_freq - f_min) * px_per_hz;
                      double xr = (double)ml + (pos_peak_freq - f_min) * px_per_hz;
                      fprintf(f, "<line x1=\"%.1f\" y1=\"%d\" x2=\"%.1f\" y2=\"%d\" stroke=\"#dc2626\" stroke-dasharray=\"6,3\" stroke-width=\"1.5\"/>\n", xl, mt, xl, mt + plot_h);
                      fprintf(f, "<line x1=\"%.1f\" y1=\"%d\" x2=\"%.1f\" y2=\"%d\" stroke=\"#dc2626\" stroke-dasharray=\"6,3\" stroke-width=\"1.5\"/>\n", xr, mt, xr, mt + plot_h);
                      fprintf(f, "<text x=\"%.1f\" y=\"%d\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#dc2626\" font-weight=\"bold\">%.0f kHz</text>\n", xl, mt - 6, neg_peak_freq / 1e3);
                      fprintf(f, "<text x=\"%.1f\" y=\"%d\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"13\" fill=\"#dc2626\" font-weight=\"bold\">+%.0f kHz</text>\n", xr, mt - 6, pos_peak_freq / 1e3);
                      fprintf(f, "</svg>\n");
                      fclose(f);
                    }
                  }
                  free(svg_buf);
                }
              }
            }
          }
        }
      }
      free(freq); free(mag); free(zero_im);
    }
  }

  t_downconv = omp_get_wtime();

  /* Step 8: Record downconversion metric */
  {
    metrics[m].stage = "MIX2_Downconv";
    metrics[m].domain = "rf_to_bb";
    metrics[m].gain_db = NAN;
    metrics[m].nf_db = NAN;
    metrics[m].filter_len = 0;
    metrics[m].is_limiter = 0;
    metrics[m].signal_power = mean_power_soa(bb_ref_re, bb_ref_im, nbb);
    metrics[m].noise_power =
        mean_noise_power_soa(bb_sig_re, bb_sig_im, bb_ref_re, bb_ref_im, nbb);
    if (metrics[m].signal_power > 0.0 && metrics[m].noise_power > 0.0) {
      metrics[m].snr_db =
          lin_to_db(metrics[m].signal_power / metrics[m].noise_power);
    } else {
      metrics[m].snr_db = -INFINITY;
    }

    /* Synchronize to find EVM and constellation SVG */
    pack_complex(bb_ref_re, bb_ref_im, nbb, temp_complex_buf);
    size_t ref_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, ref_sym);
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    size_t sig_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);
    size_t eval_n = (ref_eval_n < sig_eval_n) ? ref_eval_n : sig_eval_n;

    StageMetric temp_metric = compute_stage_metric_complex(
        "MIX2_Downconv", "rf_to_bb", ref_sym, sig_sym, eval_n);
    metrics[m].evm_pct = temp_metric.evm_pct;

    write_trace_stage_artifacts(csv_dir, trace_dir, "traces",
                                rf_stage_count + 1u, 0, "MIX2_Downconv",
                                &metrics[m], "BB", bb_ref_re, bb_sig_re, nbb,
                                600u, cfg->symbol_rate_hz * 8.0, 0.0);
    write_constellation_stage_artifacts(
        csv_dir, const_dir, "receiver", rf_stage_count + 1u, 0, "MIX2_Downconv",
        &metrics[m], "BB", constellation_template, constellation_count,
        tx_symbols, sig_sym, eval_n);

    ++m;
  }

  /* Step 9: Process through post-mix BB stages — SoA */
  for (i = 0u; i < bb_stage_count; ++i) {
    StageModel stage = bb_stages[i];

    metrics[m] =
        apply_stage_soa(&stage, bb_ref_re, bb_ref_im, bb_sig_re, bb_sig_im, nbb,
                        "rf_to_bb", N_t0_W, &N_current, &Gain_total, P_sig_in_W);
    metrics[m].signal_power = P_sig_in_W * Gain_total;

    /* Pack SoA → Complex for constellation analysis */
    pack_complex(bb_ref_re, bb_ref_im, nbb, temp_complex_buf);
    size_t ref_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, ref_sym);
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    size_t sig_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);
    size_t eval_n = (ref_eval_n < sig_eval_n) ? ref_eval_n : sig_eval_n;

    StageMetric temp_metric =
        compute_stage_metric_complex(stage.name, "rf_to_bb", ref_sym, sig_sym, eval_n);
    metrics[m].evm_pct = temp_metric.evm_pct;

    write_trace_stage_artifacts(
        csv_dir, trace_dir, "traces", rf_stage_count + i + 2u, 0, stage.name,
        &metrics[m], "BB", bb_ref_re, bb_sig_re, nbb, 600u,
        cfg->symbol_rate_hz * 8.0, 0.0);
    write_constellation_stage_artifacts(
        csv_dir, const_dir, "receiver", rf_stage_count + i + 2u, 0, stage.name,
        &metrics[m], "BB", constellation_template, constellation_count,
        tx_symbols, sig_sym, eval_n);

    ++m;
  }

  /* Step 11: Report results */
  *final_vpp = complex_real_vpp(ref_sym, nsym);
  *metric_count = m;
  *used_sps = sps;
  *used_fs_hz = fs_hz;

  t_bb_stages = omp_get_wtime();

  double total = t_bb_stages - t0;
  fprintf(stderr, "\n=== TIMING (16 threads) ===\n");
  fprintf(stderr, "Pulse shaping + upconv + input metric: %.2fs (%.1f%%)\n", t_pulse - t0, (t_pulse - t0) / total * 100);
  fprintf(stderr, "RF stages (%zu): %.2fs (%.1f%%)\n", (size_t)rf_stage_count, t_rf_stages - t_pulse, (t_rf_stages - t_pulse) / total * 100);
  fprintf(stderr, "Downconversion: %.2fs (%.1f%%)\n", t_downconv - t_rf_stages, (t_downconv - t_rf_stages) / total * 100);
  fprintf(stderr, "BB stages (%zu): %.2fs (%.1f%%)\n", (size_t)bb_stage_count, t_bb_stages - t_downconv, (t_bb_stages - t_downconv) / total * 100);
  fprintf(stderr, "Total: %.2fs\n", total);

  /* Clean up all working buffers — SoA */
  rf_setup_free(&bufs);
  return 0;
}
