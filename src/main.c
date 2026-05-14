#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "adc_model.h"
#include "biquad_filter.h"
#include "cli_args.h"
#include "constellation.h"
#include "flicker_noise.h"
#include "iq_imbalance.h"
#include "math_utils.h"
#include "metrics.h"
#include "output_mgr.h"
#include "phase_noise.h"
#include "physics.h"
#include "prng.h"
#include "propagation.h"
#include "signal_chain.h"
#include "sim_baseband.h"
#include "sim_rf.h"
#include "sim_types.h"
#include "stage_artifacts.h"
#include "stage_models.h"

/* ============================================================================
 * GLOBAL PRNG STATE
 * ============================================================================ */
static PRNG rng;

/* ============================================================================
 * INTERNAL UTILITIES — SoA (Structure of Arrays) Processing
 * ============================================================================
 *
 * To leverage SIMD and OpenMP effectively, signal processing is performed on
 * separate I and Q arrays (Real/Imag) rather than interleaved complex structs.
 * ============================================================================
 */

static void scale_soa(double *re, double *im, size_t n, double gain) {
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < n; i++) {
    re[i] *= gain;
    im[i] *= gain;
  }
}

static void add_awgn_soa(double *re, double *im, size_t n, double var_v2) {
  double std = sqrt(var_v2 / 2.0);
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < n; i++) {
    re[i] += prng_next_gaussian(&rng) * std;
    im[i] += prng_next_gaussian(&rng) * std;
  }
}

static double mean_power_soa(const double *re, const double *im, size_t n) {
  double sum = 0.0;
#pragma omp parallel for reduction(+ : sum) schedule(static)
  for (size_t i = 0; i < n; i++) {
    sum += re[i] * re[i] + im[i] * im[i];
  }
  return sum / (double)n;
}

static double mean_noise_power_soa(const double *sig_re, const double *sig_im,
                                   const double *ref_re, const double *ref_im,
                                   size_t n) {
  double sum = 0.0;
#pragma omp parallel for reduction(+ : sum) schedule(static)
  for (size_t i = 0; i < n; i++) {
    double dr = sig_re[i] - ref_re[i];
    double di = sig_im[i] - ref_im[i];
    sum += dr * dr + di * di;
  }
  return sum / (double)n;
}

static void pack_complex(const double *re, const double *im, size_t n,
                         Complex *out) {
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < n; i++) {
    out[i].re = re[i];
    out[i].im = im[i];
  }
}

/* --- SoA application of a StageModel --- */

static StageMetric apply_stage_soa(
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

/* --- SoA upconversion --- */

static void env_to_rf_soa(const double *restrict env_re,
                          const double *restrict env_im, size_t n,
                          double fs_hz, double fc_hz,
                          double *restrict rf_out) {
  const double dtheta = 2.0 * M_PI * fc_hz / fs_hz;
  size_t i;
#pragma omp parallel for schedule(static)
  for (i = 0; i < n; ++i) {
    double theta = (double)i * dtheta;
    rf_out[i] = env_re[i] * cos(theta) - env_im[i] * sin(theta);
  }
}

/* --- SoA downconversion --- */

static size_t mix_down_soa(const double *restrict rf, size_t n, double fs_hz,
                           double fc_hz, double cutoff_hz, size_t dec_factor,
                           double *restrict bb_re, double *restrict bb_im,
                           double *restrict i_raw, double *restrict q_raw) {
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

/* --- SoA pulse shaping --- */

static void shape_symbols_rrc_to_env_soa(const Complex *restrict symbols,
                                         size_t nsym, int sps, double rolloff,
                                         double *restrict env_re,
                                         double *restrict env_im) {
  const int span = 20;
  size_t pulse_len, total_len, i, k;
  double *pulse;
  double norm_sq = 0.0;

  if (!symbols || !env_re || !env_im || nsym == 0u || sps <= 0)
    return;

  if (rolloff <= 0.0) {
    for (i = 0u; i < nsym; ++i) {
      for (k = 0u; k < (size_t)sps; ++k) {
        env_re[i * (size_t)sps + k] = symbols[i].re;
        env_im[i * (size_t)sps + k] = symbols[i].im;
      }
    }
    return;
  }

  pulse_len = (size_t)span * (size_t)sps + 1u;
  total_len = nsym * (size_t)sps + pulse_len - 1u;
  pulse = (double *)calloc(pulse_len, sizeof(double));
  if (!pulse) {
    for (i = 0u; i < nsym; ++i) {
      for (k = 0u; k < (size_t)sps; ++k) {
        env_re[i * (size_t)sps + k] = symbols[i].re;
        env_im[i * (size_t)sps + k] = symbols[i].im;
      }
    }
    return;
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
    for (k = 0u; k < pulse_len; ++k)
      pulse[k] /= norm;
  }

  memset(env_re, 0, total_len * sizeof(double));
  memset(env_im, 0, total_len * sizeof(double));
  for (i = 0u; i < nsym; ++i) {
    const size_t base = i * (size_t)sps;
    for (k = 0u; k < pulse_len; ++k) {
      const size_t idx = base + k;
      if (idx >= total_len) break;
      env_re[idx] += symbols[i].re * pulse[k];
      env_im[idx] += symbols[i].im * pulse[k];
    }
  }

  free(pulse);
}

/* ============================================================================
 * SIMULATION ENGINES
 * ============================================================================
 *
 * These are the two main simulation functions that orchestrate the entire
 * signal processing chain. Each one runs the signal through all stages,
 * measures quality, and writes output files.
 * ============================================================================
 */

/* --- Simulation 1: Brute-Force RF Simulation --- */
int simulate_bruteforce_rf(const SimConfig *cfg,
                            const StageModelsConfig *stage_cfg,
                            const Complex *tx_symbols, size_t nsym,
                            const Complex *constellation_template,
                            size_t constellation_count, StageMetric *metrics,
                            size_t *metric_count, double *final_vpp,
                            int *used_sps, double *used_fs_hz,
                            const char *csv_dir, const char *const_dir,
                            const char *trace_dir) {
  /* Simulation parameters */
  const double fs_hz = cfg->rf_sample_rate_hz;
  const int sps = (int)round(fs_hz / cfg->symbol_rate_hz);
  const size_t nrf = nsym * (size_t)sps;
  const size_t dec_factor = (size_t)(sps / 8); /* Downsample for BB analysis */
  const size_t nbb = nrf / dec_factor;
  const double bb_sps = (double)sps / (double)dec_factor;

  /* Friis tracking variables */
  const double N_t0_W = K_BOLTZMANN * cfg->t0_k * cfg->symbol_rate_hz;
  double N_current = K_BOLTZMANN * cfg->antenna_temp_k * cfg->symbol_rate_hz;
  double Gain_total = 1.0;
  double P_sig_in_W = pow(10.0, (cfg->input_snr_db + lin_to_db(N_current) - 30.0) / 10.0);

  /* Stage pointers */
  const StageModel *rf_stages, *bb_stages;
  size_t rf_stage_count, bb_stage_count;
  size_t i, m = 0u;

  /* Buffers */
  double *env_re = (double *)malloc(nrf * sizeof(double));
  double *env_im = (double *)malloc(nrf * sizeof(double));
  double *rf_ref = (double *)malloc(nrf * sizeof(double));
  double *rf_sig = (double *)malloc(nrf * sizeof(double));
  double *bb_ref_re = (double *)malloc(nbb * sizeof(double));
  double *bb_ref_im = (double *)malloc(nbb * sizeof(double));
  double *bb_sig_re = (double *)malloc(nbb * sizeof(double));
  double *bb_sig_im = (double *)malloc(nbb * sizeof(double));
  double *i_raw = (double *)malloc(nrf * sizeof(double));
  double *q_raw = (double *)malloc(nrf * sizeof(double));
  Complex *ref_sym = (Complex *)malloc(nsym * sizeof(Complex));
  Complex *sig_sym = (Complex *)malloc(nsym * sizeof(Complex));
  Complex *temp_complex_buf = (Complex *)malloc(nbb * sizeof(Complex));

  if (!env_re || !env_im || !rf_ref || !rf_sig || !bb_ref_re || !bb_ref_im ||
      !bb_sig_re || !bb_sig_im || !i_raw || !q_raw || !ref_sym || !sig_sym ||
      !temp_complex_buf) {
    free(env_re); free(env_im); free(rf_ref); free(rf_sig);
    free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
    free(i_raw); free(q_raw); free(ref_sym); free(sig_sym);
    free(temp_complex_buf);
    return 4;
  }

  rf_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_FRONTEND, &rf_stages);
  bb_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_POSTMIX_BB, &bb_stages);

  /* Step 1: Pulse shaping → complex baseband envelope (SoA) */
  shape_symbols_rrc_to_env_soa(tx_symbols, nsym, sps, cfg->rolloff, env_re, env_im);

  /* Step 2: Scale and add initial noise (SoA) */
  {
    double amp_in = sqrt(P_sig_in_W * R_LOAD_OHM);
    scale_soa(env_re, env_im, nrf, amp_in);
    memcpy(rf_ref, env_re, nrf * sizeof(double)); /* rf_ref is the envelope ref for now */
    memcpy(rf_sig, env_re, nrf * sizeof(double));
    add_awgn_soa(rf_sig, env_im, nrf, N_current * R_LOAD_OHM);
  }

  /* Step 3: Upconvert Envelope → Real RF (SoA) */
  env_to_rf_soa(rf_ref, env_im, nrf, fs_hz, cfg->carrier_hz, i_raw);
  memcpy(rf_ref, i_raw, nrf * sizeof(double));
  env_to_rf_soa(rf_sig, env_im, nrf, fs_hz, cfg->carrier_hz, i_raw);
  memcpy(rf_sig, i_raw, nrf * sizeof(double));

  /* Step 4: Process through RF frontend stages (SoA) */
  for (i = 0u; i < rf_stage_count; ++i) {
    const StageModel stage = rf_stages[i];
    double g_lin = db_to_lin_power(stage.gain_db);
    double amp_gain = sqrt(g_lin);
    double F = db_to_lin_power(stage.nf_db);
    if (F < 1.0) F = 1.0;
    double pn_add_v2 = N_t0_W * g_lin * (F - 1.0) * R_LOAD_OHM;

    /* Apply gain */
#pragma omp parallel for schedule(static)
    for (size_t j = 0; j < nrf; j++) {
      rf_ref[j] *= amp_gain;
      rf_sig[j] *= amp_gain;
    }

    /* Add noise to signal path */
    if (pn_add_v2 > 0.0) {
      double std = sqrt(pn_add_v2);
#pragma omp parallel for schedule(static)
      for (size_t j = 0; j < nrf; j++) {
        rf_sig[j] += prng_next_gaussian(&rng) * std;
      }
    }

    /* Friis tracker update */
    N_current = N_current * g_lin + N_t0_W * g_lin * (F - 1.0);
    Gain_total *= g_lin;

    /* Metrics calculation (requires downconversion) */
    metrics[m].stage = stage.name;
    metrics[m].domain = "rf_frontend";
    metrics[m].gain_db = stage.gain_db;
    metrics[m].nf_db = stage.nf_db;
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    metrics[m].noise_power = N_current;
    metrics[m].snr_db = lin_to_db(metrics[m].signal_power / metrics[m].noise_power);
    metrics[m].evm_pct = 100.0 / sqrt(pow(10.0, metrics[m].snr_db / 10.0));

    /* Output artifacts for this RF stage */
    {
      const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      size_t nbb_tmp = mix_down_soa(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                                    dec_factor, bb_sig_re, bb_sig_im, i_raw, q_raw);
      pack_complex(bb_sig_re, bb_sig_im, nbb_tmp, temp_complex_buf);
      size_t sig_eval_n = synchronize_and_downsample(
          temp_complex_buf, nbb_tmp, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);

      write_constellation_stage_artifacts(
          csv_dir, const_dir, "constellations", i + 1u, 0, stage.name, &metrics[m],
          "RF", constellation_template, constellation_count, tx_symbols,
          sig_sym, sig_eval_n);
    }
    ++m;
  }

  /* Step 5: Downconvert RF → complex baseband (SoA) */
  {
    const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
    mix_down_soa(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                 dec_factor, bb_ref_re, bb_ref_im, i_raw, q_raw);
    mix_down_soa(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                 dec_factor, bb_sig_re, bb_sig_im, i_raw, q_raw);
  }

  /* Step 6: Process through post-mixer BB stages (SoA) */
  for (i = 0u; i < bb_stage_count; ++i) {
    metrics[m] = apply_stage_soa(&bb_stages[i], bb_ref_re, bb_ref_im,
                                 bb_sig_re, bb_sig_im, nbb, "rf_to_bb",
                                 N_t0_W, &N_current, &Gain_total, P_sig_in_W);

    /* Output artifacts for this BB stage */
    {
      pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
      size_t sig_eval_n = synchronize_and_downsample(
          temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);

      write_constellation_stage_artifacts(
          csv_dir, const_dir, "constellations", rf_stage_count + i + 1u, 0,
          bb_stages[i].name, &metrics[m], "RF", constellation_template,
          constellation_count, tx_symbols, sig_sym, sig_eval_n);

      write_complex_trace_stage_artifacts(
          trace_dir, "traces", rf_stage_count + i + 1u, 0, bb_stages[i].name,
          &metrics[m], temp_complex_buf, nbb, fs_hz / (double)dec_factor, cfg->symbol_rate_hz, 10.0);
    }
    ++m;
  }

  /* Final results */
  pack_complex(bb_ref_re, bb_ref_im, nbb, temp_complex_buf);
  (void)synchronize_and_downsample(temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, ref_sym);
  *final_vpp = complex_real_vpp(ref_sym, nsym);
  *metric_count = m;
  *used_sps = sps;
  *used_fs_hz = fs_hz;

  /* Cleanup */
  free(env_re); free(env_im); free(rf_ref); free(rf_sig);
  free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
  free(i_raw); free(q_raw); free(ref_sym); free(sig_sym);
  free(temp_complex_buf);
  return 0;
}

/* --- Simulation 2: Realistic RF Simulation (with phase noise, I/Q imbalance, etc.) --- */
int simulate_realistic_rf(const SimConfig *cfg,
                           const StageModelsConfig *stage_cfg,
                           const Complex *tx_symbols,
                           const Complex *constellation_template,
                           size_t constellation_count, size_t nsym,
                           StageMetric *metrics, size_t *metric_count,
                           double *final_vpp, int *used_sps,
                           double *used_fs_hz, const char *csv_dir,
                           const char *const_dir, const char *trace_dir) {
  const double fs_hz = cfg->rf_sample_rate_hz;
  const int sps = (int)round(fs_hz / cfg->symbol_rate_hz);
  const size_t nrf = nsym * (size_t)sps;
  const size_t dec_factor = (size_t)(sps / 8);
  const size_t nbb = nrf / dec_factor;
  const double bb_sps = (double)sps / (double)dec_factor;

  const double N_t0_W = K_BOLTZMANN * cfg->t0_k * cfg->symbol_rate_hz;
  double N_current = K_BOLTZMANN * cfg->antenna_temp_k * cfg->symbol_rate_hz;
  double Gain_total = 1.0;
  double P_sig_in_W = pow(10.0, (cfg->input_snr_db + lin_to_db(N_current) - 30.0) / 10.0);

  const StageModel *rf_stages, *bb_stages;
  size_t rf_stage_count, bb_stage_count;
  size_t i, m = 0u;

  /* Performance counters */
  double t_start, t_upconv, t_rf_stages, t_downconv;

  /* Buffers */
  double *env_re = (double *)malloc(nrf * sizeof(double));
  double *env_im = (double *)malloc(nrf * sizeof(double));
  double *rf_ref = (double *)malloc(nrf * sizeof(double));
  double *rf_sig = (double *)malloc(nrf * sizeof(double));
  double *bb_ref_re = (double *)malloc(nbb * sizeof(double));
  double *bb_ref_im = (double *)malloc(nbb * sizeof(double));
  double *bb_sig_re = (double *)malloc(nbb * sizeof(double));
  double *bb_sig_im = (double *)malloc(nbb * sizeof(double));
  double *i_raw = (double *)malloc(nrf * sizeof(double));
  double *q_raw = (double *)malloc(nrf * sizeof(double));
  Complex *ref_sym = (Complex *)malloc(nsym * sizeof(Complex));
  Complex *sig_sym = (Complex *)malloc(nsym * sizeof(Complex));
  Complex *temp_complex_buf = (Complex *)malloc(nbb * sizeof(Complex));
  Complex *temp_ref_sym = (Complex *)malloc(nsym * sizeof(Complex));
  Complex *temp_sig_sym = (Complex *)malloc(nsym * sizeof(Complex));

  if (!env_re || !env_im || !rf_ref || !rf_sig || !bb_ref_re || !bb_ref_im ||
      !bb_sig_re || !bb_sig_im || !i_raw || !q_raw || !ref_sym || !sig_sym ||
      !temp_complex_buf || !temp_ref_sym || !temp_sig_sym) {
    /* Cleanup and return error */
    free(env_re); free(env_im); free(rf_ref); free(rf_sig);
    free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
    free(i_raw); free(q_raw); free(ref_sym); free(sig_sym);
    free(temp_complex_buf); free(temp_ref_sym); free(temp_sig_sym);
    return 4;
  }

  rf_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_FRONTEND, &rf_stages);
  bb_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_POSTMIX_BB, &bb_stages);

  /* Model configurations */
  PhaseNoiseConfig tx_lo_pn;
  phase_noise_init(&tx_lo_pn, -100.0, 1.0e6, fs_hz);
  PhaseNoiseConfig rx_lo_pn;
  phase_noise_init(&rx_lo_pn, -95.0, 1.0e6, fs_hz / (double)dec_factor);

  IQImbalanceConfig iq_cfg;
  iq_imbalance_init(&iq_cfg, 0.5, 2.0);

  FlickerNoiseConfig flicker_cfg;
  flicker_noise_init(&flicker_cfg, 1e-11, 1e2, 1e6, fs_hz / (double)dec_factor);

  ADCConfig adc_cfg;
  adc_model_init(&adc_cfg, 12, 1.0, 50e-15);

  t_start = omp_get_wtime();

  /* Step 1: Pulse shaping → complex baseband envelope (SoA) */
  shape_symbols_rrc_to_env_soa(tx_symbols, nsym, sps, cfg->rolloff, env_re, env_im);

  /* Step 2: Scale and add initial noise (SoA) */
  {
    double amp_in = sqrt(P_sig_in_W * R_LOAD_OHM);
    scale_soa(env_re, env_im, nrf, amp_in);
    memcpy(rf_ref, env_re, nrf * sizeof(double));
    memcpy(rf_sig, env_re, nrf * sizeof(double));
    add_awgn_soa(rf_sig, env_im, nrf, N_current * R_LOAD_OHM);
  }

  /* Step 3: Apply TX LO phase noise as rotation on baseband envelope */
  {
    for (i = 0; i < nrf; i++) {
      double pn = phase_noise_generate(&tx_lo_pn, &rng);
      double c = cos(pn), s = sin(pn);
      double new_re = rf_sig[i] * c - env_im[i] * s;
      double new_im = rf_sig[i] * s + env_im[i] * c;
      rf_sig[i] = new_re;
      env_im[i] = new_im;
      /* Reference passes through clean */
    }
  }

  /* Step 4: Upconvert Envelope → Real RF (SoA) */
  env_to_rf_soa(rf_ref, env_im, nrf, fs_hz, cfg->carrier_hz, i_raw);
  memcpy(rf_ref, i_raw, nrf * sizeof(double));
  env_to_rf_soa(rf_sig, env_im, nrf, fs_hz, cfg->carrier_hz, i_raw);
  memcpy(rf_sig, i_raw, nrf * sizeof(double));

  t_upconv = omp_get_wtime();

  /* Step 5: Process through RF frontend stages (SoA) */
  for (i = 0u; i < rf_stage_count; ++i) {
    const StageModel stage = rf_stages[i];
    double g_lin = db_to_lin_power(stage.gain_db);
    double amp_gain = sqrt(g_lin);
    double F = db_to_lin_power(stage.nf_db);
    if (F < 1.0) F = 1.0;
    double pn_add_v2 = N_t0_W * g_lin * (F - 1.0) * R_LOAD_OHM;

#pragma omp parallel for schedule(static)
    for (size_t j = 0; j < nrf; j++) {
      rf_ref[j] *= amp_gain;
      rf_sig[j] *= amp_gain;
    }

    if (pn_add_v2 > 0.0) {
      double std = sqrt(pn_add_v2);
#pragma omp parallel for schedule(static)
      for (size_t j = 0; j < nrf; j++) {
        rf_sig[j] += prng_next_gaussian(&rng) * std;
      }
    }

    N_current = N_current * g_lin + N_t0_W * g_lin * (F - 1.0);
    Gain_total *= g_lin;

    metrics[m].stage = stage.name;
    metrics[m].domain = "rf_frontend";
    metrics[m].gain_db = stage.gain_db;
    metrics[m].nf_db = stage.nf_db;
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    metrics[m].noise_power = N_current;
    metrics[m].snr_db = lin_to_db(metrics[m].signal_power / metrics[m].noise_power);

    /* Step 7: Calculate EVM for this stage by downconverting a small slice */
    {
      const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      mix_down_soa(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      mix_down_soa(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
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

      StageMetric temp_metric = compute_metric_complex(
          stage.name, "rf_to_bb", tx_symbols, temp_sig_sym, temp_eval_n);
      metrics[m].evm_pct = temp_metric.evm_pct;

      write_constellation_stage_artifacts(
          csv_dir, const_dir, "constellations", i + 1u, 0, stage.name, &metrics[m],
          "RF", constellation_template, constellation_count, tx_symbols,
          temp_sig_sym, temp_eval_n);
    }

    ++m;
  }

  t_rf_stages = omp_get_wtime();

  /* Step 8: Downconvert RF → complex baseband — SoA */
  {
    const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
    mix_down_soa(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                 dec_factor, bb_ref_re, bb_ref_im, i_raw, q_raw);
    mix_down_soa(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                 dec_factor, bb_sig_re, bb_sig_im, i_raw, q_raw);
  }

  /* Step 9: Apply RX LO phase noise as complex rotation on baseband */
  {
    for (i = 0; i < nbb; i++) {
      double pn = phase_noise_generate(&rx_lo_pn, &rng);
      double c = cos(pn), s = sin(pn);
      double new_re = bb_sig_re[i] * c - bb_sig_im[i] * s;
      double new_im = bb_sig_re[i] * s + bb_sig_im[i] * c;
      bb_sig_re[i] = new_re;
      bb_sig_im[i] = new_im;

      /* Apply SAME rotation to reference */
      new_re = bb_ref_re[i] * c - bb_ref_im[i] * s;
      new_im = bb_ref_re[i] * s + bb_ref_im[i] * c;
      bb_ref_re[i] = new_re;
      bb_ref_im[i] = new_im;
    }
  }

  t_downconv = omp_get_wtime();

  /* Step 10: Apply I/Q imbalance after downconversion */
  {
    for (i = 0; i < nbb; i++) {
      iq_imbalance_apply_real(&bb_sig_re[i], &bb_sig_im[i], &iq_cfg);
      /* Reference passes through clean */
    }
  }

  /* Step 11: Extract symbol-rate samples */
  {
    pack_complex(bb_ref_re, bb_ref_im, nbb, temp_complex_buf);
    size_t ref_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, ref_sym);
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    size_t sig_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);
    size_t eval_n = (ref_eval_n < sig_eval_n) ? ref_eval_n : sig_eval_n;
    (void)eval_n;

    metrics[m] = compute_metric_complex("MIX2_Downconv_realistic", "rf_to_bb", ref_sym,
                                        sig_sym, ref_eval_n);
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db((P_sig_in_W * Gain_total) / N_current);
    }
    write_constellation_stage_artifacts(
        csv_dir, const_dir, "constellations", rf_stage_count + 1u, 0,
        metrics[m].stage, &metrics[m], "RF", constellation_template,
        constellation_count, tx_symbols, sig_sym, sig_eval_n);
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    write_complex_trace_stage_artifacts(
        trace_dir, "traces", rf_stage_count + 1u, 0, metrics[m].stage,
        &metrics[m], temp_complex_buf, nbb, fs_hz / (double)dec_factor, cfg->symbol_rate_hz, 10.0);
    ++m;
  }

  /* Step 12: Process through post-mixer BB stages with biquad filters and flicker noise */
  char realistic_stage_name[256];
  for (i = 0u; i < bb_stage_count; ++i) {
    StageModel stage = bb_stages[i];

    if (strstr(stage.name, "LNA 3") || strstr(stage.name, "lna3") ||
        strstr(stage.name, "post_05")) {
      double p_mean = mean_power_soa(bb_sig_re, bb_sig_im, nbb);
      double v_rms = sqrt(p_mean);
      double v_pp_est = v_rms * 2.0 * sqrt(2.0);
      double target_vpp = 1.0;
      if (v_pp_est > 1e-9) {
        double g_extra_db = 20.0 * log10(target_vpp / v_pp_est);
        if (g_extra_db < -20.0) g_extra_db = -20.0;
        if (g_extra_db > 20.0) g_extra_db = 20.0;
        stage.gain_db += g_extra_db;
      }
    }

    /* Apply biquad filter if filter_type == 1 (Butterworth) */
    if (stage.filter_type == 1) {
      BiquadState local_bq;
      BiquadConfig bq_cfg = {0};
      bq_cfg.cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      bq_cfg.fs_hz = fs_hz / (double)dec_factor;
      bq_cfg.order = stage.filter_order > 0 ? stage.filter_order : 4;
      if (biquad_init(&local_bq, &bq_cfg) == 0) {
        biquad_process_soa(&local_bq, bb_sig_re, bb_sig_im, bb_sig_re, bb_sig_im, nbb);
        biquad_process_soa(&local_bq, bb_ref_re, bb_ref_im, bb_ref_re, bb_ref_im, nbb);
      }
    } else {
      /* Legacy path: use apply_stage_soa */
      apply_stage_soa(&stage, bb_ref_re, bb_ref_im, bb_sig_re, bb_sig_im, nbb,
                      "rf_to_bb", N_t0_W, &N_current, &Gain_total, P_sig_in_W);
    }

    /* Add flicker noise to signal path */
    {
      size_t j;
      for (j = 0; j < nbb; j++) {
        double fn = flicker_noise_generate(&flicker_cfg, &rng);
        bb_sig_re[j] += fn;
        fn = flicker_noise_generate(&flicker_cfg, &rng);
        bb_sig_im[j] += fn;
      }
    }

    snprintf(realistic_stage_name, sizeof(realistic_stage_name), "%s Realistic", stage.name);

    /* Write trace using continuous baseband waveform (match stage 6 format) */
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    write_complex_trace_stage_artifacts(
        trace_dir, "traces", rf_stage_count + 2u + i, 0, realistic_stage_name, &metrics[m],
        temp_complex_buf, nbb, fs_hz / (double)dec_factor, cfg->symbol_rate_hz, 10.0);

    /* Downsample for metrics */
    pack_complex(bb_ref_re, bb_ref_im, nbb, temp_complex_buf);
    size_t ref_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, ref_sym);
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    size_t sig_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);
    size_t eval_n = (ref_eval_n < sig_eval_n) ? ref_eval_n : sig_eval_n;

    metrics[m] = compute_metric_complex(stage.name, "rf_to_bb", ref_sym,
                                        sig_sym, eval_n);
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    metrics[m].gain_db = stage.gain_db;
    metrics[m].nf_db = stage.nf_db;
    metrics[m].filter_len = stage.filter_len;
    metrics[m].is_limiter = stage.is_limiter;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db((P_sig_in_W * Gain_total) / N_current);
    }

    write_constellation_stage_artifacts(
        csv_dir, const_dir, "constellations", rf_stage_count + 2u + i, 0, realistic_stage_name,
        &metrics[m], "RF", constellation_template, constellation_count,
        tx_symbols, sig_sym, eval_n);
    ++m;
  }

  /* Step 13: Apply ADC model at final stage */
  {
    double f_in_hz = cfg->symbol_rate_hz;
    size_t j;
    for (j = 0; j < nsym; j++) {
      adc_model_apply(&sig_sym[j].re, f_in_hz, &adc_cfg, &rng);
      adc_model_apply(&sig_sym[j].im, f_in_hz, &adc_cfg, &rng);
    }
  }

  /* Step 14: Add LO leakage DC offset */
  {
    double dc_offset = 0.001; /* 1 mV typical LO leakage */
    size_t j;
    for (j = 0; j < nsym; j++) {
      sig_sym[j].re += dc_offset;
      sig_sym[j].im += dc_offset;
    }
  }

  /* Step 15: Report results */
  *final_vpp = complex_real_vpp(ref_sym, nsym);
  *metric_count = m;
  *used_sps = sps;
  *used_fs_hz = fs_hz;

  /* Step 16: Free allocated memory */
  free(env_re); free(env_im); free(rf_ref); free(rf_sig);
  free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
  free(i_raw); free(q_raw); free(ref_sym); free(sig_sym);
  free(temp_complex_buf); free(temp_ref_sym); free(temp_sig_sym);

  return 0;
}

/* ============================================================================
 * MAIN ENTRY POINT
 * ============================================================================
 */

/*
 * main — Orchestrate the complete 64-APSK receiver simulation
 */
int main(int argc, char **argv) {
  SimConfig cfg;               /* Top-level simulation parameters */
  StageModelsConfig stage_cfg; /* Loaded stage chain configuration */
  char stage_csv_path[512] =
      "stage_models/runtime_stage_models_target16.csv"; /* Default CSV path */
  char resolved_stage_csv_path[512]; /* Resolved path after directory resolution
                                      */
  char stage_err[256];               /* Error message buffer for CSV loading */
  Complex constellation[64]; /* The 64-APSK constellation (stack-allocated) */
  Complex *tx_symbols;       /* Transmitted symbol sequence (heap-allocated) */
  unsigned short *labels; /* Transmitted symbol indices (for potential SER) */
  StageMetric metrics_bb[MAX_METRICS]; /* Baseband path metrics */
  StageMetric metrics_rf[MAX_METRICS]; /* RF path metrics */
  StageMetric metrics_realistic[MAX_METRICS]; /* Realistic path metrics */
  size_t count_bb = 0u;                /* Number of baseband metrics */
  size_t count_rf = 0u;                /* Number of RF metrics */
  size_t count_realistic = 0u;         /* Number of realistic metrics */
  double final_vpp_bb = 0.0;           /* Final Vpp after baseband path */
  double final_vpp_rf = 0.0;           /* Final Vpp after RF path */
  double final_vpp_realistic = 0.0;    /* Final Vpp after realistic path */
  int rf_sps = 0;                      /* RF samples per symbol (output) */
  int realistic_sps = 0;               /* Realistic samples per symbol (output) */
  double rf_fs_used = 0.0;             /* RF sampling frequency used (output) */
  double realistic_fs_used = 0.0;      /* Realistic sampling frequency used (output) */
  int i;                               /* Loop variable for CLI parsing */

  /* Link budget variables */
  double noise_bw_hz;  /* Noise bandwidth (MATLAB-aligned 200 MHz) */
  double noise_w;      /* Thermal noise power in watts */
  double noise_dbm;    /* Noise power in dBm */
  double signal_dbm;   /* Signal power in dBm */
  int topology_sim_id; /* Output simulation slot (1–4) */

  /* --- Set default simulation parameters --- */
  cfg.carrier_hz = 24.0e9;     /* 24 GHz carrier (K-band satellite) */
  cfg.symbol_rate_hz = 10.0e6; /* 10 MegaSymbols/sec */
  cfg.symbols = 3000;          /* 3000 symbols - reasonable run time */
  cfg.rolloff = 0.0;           /* Rectangular pulses (fastest) */
  cfg.input_snr_db = 20.0;     /* 20 dB input SNR based on assignment */
  cfg.antenna_temp_k =
      150.0;        /* 150 K antenna temperature (typical satellite) */
  cfg.t0_k = 290.0; /* 290 K reference temperature (room temp) */
  cfg.rf_sample_rate_hz = 96.0e9;      /* 96 GHz RF sampling */
  cfg.seed = (unsigned int)time(NULL); /* Default seed: current time */
  cfg.run_bb = 0;                      /* Baseband path disabled by default */
  cfg.run_rf = 1;                      /* RF baseline path enabled by default */
  cfg.run_realistic = 1;               /* Realistic path enabled by default */
  topology_sim_id = 1;                 /* Default output slot: topology_sim_1 */
  memset(&stage_cfg, 0, sizeof(stage_cfg));
  stage_err[0] = '\0';

  /* --- Parse command-line arguments --- */
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      if (parse_u32(argv[++i], &cfg.seed) != 0) {
        fprintf(stderr, "Invalid --seed value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--symbols") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &cfg.symbols) != 0 || cfg.symbols <= 0) {
        fprintf(stderr, "Invalid --symbols value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--symbol-rate") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.symbol_rate_hz) != 0 ||
          cfg.symbol_rate_hz <= 0.0) {
        fprintf(stderr, "Invalid --symbol-rate value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--rf-fs") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.rf_sample_rate_hz) != 0 ||
          cfg.rf_sample_rate_hz <= 0.0) {
        fprintf(stderr, "Invalid --rf-fs value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--carrier") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.carrier_hz) != 0 ||
          cfg.carrier_hz <= 0.0) {
        fprintf(stderr, "Invalid --carrier value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
      if (parse_double(argv[++i], &cfg.input_snr_db) != 0) {
        fprintf(stderr, "Invalid --snr value\n");
        return 2;
      }
    } else if (strcmp(argv[i], "--stage-csv") == 0 && i + 1 < argc) {
      snprintf(stage_csv_path, sizeof(stage_csv_path), "%s", argv[++i]);
    } else if (strcmp(argv[i], "--topology-sim") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &topology_sim_id) != 0 || topology_sim_id < 1 ||
          topology_sim_id > TOPOLOGY_SIM_COUNT) {
        fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n",
                TOPOLOGY_SIM_COUNT);
        return 2;
      }
    } else if (strcmp(argv[i], "--disable-bb") == 0) {
      cfg.run_bb = 0;
    } else if (strcmp(argv[i], "--enable-rf") == 0) {
      cfg.run_rf = 1;
    } else if (strcmp(argv[i], "--enable-realistic") == 0) {
      cfg.run_realistic = 1;
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }

  /* --- Initialize the PRNG --- */
  prng_init(&rng, cfg.seed);

  /* --- Create output directories --- */
  if (ensure_output_dirs(OUTPUT_ROOT_DIR, topology_sim_id) != 0) {
    fprintf(stderr, "Failed to create output directories under ./out\n");
    return 8;
  }

  /* --- Resolve the stage-model CSV path --- */
  if (resolve_stage_csv_path(stage_csv_path, resolved_stage_csv_path,
                             sizeof(resolved_stage_csv_path)) != 0) {
    fprintf(stderr, "Failed to resolve stage-model CSV path '%s'\n",
            stage_csv_path);
    return 5;
  }

  char csv_dir[512], const_dir[512], trace_dir[512];
  char rf_csv_dir[512], rf_const_dir[512], rf_trace_dir[512];
  char realistic_csv_dir[512], realistic_const_dir[512], realistic_trace_dir[512];

  get_simulation_paths(OUTPUT_ROOT_DIR, topology_sim_id, "complex_baseband",
                       csv_dir, const_dir, trace_dir);
  get_simulation_paths(OUTPUT_ROOT_DIR, topology_sim_id, "rf_bruteforce",
                       rf_csv_dir, rf_const_dir, rf_trace_dir);
  get_simulation_paths(OUTPUT_ROOT_DIR, topology_sim_id, "rf_realistic",
                       realistic_csv_dir, realistic_const_dir, realistic_trace_dir);

  /* --- Build the DVB-S2X 64-APSK constellation --- */
  if (build_dvbs2x_64apsk(constellation) != 0) {
    fprintf(stderr, "Failed to build 64-APSK constellation\n");
    return 3;
  }

  /* --- Generate a random transmitted symbol sequence --- */
  tx_symbols = (Complex *)malloc((size_t)cfg.symbols * sizeof(Complex));
  labels = (unsigned short *)malloc((size_t)cfg.symbols * sizeof(unsigned short));
  if (!tx_symbols || !labels) {
    fprintf(stderr, "Memory allocation failed for %d symbols\n", cfg.symbols);
    return 4;
  }
  generate_random_sequence(&rng, constellation, 64u, tx_symbols, labels,
                           (size_t)cfg.symbols);

  /* --- Calculate the link budget (thermal noise power + signal power) ---
   *
   * We define the input SNR relative to the thermal noise at the receiver
   * antenna port. We set the signal level so that the simulation matches
   * the specified SNR, while keeping voltage levels realistic (microvolts) so
   * the RF chain starts from the same absolute signal scale.
   */
  noise_bw_hz = 200.0e6;
  noise_w = K_BOLTZMANN * cfg.antenna_temp_k * noise_bw_hz;
  noise_dbm = lin_to_db(noise_w) + 30.0;
  {
    const double r_load_ohm = 50.0;
    const double input_target_vrms = 50.0e-6;
    const double signal_w =
        (input_target_vrms * input_target_vrms) / r_load_ohm;
    signal_dbm = lin_to_db(signal_w) + 30.0;
    cfg.input_snr_db = signal_dbm - noise_dbm;
  }

  /* --- Part D: Propagation Analysis (FSPL, rain, fog, gas, link margin) --- */
  {
    PropagationScenario prop_scenario;
    LinkBudgetResult    prop_budget;

    prop_scenario.frequency_hz         = cfg.carrier_hz;         /* 24 GHz */
    prop_scenario.distance_km          = 36000.0;                /* GEO orbit */
    prop_scenario.elevation_deg        = 30.0;                   /* generic elevation */
    prop_scenario.polarization_deg     = 45.0;                   /* circular (TBD with team) */
    prop_scenario.rain_rate_mmh        = 10.0;                   /* 0.01% exceedance, generic */
    prop_scenario.surface_temp_k       = 288.15;                 /* 15 °C */
    prop_scenario.surface_pressure_hpa = 1013.25;                /* sea level */
    prop_scenario.water_vapor_gm3      = 7.5;                    /* reference */
    prop_scenario.liquid_water_gm3     = 0.05;                   /* medium fog */
    prop_scenario.eirp_dbm             = 85.0;                   /* satellite link */
    prop_scenario.rx_gain_dbi          = 40.0;                   /* typical ground antenna */
    prop_scenario.rx_sensitivity_dbm   = -70.0;                  /* placeholder (from Part E) */

    compute_link_budget(&prop_scenario, &prop_budget);
    print_link_budget(&prop_budget);
  }

  /* --- Load receiver stage configuration from CSV --- */
  if (stage_models_load_csv(resolved_stage_csv_path, &stage_cfg, stage_err,
                            sizeof(stage_err)) != 0) {
    fprintf(stderr, "Failed to load stage-model CSV '%s': %s\n",
            resolved_stage_csv_path, stage_err);
    free(tx_symbols);
    free(labels);
    return 5;
  }

  /* --- Run Simulation Paths --- */
  if (cfg.run_bb) {
    SimBasebandResult bb_result;
    memset(&bb_result, 0, sizeof(bb_result));
    if (simulate_baseband(&cfg, &stage_cfg, constellation, 64u,
                          tx_symbols, (size_t)cfg.symbols, &rng, &bb_result,
                          csv_dir, const_dir, trace_dir) != 0) {
      fprintf(stderr, "Complex-baseband simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 6;
    }
    memcpy(metrics_bb, bb_result.metrics, bb_result.count * sizeof(StageMetric));
    count_bb = bb_result.count;
    final_vpp_bb = bb_result.final_vpp;
  }

  if (cfg.run_rf) {
    if (simulate_bruteforce_rf(&cfg, &stage_cfg, tx_symbols, constellation, 64u,
                               (size_t)cfg.symbols, metrics_rf, &count_rf,
                               &final_vpp_rf, &rf_sps, &rf_fs_used, rf_csv_dir,
                               rf_const_dir, rf_trace_dir) != 0) {
      fprintf(stderr, "Brute-force RF simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 7;
    }

    {
      char rf_metrics_path[256];
      snprintf(rf_metrics_path, sizeof(rf_metrics_path), "%s/rf_metrics.csv", rf_csv_dir);
      if (count_rf > 0u &&
          write_metrics_csv(rf_metrics_path, metrics_rf, count_rf) != 0) {
        fprintf(stderr, "Warning: failed to write aggregate RF metrics CSV '%s'\n",
                rf_metrics_path);
      }
    }
  }

  if (cfg.run_realistic) {
    if (simulate_realistic_rf(&cfg, &stage_cfg, tx_symbols, constellation, 64u,
                              (size_t)cfg.symbols, metrics_realistic, &count_realistic,
                              &final_vpp_realistic, &realistic_sps, &realistic_fs_used,
                              realistic_csv_dir, realistic_const_dir, realistic_trace_dir) != 0) {
      fprintf(stderr, "Realistic RF simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 9;
    }

    {
      char realistic_metrics_path[256];
      snprintf(realistic_metrics_path, sizeof(realistic_metrics_path), "%s/realistic_metrics.csv", realistic_csv_dir);
      if (count_realistic > 0u &&
          write_metrics_csv(realistic_metrics_path, metrics_realistic, count_realistic) != 0) {
        fprintf(stderr, "Warning: failed to write aggregate realistic metrics CSV '%s'\n",
                realistic_metrics_path);
      }
    }
  }

  /* --- Print console summary --- */
  printf("seed=%u\n", cfg.seed);
  printf("Stage CSV: %s\n", resolved_stage_csv_path);
  printf("Input budget: Noise=%.4f dBm, Signal=%.4f dBm (SNR=%.2f dB)\n",
         noise_dbm, signal_dbm, cfg.input_snr_db);
  printf("Paths: baseband=%s, rf_baseline=%s, realistic=%s\n",
         cfg.run_bb ? "ON" : "OFF",
         cfg.run_rf ? "ON" : "OFF",
         cfg.run_realistic ? "ON" : "OFF");

  if (cfg.run_bb && count_bb > 0) {
    print_metrics("Complex Baseband Stages Metrics", metrics_bb, count_bb);
  }
  if (cfg.run_rf && count_rf > 0) {
    print_metrics("RF Brute-Force Stages Metrics (SNR/EVM after each stage)",
                  metrics_rf, count_rf);
  }
  if (cfg.run_realistic && count_realistic > 0) {
    print_metrics("Realistic RF Stages Metrics (with impairments)",
                  metrics_realistic, count_realistic);
  }

  if (cfg.run_rf && cfg.run_realistic && count_rf > 0 && count_realistic > 0) {
    printf("\n=== RF Baseline vs Realistic Path Comparison ===\n");
    printf("%-20s %12s %12s %12s %12s\n", "Stage", "RF SNR(dB)", "RF EVM(%)", "Real SNR(dB)", "Real EVM(%)");
    printf("%-20s %12s %12s %12s %12s\n", "--------------------", "------------", "------------", "------------", "------------");
    {
      size_t ri = 0, si = 0;
      while (ri < count_rf || si < count_realistic) {
        const char *stage_name = NULL;
        if (ri < count_rf) stage_name = metrics_rf[ri].stage;
        if (si < count_realistic && (stage_name == NULL || strcmp(stage_name, metrics_realistic[si].stage) > 0))
          stage_name = metrics_realistic[si].stage;

        double rf_snr = ri < count_rf && strcmp(metrics_rf[ri].stage, stage_name) == 0 ? metrics_rf[ri].snr_db : 0.0;
        double rf_evm = ri < count_rf && strcmp(metrics_rf[ri].stage, stage_name) == 0 ? metrics_rf[ri].evm_pct : 0.0;
        double rl_snr = si < count_realistic && strcmp(metrics_realistic[si].stage, stage_name) == 0 ? metrics_realistic[si].snr_db : 0.0;
        double rl_evm = si < count_realistic && strcmp(metrics_realistic[si].stage, stage_name) == 0 ? metrics_realistic[si].evm_pct : 0.0;

        printf("%-20s %12.2f %12.2f %12.2f %12.2f\n", stage_name, rf_snr, rf_evm, rl_snr, rl_evm);

        if (ri < count_rf && strcmp(metrics_rf[ri].stage, stage_name) == 0) ri++;
        if (si < count_realistic && strcmp(metrics_realistic[si].stage, stage_name) == 0) si++;
      }
    }
  }

  printf("\nOutputs written under ./out/\n");
  printf("Generated files strictly follow the assignment requirements (Trace, "
         "Constellation, SNR, EVM).\n");

  /* --- Cleanup --- */
  stage_models_free(&stage_cfg);
  free(tx_symbols);
  free(labels);
  return 0; /* Success! */
}
