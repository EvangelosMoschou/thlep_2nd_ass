#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rf_setup.h"
#include "math_utils.h"
#include "physics.h"

#define ALIGNMENT 64u

static void *alloc_aligned(size_t elem_size, size_t count) {
  size_t raw = elem_size * count;
  size_t aligned = (raw + ALIGNMENT - 1u) & ~(ALIGNMENT - 1u);
  void *p = aligned_alloc(ALIGNMENT, aligned);
  if (p) memset(p, 0, raw);
  return p;
}

#define ALLOC_ALIGNED_D(count) ((double *)alloc_aligned(sizeof(double), (count)))

double rrc_tap_value(double t, double rolloff) {
  if (fabs(t) < 1e-12) {
    return 1.0 - rolloff + (4.0 * rolloff / M_PI);
  }
  if (fabs(fabs(t) - (1.0 / (4.0 * rolloff))) < 1e-12) {
    return (rolloff / sqrt(2.0)) *
           ((1.0 + 2.0 / M_PI) * sin(M_PI / (4.0 * rolloff)) +
            (1.0 - 2.0 / M_PI) * cos(M_PI / (4.0 * rolloff)));
  }
  {
    const double num = sin(M_PI * t * (1.0 - rolloff)) +
                       4.0 * rolloff * t * cos(M_PI * t * (1.0 + rolloff));
    const double den =
        M_PI * t * (1.0 - (4.0 * rolloff * t) * (4.0 * rolloff * t));
    return num / den;
  }
}

void rrc_build_pulse(double rolloff, int sps, int span, double *pulse_out) {
  size_t pulse_len = (size_t)span * (size_t)sps + 1u;
  double norm_sq = 0.0;
  for (size_t k = 0u; k < pulse_len; ++k) {
    const double t = ((double)k - ((double)pulse_len - 1.0) / 2.0) / (double)sps;
    double val = rrc_tap_value(t, rolloff);
    pulse_out[k] = val;
    norm_sq += val * val;
  }
  if (norm_sq > 0.0) {
    const double norm = sqrt(norm_sq);
    for (size_t k = 0u; k < pulse_len; ++k) {
      pulse_out[k] /= norm;
    }
  }
}

void rrc_apply_pulse(const Complex *symbols, size_t nsym, const double *pulse, int pulse_len, int sps, double *env_re, double *env_im) {
  size_t total_len = nsym * (size_t)sps + (size_t)pulse_len - 1u;
  memset(env_re, 0, total_len * sizeof(double));
  memset(env_im, 0, total_len * sizeof(double));
  /* Serial pulse shaping loop - overlapping writes prevent simple parallelization */
  for (size_t i = 0u; i < nsym; ++i) {
    const size_t base = i * (size_t)sps;
    for (int k = 0; k < pulse_len; ++k) {
      const size_t idx = base + k;
      if (idx >= total_len) break;
      env_re[idx] += symbols[i].re * pulse[k];
      env_im[idx] += symbols[i].im * pulse[k];
    }
  }
}

int rf_setup_compute_params(RfSimParams *params, const SimConfig *cfg, const StageModelsConfig *stage_cfg, size_t nsym) {
  params->rf_stages = NULL;
  params->bb_stages = NULL;
  params->rf_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_FRONTEND, &params->rf_stages);
  params->bb_stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_RF_POSTMIX_BB, &params->bb_stages);

  params->sps = (int)llround(cfg->rf_sample_rate_hz / cfg->symbol_rate_hz);
  if (params->sps < 16) {
    params->sps = 16;
  }

  params->bb_sps = 8;
  params->dec_factor = params->sps / params->bb_sps;
  if (params->dec_factor < 1) {
    params->dec_factor = 1;
  }
  params->pulse_len = (cfg->rolloff > 0.0) ? ((size_t)6 * (size_t)params->sps + 1u) : 1u;

  params->fs_hz = cfg->symbol_rate_hz * (double)params->sps;
  params->nrf = nsym * (size_t)params->sps + params->pulse_len - 1u;
  params->nbb = (params->nrf + params->dec_factor - 1u) / params->dec_factor;

  return 0;
}

int rf_setup_allocate(RfSimBuffers *bufs, const SimConfig *cfg, const StageModelsConfig *stage_cfg, size_t nsym) {
  RfSimParams params;
  rf_setup_compute_params(&params, cfg, stage_cfg, nsym);

  memset(bufs, 0, sizeof(RfSimBuffers));

  bufs->env_re = ALLOC_ALIGNED_D(params.nrf);
  bufs->env_im = ALLOC_ALIGNED_D(params.nrf);
  bufs->rf_ref = ALLOC_ALIGNED_D(params.nrf);
  bufs->rf_sig = ALLOC_ALIGNED_D(params.nrf);
  bufs->bb_ref_re = ALLOC_ALIGNED_D(params.nrf);
  bufs->bb_ref_im = ALLOC_ALIGNED_D(params.nrf);
  bufs->bb_sig_re = ALLOC_ALIGNED_D(params.nrf);
  bufs->bb_sig_im = ALLOC_ALIGNED_D(params.nrf);
  bufs->ref_sym = (Complex *)calloc(nsym, sizeof(Complex));
  bufs->sig_sym = (Complex *)calloc(nsym, sizeof(Complex));

  if (!bufs->env_re || !bufs->env_im || !bufs->rf_ref || !bufs->rf_sig ||
      !bufs->bb_ref_re || !bufs->bb_ref_im || !bufs->bb_sig_re || !bufs->bb_sig_im ||
      !bufs->ref_sym || !bufs->sig_sym) {
    rf_setup_free(bufs);
    return -1;
  }

  bufs->temp_bb_ref_re = ALLOC_ALIGNED_D(params.nbb);
  bufs->temp_bb_ref_im = ALLOC_ALIGNED_D(params.nbb);
  bufs->temp_bb_sig_re = ALLOC_ALIGNED_D(params.nbb);
  bufs->temp_bb_sig_im = ALLOC_ALIGNED_D(params.nbb);
  bufs->temp_ref_sym = (Complex *)calloc(nsym, sizeof(Complex));
  bufs->temp_sig_sym = (Complex *)calloc(nsym, sizeof(Complex));
  bufs->temp_complex_buf = (Complex *)calloc(params.nrf > params.nbb ? params.nrf : params.nbb, sizeof(Complex));
  bufs->i_raw = ALLOC_ALIGNED_D(params.nrf);
  bufs->q_raw = ALLOC_ALIGNED_D(params.nrf);

  if (!bufs->temp_bb_ref_re || !bufs->temp_bb_ref_im || !bufs->temp_bb_sig_re || !bufs->temp_bb_sig_im ||
      !bufs->temp_ref_sym || !bufs->temp_sig_sym || !bufs->temp_complex_buf || !bufs->i_raw || !bufs->q_raw) {
    rf_setup_free(bufs);
    return -1;
  }

  return 0;
}

void rf_setup_free(RfSimBuffers *bufs) {
  if (!bufs) return;
  free(bufs->env_re);
  free(bufs->env_im);
  free(bufs->rf_ref);
  free(bufs->rf_sig);
  free(bufs->bb_ref_re);
  free(bufs->bb_ref_im);
  free(bufs->bb_sig_re);
  free(bufs->bb_sig_im);
  free(bufs->ref_sym);
  free(bufs->sig_sym);
  free(bufs->temp_bb_ref_re);
  free(bufs->temp_bb_ref_im);
  free(bufs->temp_bb_sig_re);
  free(bufs->temp_bb_sig_im);
  free(bufs->temp_ref_sym);
  free(bufs->temp_sig_sym);
  free(bufs->temp_complex_buf);
  free(bufs->i_raw);
  free(bufs->q_raw);
  memset(bufs, 0, sizeof(RfSimBuffers));
}

int rf_setup_build_envelope(RfSimBuffers *bufs, const Complex *tx_symbols, size_t nsym, const SimConfig *cfg, const RfSimParams *params) {
  if (cfg->rolloff <= 0.0) {
    for (size_t i = 0u; i < nsym; ++i) {
      for (size_t k = 0u; k < (size_t)params->sps; ++k) {
        bufs->env_re[i * (size_t)params->sps + k] = tx_symbols[i].re;
        bufs->env_im[i * (size_t)params->sps + k] = tx_symbols[i].im;
      }
    }
  } else {
    const int span = 20;
    size_t full_pulse_len = (size_t)span * (size_t)params->sps + 1u;
    double *pulse = (double *)calloc(full_pulse_len, sizeof(double));
    if (!pulse) return -1;

    rrc_build_pulse(cfg->rolloff, params->sps, span, pulse);
    rrc_apply_pulse(tx_symbols, nsym, pulse, (int)params->pulse_len, params->sps, bufs->env_re, bufs->env_im);
    free(pulse);
  }

  /* Map the shaped waveform onto the physical input power level (SoA) */
  {
    const double local_noise_w = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
    const double tx_signal_w = local_noise_w * db_to_lin_power(cfg->input_snr_db);
    const double tx_target_mean_square_v = tx_signal_w * R_LOAD_OHM;

    double p = 0.0;
    for (size_t i = 0; i < params->nrf; ++i) {
      p += bufs->env_re[i] * bufs->env_re[i] + bufs->env_im[i] * bufs->env_im[i];
    }
    const double tx_current_mean_square_v = p / (double)params->nrf;

    if (tx_current_mean_square_v > 0.0 && tx_target_mean_square_v > 0.0) {
      double amp = sqrt(tx_target_mean_square_v / tx_current_mean_square_v);
      for (size_t i = 0; i < params->nrf; ++i) {
        bufs->env_re[i] *= amp;
        bufs->env_im[i] *= amp;
      }
    }
  }

  return 0;
}
