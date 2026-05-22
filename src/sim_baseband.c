#include "sim_baseband.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "math_utils.h"
#include "metrics.h"
#include "physics.h"
#include "prng.h"
#include "signal_chain.h"
#include "stage_artifacts.h"
#include "stage_models.h"
#include "fft.h"

static void copy_complex(Complex *restrict dst, const Complex *restrict src,
                         size_t n) {
  if (!dst || !src || n == 0u) {
    return;
  }
  memcpy(dst, src, n * sizeof(Complex));
}

static void add_awgn_complex_re(Complex *x, size_t n, double noise_power,
                                PrngState *rng) {
  size_t i;
  double sigma;

  if (!x || n == 0u || noise_power <= 0.0) {
    return;
  }

  sigma = sqrt(noise_power / 2.0);
  for (i = 0u; i < n; ++i) {
    x[i].re += sigma * prng_gauss(rng);
    x[i].im += sigma * prng_gauss(rng);
  }
}

static double complex_real_vpp(const Complex *x, size_t n) {
  size_t i;
  double mn;
  double mx;

  if (!x || n == 0u) {
    return 0.0;
  }

  mn = x[0].re;
  mx = x[0].re;
  for (i = 1u; i < n; ++i) {
    if (x[i].re < mn) mn = x[i].re;
    if (x[i].re > mx) mx = x[i].re;
  }
  return mx - mn;
}

int simulate_baseband(const SimConfig *cfg,
                      const StageModelsConfig *stage_cfg,
                      const Complex *constellation_template,
                      size_t constellation_count,
                      const Complex *tx_symbols,
                      size_t nsym,
                      PrngState *rng,
                      SimBasebandResult *result,
                      const char *csv_dir,
                      const char *const_dir,
                      const char *trace_dir) {

  const StageModel *stages = NULL;
  size_t stage_count;
  size_t m = 0u;
  size_t i;
  Complex *ref;
  Complex *sig;

  const double N_t0_W_bb = K_BOLTZMANN * cfg->t0_k * B_NOISE_HZ;
  double N_current_bb = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
  double Gain_total_bb = 1.0;
  const double P_sig_in_W_bb = K_BOLTZMANN * cfg->antenna_temp_k *
                                B_NOISE_HZ *
                                db_to_lin_power(cfg->input_snr_db);

  stage_count = stage_models_get(stage_cfg, STAGE_CHAIN_BASEBAND_RX, &stages);

  if (!stages || stage_count == 0u) {
    return -2;
  }

  if (stage_count + 1u > MAX_BB_METRICS) {
    fprintf(stderr,
            "baseband_rx has %zu stages; increase MAX_BB_METRICS (currently %d)\n",
            stage_count, MAX_BB_METRICS);
    return -3;
  }

  ref = (Complex *)calloc(nsym, sizeof(Complex));
  sig = (Complex *)calloc(nsym, sizeof(Complex));
  if (!ref || !sig) {
    free(ref);
    free(sig);
    return -1;
  }

  copy_complex(ref, tx_symbols, nsym);
  copy_complex(sig, tx_symbols, nsym);

  {
    const double ps = mean_power_complex(ref, nsym);
    const double pn = ps / db_to_lin_power(cfg->input_snr_db);
    add_awgn_complex_re(sig, nsym, pn, rng);
  }

  {
    result->metrics[m] =
        compute_metric_complex("input", "complex_baseband", ref, sig, nsym);
    if (N_current_bb > 0.0) {
      result->metrics[m].snr_db = lin_to_db(P_sig_in_W_bb / N_current_bb);
    }

    if (csv_dir && const_dir) {
      write_constellation_stage_artifacts(csv_dir, const_dir, "receiver",
                                          0u, 1, "input", &result->metrics[m],
                                          "Baseband", constellation_template,
                                          constellation_count, ref, sig, nsym);
    }
    if (trace_dir) {
      write_complex_trace_stage_artifacts(
          trace_dir, "traces", 0u, 1, "input", &result->metrics[m], sig, nsym,
          cfg->symbol_rate_hz, cfg->symbol_rate_hz, -1.0);
    }
    ++m;
  }

  for (i = 0u; i < stage_count; ++i) {
    StageModel stage = stages[i];

    result->metrics[m] = apply_stage_complex(&stage, ref, sig, nsym,
                                             "complex_baseband", N_t0_W_bb,
                                             &N_current_bb, &Gain_total_bb,
                                             P_sig_in_W_bb, rng);

    /* Skip constellation/trace artifacts for the last stage (e.g. antenna) */
    if (i < stage_count - 1u) {
      if (csv_dir && const_dir) {
        write_constellation_stage_artifacts(
            csv_dir, const_dir, "receiver", i + 1u, 0, stage.name,
            &result->metrics[m], "Baseband", constellation_template,
            constellation_count, ref, sig, nsym);
      }
      if (trace_dir) {
        write_complex_trace_stage_artifacts(trace_dir, "traces", i + 1u, 0,
                                            stage.name, &result->metrics[m], sig,
                                            nsym, cfg->symbol_rate_hz,
                                            cfg->symbol_rate_hz, -1.0);
      }
    }
    ++m;
  }

  result->final_vpp = complex_real_vpp(ref, nsym);
  result->count = m;

  free(ref);
  free(sig);
  return 0;
}

int simulate_transmitter(const SimConfig *cfg,
                         const StageModelsConfig *tx_stage_cfg,
                         const Complex *constellation_template,
                         size_t constellation_count,
                         const Complex *tx_symbols,
                         size_t nsym,
                         PrngState *rng,
                         const char *csv_dir,
                         const char *const_dir,
                         const char *trace_dir,
                         const char *spectrum_dir) {
  const StageModel *stages = NULL;
  size_t stage_count;
  size_t i;
  Complex *ref;
  Complex *sig;

  const double N_t0_W = K_BOLTZMANN * cfg->t0_k * B_NOISE_HZ;
  double N_current = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
  double Gain_total = 1.0;
  const double P_sig_in_W = K_BOLTZMANN * cfg->antenna_temp_k *
                            B_NOISE_HZ * db_to_lin_power(cfg->input_snr_db);

  stage_count = stage_models_get(tx_stage_cfg, STAGE_CHAIN_BASEBAND_RX, &stages);
  if (!stages || stage_count == 0u) return -2;

  ref = (Complex *)calloc(nsym, sizeof(Complex));
  sig = (Complex *)calloc(nsym, sizeof(Complex));
  if (!ref || !sig) { free(ref); free(sig); return -1; }

  copy_complex(ref, tx_symbols, nsym);
  copy_complex(sig, tx_symbols, nsym);

  {
    const double ps = mean_power_complex(ref, nsym);
    const double pn = ps / db_to_lin_power(cfg->input_snr_db);
    add_awgn_complex_re(sig, nsym, pn, rng);
  }

  if (const_dir) {
    StageMetric in_metric =
        compute_metric_complex("tx_input", "tx", ref, sig, nsym);
    if (N_current > 0.0) in_metric.snr_db = lin_to_db(P_sig_in_W / N_current);
    write_constellation_stage_artifacts(
        csv_dir, const_dir, "transmitter", 0u, 1, "input", &in_metric,
        "TX", constellation_template, constellation_count, ref, sig, nsym);
  }

  for (i = 0u; i < stage_count; ++i) {
    StageModel stage = stages[i];
    StageMetric metric = apply_stage_complex(
        &stage, ref, sig, nsym, "tx", N_t0_W, &N_current, &Gain_total,
        P_sig_in_W, rng);

    if (const_dir && csv_dir && i < stage_count - 1u) {
      write_constellation_stage_artifacts(
          csv_dir, const_dir, "transmitter", i + 1u, 0, stage.name,
          &metric, "TX", constellation_template, constellation_count,
          ref, sig, nsym);
    }
    if (trace_dir && i < stage_count - 1u) {
      write_complex_trace_stage_artifacts(
          trace_dir, "traces", i + 1u, 0, stage.name, &metric, sig, nsym,
          cfg->symbol_rate_hz, cfg->symbol_rate_hz, -1.0);
    }
    if (spectrum_dir && i < stage_count - 1u) {
      double *rsig = malloc(nsym * sizeof(double));
      if (rsig) {
        size_t j; for (j = 0u; j < nsym; j++) rsig[j] = sig[j].re;
        size_t nfft = 1u; while (nfft * 2u <= nsym && nfft < 512u) nfft *= 2u;
        size_t nb = nfft / 2u;
        double *freq = malloc(nb * sizeof(double));
        double *mag  = malloc(nb * sizeof(double));
        if (freq && mag && fft_spectrum_dB(rsig, nfft, cfg->symbol_rate_hz, freq, mag, nb) > 0) {
          char sp[512]; snprintf(sp, sizeof(sp), "%s/transmitter_stage_%02zu_spectrum.svg", spectrum_dir, i + 1u);
          write_spectrum_svg(sp, freq, mag, nb, cfg->symbol_rate_hz, stage.name);
        }
        free(freq); free(mag); free(rsig);
      }
    }
  }

  free(ref);
  free(sig);
  return 0;
}
