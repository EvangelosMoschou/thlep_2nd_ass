#define _USE_MATH_DEFINES
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <omp.h>

#include "sim_rf_realistic.h"
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

/* Impairment module headers */
#include "adc_model.h"
#include "biquad_filter.h"
#include "flicker_noise.h"
#include "iq_imbalance.h"
#include "phase_noise.h"

/* Reference to global PRNG state in main.c */
extern PrngState rng;

/* Per-thread PRNG for RF simulation noise injection */
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
 * REALISTIC RF SIMULATION IMPLEMENTATION
 * ============================================================================ */

int simulate_realistic_rf(
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

  size_t m = 0u;
  size_t i;

  const double N_t0_W = K_BOLTZMANN * cfg->t0_k * B_NOISE_HZ;
  double N_current = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
  double Gain_total = 1.0;
  const double P_sig_in_W = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ *
                            db_to_lin_power(cfg->input_snr_db);

  /* Impairment module configs */
  PhaseNoiseConfig tx_lo_pn = {0};
  PhaseNoiseConfig rx_lo_pn = {0};
  IQImbalanceConfig iq_cfg = {0};
  FlickerNoiseConfig flicker_cfg = {0};
  ADCModelConfig adc_cfg = {0};
  BiquadState bb_filter_state = {0};

  /* Initialize impairment modules */
  {
    /* TX LO phase noise */
    tx_lo_pn.white_floor_dbc_hz = -155.0;
    tx_lo_pn.f2_corner_hz = 100e3;
    tx_lo_pn.f3_corner_hz = 10e3;
    tx_lo_pn.f2_slope_dbc_hz = -100.0;
    tx_lo_pn.f3_slope_dbc_hz = -80.0;
    tx_lo_pn.sample_rate_hz = fs_hz;
    if (phase_noise_init(&tx_lo_pn) != 0) {
      fprintf(stderr, "Failed to init TX LO phase noise\n");
      goto cleanup;
    }

    /* RX LO phase noise */
    rx_lo_pn.white_floor_dbc_hz = -150.0;
    rx_lo_pn.f2_corner_hz = 50e3;
    rx_lo_pn.f3_corner_hz = 5e3;
    rx_lo_pn.f2_slope_dbc_hz = -95.0;
    rx_lo_pn.f3_slope_dbc_hz = -75.0;
    rx_lo_pn.sample_rate_hz = fs_hz;
    if (phase_noise_init(&rx_lo_pn) != 0) {
      fprintf(stderr, "Failed to init RX LO phase noise\n");
      goto cleanup;
    }

    /* I/Q imbalance from first BB stage if available */
    iq_cfg.gain_error_db = 0.3;
    iq_cfg.phase_error_deg = 1.5;
    if (bb_stage_count > 0) {
      if (bb_stages[0].iq_gain_error_db > 0.0 || bb_stages[0].iq_phase_error_deg > 0.0) {
        iq_cfg.gain_error_db = bb_stages[0].iq_gain_error_db;
        iq_cfg.phase_error_deg = bb_stages[0].iq_phase_error_deg;
      }
    }

    /* Flicker noise */
    flicker_cfg.corner_freq_hz = 1000.0;
    flicker_cfg.white_noise_power = 1e-12;
    flicker_cfg.sample_rate_hz = fs_hz / (double)dec_factor;
    if (flicker_noise_init(&flicker_cfg) != 0) {
      fprintf(stderr, "Failed to init flicker noise\n");
      goto cleanup;
    }

    /* ADC model */
    adc_cfg.bit_depth = 12;
    adc_cfg.full_scale_vpp = 1.0;
    adc_cfg.jitter_ps = 1.0;
    if (adc_model_init(&adc_cfg) != 0) {
      fprintf(stderr, "Failed to init ADC model\n");
      goto cleanup;
    }

    /* Biquad filter for BB stages */
    {
      BiquadConfig bq_cfg = {0};
      double fs_bb = fs_hz / (double)dec_factor;
      bq_cfg.cutoff_hz = 0.4 * fs_bb;
      bq_cfg.fs_hz = fs_bb;
      bq_cfg.order = 4;
      if (biquad_init(&bb_filter_state, &bq_cfg) != 0) {
        fprintf(stderr, "Failed to init biquad filter\n");
        goto cleanup;
      }
    }
  }

  prng_init_parallel(rng_threads, (uint32_t)(cfg->seed + 1u));

  double t0 = omp_get_wtime();
  double t_pulse = 0, t_rf_stages = 0, t_downconv = 0, t_bb_stages = 0;

  if (rf_setup_build_envelope(&bufs, tx_symbols, nsym, cfg, &params) != 0) {
    goto cleanup;
  }

  /* Step 2: Add antenna noise to envelope — SoA */
  {
    double *env_noisy_re = ALLOC_ALIGNED_D(nrf);
    double *env_noisy_im = ALLOC_ALIGNED_D(nrf);
    if (!env_noisy_re || !env_noisy_im) {
      goto cleanup;
    }
    memcpy(env_noisy_re, env_re, nrf * sizeof(double));
    memcpy(env_noisy_im, env_im, nrf * sizeof(double));

    {
      const double P_noise_v2 =
          K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ * R_LOAD_OHM;
      add_awgn_soa(env_noisy_re, env_noisy_im, nrf, P_noise_v2);
    }

    /* Step 4: Apply TX LO phase noise before upconversion */
    {
      for (i = 0; i < nrf; i++) {
        double pn = phase_noise_generate(&tx_lo_pn, &rng);
        double cos_pn = cos(pn);
        double sin_pn = sin(pn);
        double tmp_re = env_noisy_re[i] * cos_pn - env_noisy_im[i] * sin_pn;
        double tmp_im = env_noisy_re[i] * sin_pn + env_noisy_im[i] * cos_pn;
        env_noisy_re[i] = tmp_re;
        env_noisy_im[i] = tmp_im;

        tmp_re = env_re[i] * cos_pn - env_im[i] * sin_pn;
        tmp_im = env_re[i] * sin_pn + env_im[i] * cos_pn;
        env_re[i] = tmp_re;
        env_im[i] = tmp_im;
      }
    }

    /* Step 5: IQ-modulate both clean and noisy envelopes — SoA */
    sim_rf_upconvert(env_re, env_im, nrf, fs_hz, cfg->carrier_hz, rf_ref);
    sim_rf_upconvert(env_noisy_re, env_noisy_im, nrf, fs_hz, cfg->carrier_hz, rf_sig);
    free(env_noisy_re);
    free(env_noisy_im);
  }

  /* Step 6: Record the INPUT RF trace metric */
  {
    metrics[m] =
        compute_stage_metric_real("input_rf_realistic", "rf_real", rf_ref, rf_sig, nrf);
    metrics[m].signal_power = P_sig_in_W;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db(P_sig_in_W / N_current);
    }
    write_trace_stage_artifacts(csv_dir, trace_dir, "traces", 0u, 1,
                                "input_rf_realistic", &metrics[m], "RF", rf_ref, rf_sig,
                                nrf, 24000u, fs_hz, cfg->carrier_hz);

    {
      const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      sim_rf_downconvert(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      sim_rf_downconvert(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
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
          "input_rf_realistic", "rf_to_bb", temp_ref_sym, temp_sig_sym, temp_eval_n);
      metrics[m].evm_pct = temp_metric.evm_pct;

      write_constellation_stage_artifacts(
          csv_dir, const_dir, "receiver", 0u, 1, "input_rf_realistic", &metrics[m],
          "RF", constellation_template, constellation_count, tx_symbols,
          temp_sig_sym, temp_eval_n);
    }

    ++m;
  }

  t_pulse = omp_get_wtime();

  /* Step 7: Process through RF frontend stages with AM-to-PM */
  double realistic_center_hz = cfg->carrier_hz;
  for (i = 0u; i < rf_stage_count; ++i) {
    StageModel stage = rf_stages[i];

    metrics[m] =
        apply_stage_real_fused(&stage, rf_ref, rf_sig, nrf, "rf_real", N_t0_W, fs_hz,
                         cfg->carrier_hz, &N_current, &Gain_total, P_sig_in_W);
    if (stage.lo_hz > 0.0) {
      realistic_center_hz = fabs(realistic_center_hz - stage.lo_hz);
    }
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    write_trace_stage_artifacts(csv_dir, trace_dir, "traces", i + 1u, 0,
                                stage.name, &metrics[m], "RF", rf_ref, rf_sig,
                                nrf, 6000u, fs_hz, realistic_center_hz);

    {
      double spectrum_span = 4.0e9;
      if (stage.lo_hz > 0.0 && realistic_center_hz < 1e9) {
        spectrum_span = 10.0e6;
      }
      write_stage_spectrum(spectrum_dir, "receiver", i + 1u, stage.name,
                           rf_ref, nrf, fs_hz,
                           realistic_center_hz, spectrum_span);
    }

    {  /* Generate Constellation & EVM for realistic path */
      const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      sim_rf_downconvert(rf_ref, nrf, fs_hz, realistic_center_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      sim_rf_downconvert(rf_sig, nrf, fs_hz, realistic_center_hz, cutoff_hz,
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

  /* Step 8: Downconvert RF → complex baseband — SoA */
  {
    const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
    sim_rf_downconvert(rf_ref, nrf, fs_hz, realistic_center_hz, cutoff_hz,
                 dec_factor, bb_ref_re, bb_ref_im, i_raw, q_raw);
    sim_rf_downconvert(rf_sig, nrf, fs_hz, realistic_center_hz, cutoff_hz,
                 dec_factor, bb_sig_re, bb_sig_im, i_raw, q_raw);
  }

  /* IF Spectrum for realistic path: same as baseline */
  if (realistic_center_hz != cfg->carrier_hz) {
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

    metrics[m] = compute_stage_metric_complex("MIX2_Downconv_realistic", "rf_to_bb", ref_sym,
                                        sig_sym, ref_eval_n);
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db((P_sig_in_W * Gain_total) / N_current);
    }
    write_constellation_stage_artifacts(
        csv_dir, const_dir, "receiver", rf_stage_count + 1u, 0,
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

    /* LNA 3 VGA disabled — same impedance mismatch bug as baseband path above */

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

    {
      double bb_fs = fs_hz / (double)dec_factor;
      size_t nfft = 1u; while (nfft * 2u <= nbb) nfft *= 2u;
      size_t m_fft = nfft + 2u;
      double *freq = malloc(m_fft * sizeof(double));
      double *mag = malloc(m_fft * sizeof(double));
      if (freq && mag) {
        int nb = fft_complex_spectrum_dB(bb_sig_re, bb_sig_im, nfft, bb_fs, freq, mag, m_fft);
        if (nb > 0) {
          size_t max_d = 1000u, di, nc = (size_t)nb;
          if (nc > max_d) {
            size_t stride = nc / max_d;
            for (di = 0u; di < max_d; di++) {
              double sa = 0.0; size_t sj, start = di * stride;
              for (sj = 0u; sj < stride && start + sj < nc; sj++) {
                sa += mag[start + sj];
              }
              mag[di] = sa / (double)stride;
              freq[di] = freq[start + stride/2u];
            }
            nc = max_d;
          }
          char path[512];
          snprintf(path, sizeof(path), "%s/receiver_stage_%02zu_spectrum.svg",
                   spectrum_dir, rf_stage_count + 2u + i);
          write_spectrum_svg(path, freq, mag, nc, 0.0, realistic_stage_name);
        }
      }
      free(freq); free(mag);
    }

    /* Write trace using continuous baseband waveform (match stage 6 format) -- realistic */
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    if (i < bb_stage_count - 1u) {
      write_complex_trace_stage_artifacts(
          trace_dir, "traces", rf_stage_count + 2u + i, 0, realistic_stage_name, &metrics[m],
          temp_complex_buf, nbb, fs_hz / (double)dec_factor, cfg->symbol_rate_hz, 10.0);
    }

    /* Downsample for metrics */
    pack_complex(bb_ref_re, bb_ref_im, nbb, temp_complex_buf);
    size_t ref_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, ref_sym);
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    size_t sig_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);
    size_t eval_n = (ref_eval_n < sig_eval_n) ? ref_eval_n : sig_eval_n;

    /* MATLAB parity:
     * Metrics are computed from downsampled symbols, not from the oversampled
     * waveform.  This matches the baseband path and the MATLAB cascade order of
     * processing */
    metrics[m] = compute_stage_metric_complex(stage.name, "rf_to_bb", ref_sym,
                                        sig_sym, eval_n);
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    metrics[m].gain_db = stage.gain_db;
    metrics[m].nf_db = stage.nf_db;
    metrics[m].filter_len = stage.filter_len;
    metrics[m].is_limiter = stage.is_limiter;
    /* Override with analytic Friis SNR */
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db((P_sig_in_W * Gain_total) / N_current);
    }

    if (i < bb_stage_count - 1u) {
      write_constellation_stage_artifacts(
          csv_dir, const_dir, "receiver", rf_stage_count + 2u + i, 0, stage.name,
          &metrics[m], "RF", constellation_template, constellation_count,
          tx_symbols, sig_sym, eval_n);
    }
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

  t_bb_stages = omp_get_wtime();

  double total = t_bb_stages - t0;
  fprintf(stderr, "\n=== TIMING Realistic RF (16 threads) ===\n");
  fprintf(stderr, "Pulse shaping + upconv + input metric: %.2fs (%.1f%%)\n", t_pulse - t0, (t_pulse - t0) / total * 100);
  fprintf(stderr, "RF stages (%zu): %.2fs (%.1f%%)\n", (size_t)rf_stage_count, t_rf_stages - t_pulse, (t_rf_stages - t_pulse) / total * 100);
  fprintf(stderr, "Downconversion: %.2fs (%.1f%%)\n", t_downconv - t_rf_stages, (t_downconv - t_rf_stages) / total * 100);
  fprintf(stderr, "BB stages (%zu): %.2fs (%.1f%%)\n", (size_t)bb_stage_count, t_bb_stages - t_downconv, (t_bb_stages - t_downconv) / total * 100);
  fprintf(stderr, "Total: %.2fs\n", total);

cleanup:
  /* Clean up all working buffers */
  rf_setup_free(&bufs);

  /* Free impairment module resources */
  phase_noise_free(&tx_lo_pn);
  phase_noise_free(&rx_lo_pn);
  flicker_noise_free(&flicker_cfg);
  adc_model_free(&adc_cfg);

  return 0;
}
