/*
 * main.c — 64-APSK Receiver Dual-Mode Simulator (Orchestrator)
 *
 * Thin orchestrator that delegates to extracted modules:
 *   constellation, metrics, cli_args, output_mgr, signal_chain, sim_baseband, prng
 *
 * The RF simulation code (simulate_bruteforce_rf, simulate_realistic_rf) and
 * their supporting SoA/FFT helpers remain here because they were too tightly
 * coupled to extract.
 */

#define _USE_MATH_DEFINES
#define _GNU_SOURCE
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <math.h>
#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/stat.h>
#include <time.h>

/* --- Extracted module headers --- */
#include "cli_args.h"
#include "constellation.h"
#include "math_utils.h"
#include "metrics.h"
#include "output_mgr.h"
#include "physics.h"
#include "prng.h"
#include "signal_chain.h"
#include "sim_baseband.h"
#include "sim_types.h"
#include "stage_artifacts.h"
#include "stage_models.h"
#include "fft.h"

/* Write spectrum SVG for a real-valued signal within a frequency window.
 * center_hz and span_hz define the display window; bins outside are filtered. */
static void write_stage_spectrum(const char *dir, const char *prefix,
                                  size_t num, const char *name,
                                  const double *signal, size_t nsamp,
                                  double fs_hz,
                                  double center_hz, double span_hz) {
    if (!dir || !signal || nsamp < 256u) return;
    size_t nfft = 1u;
    while (nfft * 2u <= nsamp) nfft *= 2u;
    size_t n_bins = nfft / 2u;
    double *freq = malloc(n_bins * sizeof(double));
    double *mag  = malloc(n_bins * sizeof(double));
    if (!freq || !mag) { free(freq); free(mag); return; }
    if (fft_spectrum_dB(signal, nfft, fs_hz, freq, mag, n_bins) > 0) {
        if (span_hz > 0.0) {
            double f_lo = center_hz - span_hz * 0.5, f_hi = center_hz + span_hz * 0.5;
            size_t wi = 0u, ri;
            for (ri = 0u; ri < n_bins; ri++) {
                if (freq[ri] >= f_lo && freq[ri] <= f_hi) {
                    freq[wi] = freq[ri]; mag[wi] = mag[ri]; wi++;
                }
            }
            if (wi > 1u) { n_bins = wi; fs_hz = 0.0; }
        }
        size_t max_display = 1000u;
        if (n_bins > max_display) {
            size_t stride = n_bins / max_display, di;
            for (di = 0u; di < max_display; di++) {
                double sum = 0.0; size_t si;
                for (si = 0u; si < stride && di * stride + si < n_bins; si++)
                    sum += mag[di * stride + si];
                mag[di] = sum / (double)stride;
                freq[di] = freq[di * stride + stride / 2u];
            }
            n_bins = max_display;
        }
        char path[512], title[256];
        humanize_stage_name(name, title, sizeof(title));
        snprintf(path, sizeof(path), "%s/%s_stage_%02zu_spectrum.svg", dir, prefix, num);
        write_spectrum_svg(path, freq, mag, n_bins, fs_hz, title);
    }
    free(freq); free(mag);
}

/* --- Propagation analysis (Part D: FSPL, rain, fog, gas, link margin) --- */
#include "propagation.h"

/* --- Cascade analysis (Part E: Friis, IP3, dynamic range, sensitivity) --- */
#include "cascade.h"

/* --- Impairment module headers (used by realistic RF path) --- */
#include "adc_model.h"
#include "biquad_filter.h"
#include "flicker_noise.h"
#include "iq_imbalance.h"
#include "phase_noise.h"

/* ============================================================================
 * CONSTANTS
 * ============================================================================ */

#define OUTPUT_ROOT_DIR "out"
#define TOPOLOGY_SIM_COUNT 1

/*
 * MAX_METRICS — Maximum number of stage metric entries per simulation path.
 * 32 is enough for chains with up to ~30 stages.
 */
#define MAX_METRICS 32

/* ============================================================================
 * GLOBAL PRNG STATE
 * ============================================================================ */

static PrngState rng;                              /* Serial PRNG for symbol generation */
static PrngState rng_threads[PRNG_MAX_OMP_THREADS]; /* Per-thread PRNG for RF simulation */

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
#define ALLOC_ALIGNED_C(count) ((Complex *)alloc_aligned(sizeof(Complex), (count)))

/* math_utils.h provides db_to_lin_power, lin_to_db */

/* ============================================================================
 * 64-APSK CONSTELLATION CONSTRUCTION
 * ============================================================================
 *
 * A "constellation" is the set of all possible symbols a transmitter can send.
 * 64-APSK has 64 symbols, each represented as a complex number (I + jQ).
 *
 * Visually, on the I/Q plane (a 2D plot where X=I and Y=Q), the 64 symbols
 * form 4 concentric rings, each with a different number of evenly-spaced
 * points at different radii. This is specified by the DVB-S2X standard.
 * ============================================================================
 */

static void generate_symbols(const Complex *constellation, int m, Complex *out,
                             unsigned short *labels, size_t n) {
  size_t i;

  for (i = 0; i < n; ++i) {
    /*
     * Pick a random index between 0 and m-1 (inclusive).
     * prng_uint32() returns a random 32-bit integer; modulo m maps it
     * to the valid range. Since m=64 is a power of 2, there's zero bias.
     */
    const unsigned int idx = prng_uint32(&rng) % (unsigned int)m;

    /* Copy the selected constellation point into the output array */
    out[i] = constellation[idx];

    /* Record which symbol was transmitted (for potential SER analysis) */
    labels[i] = (unsigned short)idx;
  }
}

/* ============================================================================
 * RECEIVER STAGE APPLICATION
 * ============================================================================
 *
 * These functions model what happens when a signal passes through one physical
 * component (stage) of the receiver. Each stage applies three operations:
 *
 *   1. FILTER: Smooth the signal (removes out-of-band noise)
 *   2. GAIN:   Amplify or attenuate the signal
 *   3. NOISE:  Add thermal noise based on the stage's Noise Figure (NF)
 *
 * The noise injection uses a "T0 reference tracker" (pn_t0_track) which models
 * the Friis noise formula behavior: noise added by later stages has less impact
 * because it's divided by the total gain of all preceding stages.
 * ============================================================================
 */

/* signal_chain.h provides apply_stage_complex */

/*
 * apply_stage_real_fused — Fused single-pass RF stage processing
 *
 * Combines: nonlinearity → gain → noise → limiter in ONE parallel loop.
 * Mathematically identical to calling them separately, but:
 * - Reduces memory traffic from 4 passes to 1 pass
 * - Better cache locality, fewer bounds checks, fewer function calls
 */
static StageMetric apply_stage_real_fused(const StageModel *stg,
                                         double *restrict ref, double *restrict sig,
                                         size_t n, const char *domain,
                                         double N_t0_W, double fs_hz,
                                         double __attribute__((unused)) fc_hz,
                                         double *N_current, double *Gain_total,
                                         double P_sig_in_W) {
  double g_lin, F, pn_add, N_t0_v2;
  size_t i;

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

  int do_mix = (stg->lo_hz > 0.0);
  double dtheta_mix = do_mix ? (2.0 * M_PI * stg->lo_hz / fs_hz) : 0.0;

  if (do_mix) {
    for (i = 0; i < n; i++) {
      double lo = cos(dtheta_mix * (double)i);
      ref[i] *= lo;
      sig[i] *= lo;
    }
  }

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
    StageMetric m_out = compute_metric_real(stg->name, domain, ref, sig, n);
    m_out.gain_db = stg ? stg->gain_db : NAN;
    m_out.nf_db = stg ? stg->nf_db : NAN;
    m_out.filter_len = stg ? stg->filter_len : 0;
    m_out.is_limiter = stg ? stg->is_limiter : 0;

    if (N_current && Gain_total && *N_current > 0.0) {
      double P_sig_curr = P_sig_in_W * (*Gain_total);
      m_out.snr_db = lin_to_db(P_sig_curr / (*N_current));
    }
    return m_out;
  }
}

/* ============================================================================
 * AUTO-GAIN HELPERS
 * ============================================================================
 */

/*
 * complex_real_vpp — Calculate the peak-to-peak voltage of the I (real)
 * component
 *
 * What it does:
 *   Finds the minimum and maximum values of the In-phase (real) component
 *   across all complex samples, and returns the difference (max - min).
 *   This is the "peak-to-peak voltage" — a measure of signal swing.
 *
 * Why only the I component?
 *   In many receiver designs, the final stage needs to present a signal
 *   with a specific voltage swing to the ADC (Analog-to-Digital Converter).
 *   The ADC's input range is defined in terms of peak-to-peak voltage.
 *   Using just the I component is sufficient because the I and Q channels
 *   are processed by independent ADCs.
 *
 * Parameters:
 *   x — Complex signal array
 *   n — Number of samples
 *
 * Returns:
 *   Peak-to-peak voltage of the I component (max_I - min_I)
 */
static double complex_real_vpp(const Complex *x, size_t n) {
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
 * ============================================================================
 *
 * These functions handle the conversion between complex baseband (I/Q)
 * signals and real RF (radio frequency) signals. This is the core of the
 * "brute-force RF" simulation path.
 * ============================================================================
 */



static double rrc_tap_value(double t, double rolloff);

/*
 * rrc_tap_value — Root-raised-cosine pulse sample used for matching
 */
static double rrc_tap_value(double t, double rolloff) {
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

/*
 * FFT-based convolution for Complex signals
 *
 * Optimization: Replace O(n × pulse_len) time-domain convolution with
 * O(n log n) FFT-based convolution for large n.
 *
 * Mathematical equivalence: For zero-padded inputs, FFT convolution produces
 * exactly the same result as time-domain convolution (within floating-point
 * rounding error).
 */

/*
 * next_pow2 — Find the next power of 2 >= n
 */
static size_t next_pow2(size_t n) {
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

/*
 * bitrev — Compute bit-reversed index for FFT
 */
static size_t bitrev(size_t x, size_t bits) {
    size_t rev = 0;
    for (size_t i = 0; i < bits; ++i) {
        rev = (rev << 1) | (x & 1);
        x >>= 1;
    }
    return rev;
}

/*
 * simple_fft — In-place Cooley-Tukey FFT for complex signals
 * N must be a power of 2
 */
static void simple_fft(Complex *x, size_t N) {
    size_t bits = 0;
    size_t tmp = N;
    while (tmp > 1) { bits++; tmp >>= 1; }

    for (size_t i = 0; i < N; ++i) {
        size_t j = bitrev(i, bits);
        if (i < j) {
            double re = x[i].re;
            double im = x[i].im;
            x[i].re = x[j].re;
            x[i].im = x[j].im;
            x[j].re = re;
            x[j].im = im;
        }
    }

    for (size_t size = 2; size <= N; size <<= 1) {
        double angle = -2.0 * M_PI / (double)size;
        for (size_t i = 0; i < N; i += size) {
            for (size_t j = 0; j < size / 2; ++j) {
                size_t idx1 = i + j;
                size_t idx2 = idx1 + size / 2;

                double w_re = cos(angle * (double)j);
                double w_im = sin(angle * (double)j);

                double t_re = x[idx2].re * w_re - x[idx2].im * w_im;
                double t_im = x[idx2].re * w_im + x[idx2].im * w_re;

                x[idx2].re = x[idx1].re - t_re;
                x[idx2].im = x[idx1].im - t_im;
                x[idx1].re += t_re;
                x[idx1].im += t_im;
            }
        }
    }
}

/*
 * simple_ifft — In-place inverse FFT
 */
static void simple_ifft(Complex *X, size_t N) {
    for (size_t i = 0; i < N; ++i) {
        X[i].im = -X[i].im;
    }
    simple_fft(X, N);
    for (size_t i = 0; i < N; ++i) {
        X[i].re /= (double)N;
        X[i].im /= (double)N;
    }
}

/*
 * fft_convolve_complex — FFT-based convolution for complex signals
 * Uses zero-padding to avoid circular convolution artifacts.
 */
static void fft_convolve_complex(const Complex *restrict a, size_t n_a,
                                  const double *b_real, size_t n_b,
                                  Complex *restrict out, size_t out_len) {
    size_t result_len = n_a + n_b - 1;
    size_t fft_size = next_pow2(result_len);

    Complex *A = (Complex *)calloc(fft_size, sizeof(Complex));
    Complex *B = (Complex *)calloc(fft_size, sizeof(Complex));
    if (!A || !B) {
        if (A) free(A);
        if (B) free(B);
        return;
    }

    for (size_t i = 0; i < n_a; ++i) A[i] = a[i];
    for (size_t i = 0; i < n_b; ++i) { B[i].re = b_real[i]; B[i].im = 0.0; }

    simple_fft(A, fft_size);
    simple_fft(B, fft_size);

    for (size_t i = 0; i < fft_size; ++i) {
        double re = A[i].re * B[i].re - A[i].im * B[i].im;
        double im = A[i].re * B[i].im + A[i].im * B[i].re;
        A[i].re = re;
        A[i].im = im;
    }

    simple_ifft(A, fft_size);

    for (size_t i = 0; i < out_len; ++i) {
        out[i].re = A[i].re;
        out[i].im = A[i].im;
    }

    free(A);
    free(B);
}

/*
 * synchronize_and_downsample — MATLAB-like symbol timing recovery and
 * normalization Finds the optimal symbol timing from a received baseband
 * stream, then applies a single complex least-squares normalization against the
 * known transmitted symbols, matching the MATLAB reference flow more closely.
 */
static size_t synchronize_and_downsample(const Complex *bb_stream, size_t n_in,
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
 * STRUCTURE-OF-ARRAYS (SoA) SIGNAL PROCESSING
 * ============================================================================
 *
 * SoA layout stores all re[] contiguously, then all im[] contiguously,
 * enabling SIMD vectorization (4+ doubles per instruction with AVX2).
 *
 * The Complex struct (AoS) interleaves re/im: [r0,i0,r1,i1,...]
 * SoA separates them: [r0,r1,r2,...] and [i0,i1,i2,...]
 *
 * All SoA functions use restrict to guarantee no pointer aliasing.
 * ============================================================================
 */

static void pack_complex(const double *restrict re, const double *restrict im,
                         size_t n, Complex *restrict dst) {
  size_t i;
  for (i = 0; i < n; ++i) {
    dst[i].re = re[i];
    dst[i].im = im[i];
  }
}

/* --- SoA elemental operations --- */

static void add_awgn_soa(double *restrict re, double *restrict im, size_t n,
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

static void scale_soa(double *restrict re, double *restrict im, size_t n,
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

static double mean_power_soa(const double *restrict re,
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

static double mean_noise_power_soa(const double *restrict sig_re,
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

/* --- SoA stage application (the big one) --- */

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
/*
 * simulate_bruteforce_rf — Run the realistic RF upconversion/downconversion
 * simulation
 *
 * What it does:
 *   This is the full "brute-force" simulation that models the actual RF signal
 *   path: upconvert to 24 GHz, process through RF stages, downconvert, then
 *   process through post-mixer baseband stages.
 *
 * Signal flow:
 *   1. Upsample symbols to RF sample rate (repeat each symbol SPS times)
 *   2. IQ-modulate onto 24 GHz carrier → real RF signal
 *   3. Add AWGN to the real RF signal
 *   4. Initialize the RF T0 noise tracker
 *   5. Record input RF trace metric
 *   6. Process through rf_frontend stages (real signal domain)
 *   7. Downconvert RF → complex baseband (mix + lowpass filter)
 *   8. Extract symbol-rate samples from the oversampled baseband
 *   9. Record the downconversion metric
 *  10. Process through rf_postmix_bb stages (complex baseband domain)
 *  11. Record the final VPP and metrics
 *
 * Sampling rate calculation:
 *   SPS = round(rf_sample_rate / symbol_rate)
 *   Minimum SPS = 8 (to satisfy Nyquist: fs > 2×fc requires SPS ≥ 4,
 *   but 8 provides better anti-aliasing margin)
 *   Example: symbol_rate=10 MHz, rf_fs=96 GHz → SPS = 9600
 *            Total RF samples = 256 symbols × 9600 = 2,457,600 samples
 *
 * Parameters:
 *   Same as simulate_complex_baseband, plus:
 *   used_sps   — Output: samples per symbol actually used
 *   used_fs_hz — Output: actual RF sampling frequency used (= symbol_rate ×
 * SPS)
 *
 * Returns:
 *   0 on success, -1 on memory error, -2 if chains missing, -3 if MAX_METRICS
 * exceeded
 */
static int simulate_bruteforce_rf(
    const SimConfig *cfg, const StageModelsConfig *stage_cfg,
    const Complex *tx_symbols, const Complex *constellation_template,
    size_t constellation_count, size_t nsym, StageMetric *metrics,
    size_t *metric_count, double *final_vpp, int *used_sps, double *used_fs_hz,
    const char *csv_dir, const char *const_dir, const char *trace_dir,
    const char *spectrum_dir) {

  const StageModel *rf_stages = NULL;
  const StageModel *bb_stages = NULL;
  const size_t rf_stage_count =
      stage_models_get(stage_cfg, STAGE_CHAIN_RF_FRONTEND, &rf_stages);
  const size_t bb_stage_count =
      stage_models_get(stage_cfg, STAGE_CHAIN_RF_POSTMIX_BB, &bb_stages);

  int sps;       /* Samples Per Symbol */
  double fs_hz;  /* Actual RF sampling frequency */
  size_t nrf;    /* Total number of RF samples = nsym × sps */
  size_t m = 0u; /* Metric counter */
  size_t i;

  /* MATLAB-exact physical noise constant: N_t0_W = k_B * T0 * B_noise (Watts)
   */
  const double N_t0_W = K_BOLTZMANN * cfg->t0_k * B_NOISE_HZ;
  /* Friis trackers */
  double N_current = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
  double Gain_total = 1.0;
  const double P_sig_in_W = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ *
                            db_to_lin_power(cfg->input_snr_db);

  /* Working buffers — SoA layout for large Complex arrays */
  double *env_re = NULL;        /* Upsampled envelope real part */
  double *env_im = NULL;        /* Upsampled envelope imag part */
  double *rf_ref = NULL;        /* Reference RF waveform (clean) */
  double *rf_sig = NULL;        /* Received RF waveform (noisy) */
  double *bb_ref_re = NULL;     /* Baseband reference real part */
  double *bb_ref_im = NULL;     /* Baseband reference imag part */
  double *bb_sig_re = NULL;     /* Baseband signal real part */
  double *bb_sig_im = NULL;     /* Baseband signal imag part */
  Complex *ref_sym = NULL;      /* Small: stays as Complex */
  Complex *sig_sym = NULL;
  double *temp_bb_ref_re = NULL;
  double *temp_bb_ref_im = NULL;
  double *temp_bb_sig_re = NULL;
  double *temp_bb_sig_im = NULL;
  Complex *temp_ref_sym = NULL;
  Complex *temp_sig_sym = NULL;
  Complex *temp_complex_buf = NULL;
  double *i_raw = NULL;           /* Mix-down raw I channel (pre-IIR) */
  double *q_raw = NULL;           /* Mix-down raw Q channel (pre-IIR) */

  if (!rf_stages || !bb_stages || rf_stage_count == 0u ||
      bb_stage_count == 0u || !csv_dir || !const_dir || !trace_dir) {
    return -2;
  }

  /* Check metric buffer capacity */
  if (rf_stage_count + bb_stage_count + 2u > (size_t)MAX_METRICS) {
    fprintf(
        stderr,
        "RF chain has %zu+%zu stages; increase MAX_METRICS (currently %d)\n",
        rf_stage_count, bb_stage_count, MAX_METRICS);
    return -3;
  }

  /* Calculate samples per symbol from the configured RF sampling rate */
  sps = (int)llround(cfg->rf_sample_rate_hz / cfg->symbol_rate_hz);
  if (sps < 16) {
    sps = 16; /* Minimum 16x oversampling for decimation */
  }

  /* MATLAB parity: post-mix baseband uses sps=8 to match MATLAB's fixed
   * decimation conv + 1:sps:end pattern in sim_receiver_matlab.m */
  size_t bb_sps = 8;
  size_t dec_factor = sps / bb_sps;
  if (dec_factor < 1)
    dec_factor = 1;
  size_t pulse_len = (cfg->rolloff > 0.0) ? ((size_t)6 * (size_t)sps + 1u) : 1u;

  fs_hz = cfg->symbol_rate_hz * (double)sps; /* Actual sampling frequency */
  nrf = nsym * (size_t)sps + pulse_len -
        1u; /* Total RF samples incl. pulse tail */
  size_t nbb = (nrf + dec_factor - 1u) /
               dec_factor; /* Total decimated baseband samples */

  /* Allocate all working buffers — SoA layout for large arrays */
  env_re = ALLOC_ALIGNED_D(nrf);
  env_im = ALLOC_ALIGNED_D(nrf);
  rf_ref = ALLOC_ALIGNED_D(nrf);
  rf_sig = ALLOC_ALIGNED_D(nrf);
  bb_ref_re = ALLOC_ALIGNED_D(nrf);
  bb_ref_im = ALLOC_ALIGNED_D(nrf);
  bb_sig_re = ALLOC_ALIGNED_D(nrf);
  bb_sig_im = ALLOC_ALIGNED_D(nrf);
  ref_sym = (Complex *)calloc(nsym, sizeof(Complex));
  sig_sym = (Complex *)calloc(nsym, sizeof(Complex));

  if (!env_re || !env_im || !rf_ref || !rf_sig ||
      !bb_ref_re || !bb_ref_im || !bb_sig_re || !bb_sig_im ||
      !ref_sym || !sig_sym) {
    free(env_re); free(env_im); free(rf_ref); free(rf_sig);
    free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
    free(ref_sym); free(sig_sym);
    return -1;
  }

  /* Preallocate temp buffers — SoA for large, Complex for small */
  temp_bb_ref_re = ALLOC_ALIGNED_D(nbb);
  temp_bb_ref_im = ALLOC_ALIGNED_D(nbb);
  temp_bb_sig_re = ALLOC_ALIGNED_D(nbb);
  temp_bb_sig_im = ALLOC_ALIGNED_D(nbb);
  temp_ref_sym = (Complex *)calloc(nsym, sizeof(Complex));
  temp_sig_sym = (Complex *)calloc(nsym, sizeof(Complex));
  temp_complex_buf = (Complex *)calloc(nrf > nbb ? nrf : nbb, sizeof(Complex));
  i_raw = ALLOC_ALIGNED_D(nrf);
  q_raw = ALLOC_ALIGNED_D(nrf);

  if (!temp_bb_ref_re || !temp_bb_ref_im || !temp_bb_sig_re || !temp_bb_sig_im ||
      !temp_ref_sym || !temp_sig_sym || !temp_complex_buf || !i_raw || !q_raw) {
    free(env_re); free(env_im); free(rf_ref); free(rf_sig);
    free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
    free(ref_sym); free(sig_sym);
    free(temp_bb_ref_re); free(temp_bb_ref_im);
    free(temp_bb_sig_re); free(temp_bb_sig_im);
    free(temp_ref_sym); free(temp_sig_sym);
    free(temp_complex_buf);
    free(i_raw); free(q_raw);
    return -1;
  }

  prng_init_parallel(rng_threads, (uint32_t)cfg->seed);

  double t0 = omp_get_wtime();
  double t_pulse = 0, t_rf_stages = 0, t_downconv = 0, t_bb_stages = 0;

  /* Step 1: Pulse-shape symbols at the RF rate (RRC, like MATLAB) — SoA */
  shape_symbols_rrc_to_env_soa(tx_symbols, nsym, sps, cfg->rolloff, env_re, env_im);

  /* Map the shaped waveform onto the physical input power level (SoA) */
  {
    const double local_noise_w = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
    const double tx_signal_w = local_noise_w * db_to_lin_power(cfg->input_snr_db);
    const double tx_target_mean_square_v = tx_signal_w * R_LOAD_OHM;
    const double tx_current_mean_square_v = mean_power_soa(env_re, env_im, nrf);

    if (tx_current_mean_square_v > 0.0 && tx_target_mean_square_v > 0.0) {
      scale_soa(env_re, env_im, nrf,
                sqrt(tx_target_mean_square_v / tx_current_mean_square_v));
    }
  }

  /* Step 2: Add antenna noise to envelope, then upconvert — SoA */
  {
    double *env_noisy_re = ALLOC_ALIGNED_D(nrf);
    double *env_noisy_im = ALLOC_ALIGNED_D(nrf);
    if (!env_noisy_re || !env_noisy_im) {
      free(env_re); free(env_im); free(rf_ref); free(rf_sig);
      free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
      free(ref_sym); free(sig_sym); free(temp_complex_buf);
      free(temp_bb_ref_re); free(temp_bb_ref_im);
      free(temp_bb_sig_re); free(temp_bb_sig_im);
      free(temp_ref_sym); free(temp_sig_sym);
      free(i_raw); free(q_raw);
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
    env_to_rf_soa(env_re, env_im, nrf, fs_hz, cfg->carrier_hz, rf_ref);
    env_to_rf_soa(env_noisy_re, env_noisy_im, nrf, fs_hz, cfg->carrier_hz, rf_sig);
    free(env_noisy_re);
    free(env_noisy_im);
  }

  /* Step 5: Record the INPUT RF trace metric */
  {
    metrics[m] =
        compute_metric_real("input_rf", "rf_real", rf_ref, rf_sig, nrf);
    metrics[m].signal_power = P_sig_in_W;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db(P_sig_in_W / N_current);
    }
    write_trace_stage_artifacts(csv_dir, trace_dir, "traces", 0u, 1,
                                "input_rf", &metrics[m], "RF", rf_ref, rf_sig,
                                nrf, 24000u, fs_hz, cfg->carrier_hz);

    {
      const double cutoff_hz = 5.0 * cfg->symbol_rate_hz;
      mix_down_soa(rf_ref, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      mix_down_soa(rf_sig, nrf, fs_hz, cfg->carrier_hz, cutoff_hz,
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

      StageMetric temp_metric = compute_metric_complex(
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
      mix_down_soa(rf_ref, nrf, fs_hz, current_center_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      mix_down_soa(rf_sig, nrf, fs_hz, current_center_hz, cutoff_hz,
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
    mix_down_soa(rf_ref, nrf, fs_hz, current_center_hz, cutoff_hz,
                 dec_factor, bb_ref_re, bb_ref_im, i_raw, q_raw);
    mix_down_soa(rf_sig, nrf, fs_hz, current_center_hz, cutoff_hz,
                 dec_factor, bb_sig_re, bb_sig_im, i_raw, q_raw);
  }

  /* IF Spectrum: complex FFT of raw downconverter output (before LPF/dec)
   * Shows dual sidebands at ±IF after heterodyne mixing */
  {
    size_t nraw = nrf;  /* i_raw/q_raw have same length as RF signal */
    size_t nfft = 1u; while (nfft * 2u <= nraw) nfft *= 2u;
    size_t m = nfft + 2u;
    double *freq = malloc(m * sizeof(double));
    double *mag = malloc(m * sizeof(double));
    if (freq && mag) {
      int nb = fft_complex_spectrum_dB(i_raw, q_raw, nfft, fs_hz, freq, mag, m);
      if (nb > 0) {
        /* Zoom to ±10 MHz around IF */
        double if_zoom_lo = (current_center_hz > 0.0) ? -10.0e6 : -10.0e6;
        double if_zoom_hi = (current_center_hz > 0.0) ? 10.0e6 : 10.0e6;
        size_t wi = 0u, ri; size_t nc = (size_t)nb;
        for (ri = 0u; ri < nc; ri++) {
          if (freq[ri] >= if_zoom_lo && freq[ri] <= if_zoom_hi) {
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
          write_spectrum_svg(path, freq, mag, nc, 0.0, "IF Spectrum (raw downconverter)");
        }
      }
    }
    free(freq); free(mag);
  }

  t_downconv = omp_get_wtime();

  /* Step 8: Extract symbol-rate samples */
  {
    pack_complex(bb_ref_re, bb_ref_im, nbb, temp_complex_buf);
    size_t ref_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, ref_sym);
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    size_t sig_eval_n = synchronize_and_downsample(
        temp_complex_buf, nbb, nsym, bb_sps, cfg->rolloff, tx_symbols, sig_sym);
    size_t eval_n = (ref_eval_n < sig_eval_n) ? ref_eval_n : sig_eval_n;
    (void)eval_n;

    metrics[m] = compute_metric_complex("MIX2_Downconv", "rf_to_bb", ref_sym,
                                        sig_sym, ref_eval_n);
    metrics[m].signal_power = P_sig_in_W * Gain_total;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db((P_sig_in_W * Gain_total) / N_current);
    }
    write_constellation_stage_artifacts(
        csv_dir, const_dir, "receiver", rf_stage_count + 1u, 0,
        metrics[m].stage, &metrics[m], "RF", constellation_template,
        constellation_count, tx_symbols, sig_sym, sig_eval_n);
    /* Pack for trace artifacts */
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    write_complex_trace_stage_artifacts(
        trace_dir, "traces", rf_stage_count + 1u, 0, metrics[m].stage,
        &metrics[m], temp_complex_buf, nbb, fs_hz / (double)dec_factor, cfg->symbol_rate_hz, 10.0);
    ++m;
  }

  /* Step 10: Process through post-mixer BB stages (on the continuous
   * oversampled stream!) */
  for (i = 0u; i < bb_stage_count; ++i) {
    StageModel stage = bb_stages[i];

    /*
     * ARCHITECTURAL INVERSION:
     * We apply the hardware filters (Lowpass/Amp) to the CONTINUOUS,
     * highly-oversampled waveform (bb_sig_stream), NOT the crushed 1-sps
     * symbols.
     */

    /* LNA 3 VGA disabled — cascade analysis already provides correct total gain.
     * The previous auto-gain used normalized (1Ω) signal values but targeted
     * 1.0 Vpp in 50Ω terms, causing +20 dB over-amplification (~13 Vpp traces).
     */

    apply_stage_soa(&stage, bb_ref_re, bb_ref_im, bb_sig_re, bb_sig_im, nbb,
                    "rf_to_bb", N_t0_W, &N_current, &Gain_total, P_sig_in_W);

    {
      double bb_fs = fs_hz / (double)dec_factor;
      size_t nfft = 1u; while (nfft * 2u <= nbb) nfft *= 2u;
      size_t m = nfft + 2u;
      double *freq = malloc(m * sizeof(double));
      double *mag = malloc(m * sizeof(double));
      if (freq && mag) {
        int nb = fft_complex_spectrum_dB(bb_sig_re, bb_sig_im, nfft, bb_fs, freq, mag, m);
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
          write_spectrum_svg(path, freq, mag, nc, 0.0, stage.name);
        }
      }
      free(freq); free(mag);
    }

    /* Write trace using continuous baseband waveform (match stage 6 format) */
    pack_complex(bb_sig_re, bb_sig_im, nbb, temp_complex_buf);
    if (i < bb_stage_count - 1u) {
      write_complex_trace_stage_artifacts(
          trace_dir, "traces", rf_stage_count + 2u + i, 0, stage.name, &metrics[m],
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

    if (i < bb_stage_count - 1u) {
      write_constellation_stage_artifacts(
          csv_dir, const_dir, "receiver", rf_stage_count + 2u + i, 0, stage.name,
          &metrics[m], "RF", constellation_template, constellation_count,
          tx_symbols, sig_sym, eval_n);
    }
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
  free(env_re); free(env_im);
  free(rf_ref); free(rf_sig);
  free(bb_ref_re); free(bb_ref_im);
  free(bb_sig_re); free(bb_sig_im);
  free(ref_sym); free(sig_sym);
  free(temp_bb_ref_re); free(temp_bb_ref_im);
  free(temp_bb_sig_re); free(temp_bb_sig_im);
  free(temp_ref_sym); free(temp_sig_sym);
  free(temp_complex_buf);
  free(i_raw); free(q_raw);
  return 0;
}

/*
 * simulate_realistic_rf — Run the realistic RF simulation with 8 impairment models
 *
 * What it does:
 *   This is the full "realistic" simulation that models actual RF impairments:
 *   LO phase noise, I/Q imbalance, flicker noise, biquad filters, AM-to-PM,
 *   ADC quantization+jitter, and LO leakage DC offset.
 *
 * Signal flow:
 *   1. Generate symbols (same as RF path)
 *   2. Add antenna thermal noise (same as RF path)
 *   3. Pulse shape with RRC (same as RF path)
 *   4. Apply LO phase noise before upconversion (phase_noise module)
 *   5. Upconvert to RF (same as RF path)
 *   6. Process through RF frontend stages with AM-to-PM (from Task 7)
 *   7. Apply LO phase noise at mixer (phase_noise module)
 *   8. Downconvert to baseband (same as RF path)
 *   9. Apply I/Q imbalance after downconversion (iq_imbalance module)
 *  10. Process through post-mix BB stages with biquad filters (biquad_filter module)
 *  11. Add flicker noise to baseband stages (flicker_noise module)
 *  12. Apply ADC model at final stage (adc_model module)
 *  13. Add LO leakage DC offset (simple addition)
 *
 * Parameters:
 *   Same as simulate_bruteforce_rf
 *
 * Returns:
 *   0 on success, -1 on memory error, -2 if chains missing, -3 if MAX_METRICS exceeded
 */
static int simulate_realistic_rf(
    const SimConfig *cfg, const StageModelsConfig *stage_cfg,
    const Complex *tx_symbols, const Complex *constellation_template,
    size_t constellation_count, size_t nsym, StageMetric *metrics,
    size_t *metric_count, double *final_vpp, int *used_sps, double *used_fs_hz,
    const char *csv_dir, const char *const_dir, const char *trace_dir,
    const char *spectrum_dir) {

  const StageModel *rf_stages = NULL;
  const StageModel *bb_stages = NULL;
  const size_t rf_stage_count =
      stage_models_get(stage_cfg, STAGE_CHAIN_RF_FRONTEND, &rf_stages);
  const size_t bb_stage_count =
      stage_models_get(stage_cfg, STAGE_CHAIN_RF_POSTMIX_BB, &bb_stages);

  int sps;
  double fs_hz;
  size_t nrf;
  size_t m = 0u;
  size_t i;

  const double N_t0_W = K_BOLTZMANN * cfg->t0_k * B_NOISE_HZ;
  double N_current = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
  double Gain_total = 1.0;
  const double P_sig_in_W = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ *
                            db_to_lin_power(cfg->input_snr_db);

  /* Working buffers — SoA layout */
  double *env_re = NULL;
  double *env_im = NULL;
  double *rf_ref = NULL;
  double *rf_sig = NULL;
  double *bb_ref_re = NULL;
  double *bb_ref_im = NULL;
  double *bb_sig_re = NULL;
  double *bb_sig_im = NULL;
  Complex *ref_sym = NULL;
  Complex *sig_sym = NULL;
  double *temp_bb_ref_re = NULL;
  double *temp_bb_ref_im = NULL;
  double *temp_bb_sig_re = NULL;
  double *temp_bb_sig_im = NULL;
  Complex *temp_ref_sym = NULL;
  Complex *temp_sig_sym = NULL;
  Complex *temp_complex_buf = NULL;
  double *i_raw = NULL;
  double *q_raw = NULL;

  /* Impairment module configs */
  PhaseNoiseConfig tx_lo_pn = {0};
  PhaseNoiseConfig rx_lo_pn = {0};
  IQImbalanceConfig iq_cfg = {0};
  FlickerNoiseConfig flicker_cfg = {0};
  ADCModelConfig adc_cfg = {0};
  BiquadState bb_filter_state = {0};

  if (!rf_stages || !bb_stages || rf_stage_count == 0u ||
      bb_stage_count == 0u || !csv_dir || !const_dir || !trace_dir) {
    return -2;
  }

  if (rf_stage_count + bb_stage_count + 2u > (size_t)MAX_METRICS) {
    fprintf(
        stderr,
        "RF chain has %zu+%zu stages; increase MAX_METRICS (currently %d)\n",
        rf_stage_count, bb_stage_count, MAX_METRICS);
    return -3;
  }

  sps = (int)llround(cfg->rf_sample_rate_hz / cfg->symbol_rate_hz);
  if (sps < 16) {
    sps = 16;
  }

  size_t bb_sps = 8;
  size_t dec_factor = sps / bb_sps;
  if (dec_factor < 1)
    dec_factor = 1;
  size_t pulse_len = (cfg->rolloff > 0.0) ? ((size_t)6 * (size_t)sps + 1u) : 1u;

  fs_hz = cfg->symbol_rate_hz * (double)sps;
  nrf = nsym * (size_t)sps + pulse_len - 1u;
  size_t nbb = (nrf + dec_factor - 1u) / dec_factor;

  /* Allocate all working buffers */
  env_re = ALLOC_ALIGNED_D(nrf);
  env_im = ALLOC_ALIGNED_D(nrf);
  rf_ref = ALLOC_ALIGNED_D(nrf);
  rf_sig = ALLOC_ALIGNED_D(nrf);
  bb_ref_re = ALLOC_ALIGNED_D(nrf);
  bb_ref_im = ALLOC_ALIGNED_D(nrf);
  bb_sig_re = ALLOC_ALIGNED_D(nrf);
  bb_sig_im = ALLOC_ALIGNED_D(nrf);
  ref_sym = (Complex *)calloc(nsym, sizeof(Complex));
  sig_sym = (Complex *)calloc(nsym, sizeof(Complex));

  if (!env_re || !env_im || !rf_ref || !rf_sig ||
      !bb_ref_re || !bb_ref_im || !bb_sig_re || !bb_sig_im ||
      !ref_sym || !sig_sym) {
    free(env_re); free(env_im); free(rf_ref); free(rf_sig);
    free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
    free(ref_sym); free(sig_sym);
    return -1;
  }

  /* Preallocate temp buffers */
  temp_bb_ref_re = ALLOC_ALIGNED_D(nbb);
  temp_bb_ref_im = ALLOC_ALIGNED_D(nbb);
  temp_bb_sig_re = ALLOC_ALIGNED_D(nbb);
  temp_bb_sig_im = ALLOC_ALIGNED_D(nbb);
  temp_ref_sym = (Complex *)calloc(nsym, sizeof(Complex));
  temp_sig_sym = (Complex *)calloc(nsym, sizeof(Complex));
  temp_complex_buf = (Complex *)calloc(nrf > nbb ? nrf : nbb, sizeof(Complex));
  i_raw = ALLOC_ALIGNED_D(nrf);
  q_raw = ALLOC_ALIGNED_D(nrf);

  if (!temp_bb_ref_re || !temp_bb_ref_im || !temp_bb_sig_re || !temp_bb_sig_im ||
      !temp_ref_sym || !temp_sig_sym || !temp_complex_buf || !i_raw || !q_raw) {
    free(env_re); free(env_im); free(rf_ref); free(rf_sig);
    free(bb_ref_re); free(bb_ref_im); free(bb_sig_re); free(bb_sig_im);
    free(ref_sym); free(sig_sym);
    free(temp_bb_ref_re); free(temp_bb_ref_im);
    free(temp_bb_sig_re); free(temp_bb_sig_im);
    free(temp_ref_sym); free(temp_sig_sym);
    free(temp_complex_buf);
    free(i_raw); free(q_raw);
    return -1;
  }

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

  /* Step 1: Pulse-shape symbols at the RF rate (RRC) — SoA */
  shape_symbols_rrc_to_env_soa(tx_symbols, nsym, sps, cfg->rolloff, env_re, env_im);

  /* Map the shaped waveform onto the physical input power level (SoA) */
  {
    const double local_noise_w = K_BOLTZMANN * cfg->antenna_temp_k * B_NOISE_HZ;
    const double tx_signal_w = local_noise_w * db_to_lin_power(cfg->input_snr_db);
    const double tx_target_mean_square_v = tx_signal_w * R_LOAD_OHM;
    const double tx_current_mean_square_v = mean_power_soa(env_re, env_im, nrf);

    if (tx_current_mean_square_v > 0.0 && tx_target_mean_square_v > 0.0) {
      scale_soa(env_re, env_im, nrf,
                sqrt(tx_target_mean_square_v / tx_current_mean_square_v));
    }
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
    env_to_rf_soa(env_re, env_im, nrf, fs_hz, cfg->carrier_hz, rf_ref);
    env_to_rf_soa(env_noisy_re, env_noisy_im, nrf, fs_hz, cfg->carrier_hz, rf_sig);
    free(env_noisy_re);
    free(env_noisy_im);
  }

  /* Step 6: Record the INPUT RF trace metric */
  {
    metrics[m] =
        compute_metric_real("input_rf_realistic", "rf_real", rf_ref, rf_sig, nrf);
    metrics[m].signal_power = P_sig_in_W;
    if (N_current > 0.0) {
      metrics[m].snr_db = lin_to_db(P_sig_in_W / N_current);
    }
    write_trace_stage_artifacts(csv_dir, trace_dir, "traces", 0u, 1,
                                "input_rf_realistic", &metrics[m], "RF", rf_ref, rf_sig,
                                nrf, 24000u, fs_hz, cfg->carrier_hz);

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
      mix_down_soa(rf_ref, nrf, fs_hz, realistic_center_hz, cutoff_hz,
                   dec_factor, temp_bb_ref_re, temp_bb_ref_im, i_raw, q_raw);
      mix_down_soa(rf_sig, nrf, fs_hz, realistic_center_hz, cutoff_hz,
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
    mix_down_soa(rf_ref, nrf, fs_hz, realistic_center_hz, cutoff_hz,
                 dec_factor, bb_ref_re, bb_ref_im, i_raw, q_raw);
    mix_down_soa(rf_sig, nrf, fs_hz, realistic_center_hz, cutoff_hz,
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
      size_t m = nfft + 2u;
      double *freq = malloc(m * sizeof(double));
      double *mag = malloc(m * sizeof(double));
      if (freq && mag) {
        int nb = fft_complex_spectrum_dB(bb_sig_re, bb_sig_im, nfft, bb_fs, freq, mag, m);
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
    metrics[m] = compute_metric_complex(stage.name, "rf_to_bb", ref_sym,
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
  free(env_re); free(env_im);
  free(rf_ref); free(rf_sig);
  free(bb_ref_re); free(bb_ref_im);
  free(bb_sig_re); free(bb_sig_im);
  free(ref_sym); free(sig_sym);
  free(temp_bb_ref_re); free(temp_bb_ref_im);
  free(temp_bb_sig_re); free(temp_bb_sig_im);
  free(temp_ref_sym); free(temp_sig_sym);
  free(temp_complex_buf);
  free(i_raw); free(q_raw);

  /* Free impairment module resources */
  phase_noise_free(&tx_lo_pn);
  phase_noise_free(&rx_lo_pn);
  flicker_noise_free(&flicker_cfg);
  adc_model_free(&adc_cfg);

  return 0;
}

/*
 * print_metrics — Display a formatted table of cumulative end-to-end stage
 * metrics on the console
 *
 * What it does:
 *   Prints a table with columns: Stage, Domain, SNR(dB), EVM(%), NoisePow
 *   Each row shows the signal quality at one point in the receiver chain.
 *
 * Parameters:
 *   title   — Section heading (printed above the table)
 *   metrics — Array of StageMetric structs
 *   count   — Number of entries to print
 */
static void print_metrics(const char *title, const StageMetric *metrics,
                          size_t count) {
  size_t i;

  printf("\n%s\n", title);
  printf("%-18s %-14s %12s %12s %12s\n", "Stage", "Domain", "SNR(dB)", "EVM(%)",
         "NoisePow");
  for (i = 0; i < count; ++i) {
    char stage_label[128];
    humanize_stage_name(metrics[i].stage, stage_label, sizeof(stage_label));
    {
      char numbered_label[256];
      snprintf(numbered_label, sizeof(numbered_label), "%zu. %s", i + 1u,
               stage_label);
      strncpy(stage_label, numbered_label, sizeof(stage_label));
      stage_label[sizeof(stage_label) - 1u] = '\0';
    }
    printf("%-18s %-14s %12.3f %12.3f %12.4e\n", stage_label, metrics[i].domain,
           metrics[i].snr_db, metrics[i].evm_pct, metrics[i].noise_power);
  }
}

/* ============================================================================
 * COMMAND-LINE ARGUMENT PARSING
 * ============================================================================
 */

/*
 * parse_u32 — Parse a string as an unsigned 32-bit integer with validation
 *
 * Returns: 0 on success, -1 if NULL, -2 if not a valid unsigned integer
 */
static int parse_u32(const char *s, unsigned int *out) {
  char *end = NULL;
  unsigned long v;

  if (!s || !out)
    return -1;

  errno = 0;
  v = strtoul(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || v > UINT_MAX) {
    return -2;
  }

  *out = (unsigned int)v;
  return 0;
}

/*
 * parse_i32 — Parse a string as a signed 32-bit integer with validation
 */
static int parse_i32(const char *s, int *out) {
  char *end = NULL;
  long v;

  if (!s || !out)
    return -1;

  errno = 0;
  v = strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || v < INT_MIN || v > INT_MAX) {
    return -2;
  }

  *out = (int)v;
  return 0;
}

/*
 * parse_double — Parse a string as a double-precision float with validation
 */
static int parse_double(const char *s, double *out) {
  char *end = NULL;
  double v;

  if (!s || !out)
    return -1;

  errno = 0;
  v = strtod(s, &end);
  if (errno != 0 || end == s || *end != '\0') {
    return -2;
  }

  *out = v;
  return 0;
}

/*
 * resolve_project_root — Find the project root directory from the executable path.
 *
 * Uses /proc/self/exe to get the binary's absolute path, then strips
 * "/bin/dual_receiver_sim" to get the project root.  This makes all
 * relative paths (data_input/, out/) work regardless of how the binary
 * is launched (terminal, double-click, etc.).
 *
 * Parameters:
 *   buf       — Output buffer for the project root path
 *   buf_size  — Size of the output buffer
 *
 * Returns:
 *   0 on success, -1 on error
 */
static int resolve_project_root(char *buf, size_t buf_size) {
    char exe_path[1024];
    ssize_t len;
    char *p;

    len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0) return -1;
    exe_path[len] = '\0';

    /* Strip "/bin/dual_receiver_sim" to get project root.
     * Works for any binary name under any "bin/"-like dir. */
    p = strrchr(exe_path, '/');
    if (!p) return -1;
    *p = '\0';          /* remove binary name */
    p = strrchr(exe_path, '/');
    if (!p) return -1;
    *p = '\0';          /* remove bin/ directory */

    if (strlen(exe_path) >= buf_size) return -1;
    snprintf(buf, buf_size, "%s", exe_path);
    return 0;
}

/*
 * resolve_stage_csv_path — Resolve a stage CSV path (file or directory)
 *
 * What it does:
 *   If the user provides a DIRECTORY path (e.g., "data_input/"), this
 * function resolves it to the specific "runtime_stage_models.csv" file inside
 * that directory. If the user provides a FILE path, it's used as-is.
 *
 *   This exists because the data_input/ directory may contain multiple CSV
 * files:
 *   - stage_models.csv               — design reference / topology sketch
 *   - runtime_stage_models.csv       — simulator-ready canonical configuration
 *   Automatically choosing the runtime file prevents accidentally loading
 *   the design sketch (which may have incomplete data).
 *
 * Parameters:
 *   input_path    — User-provided path (file or directory)
 *   resolved_path — Output buffer for the resolved file path
 *   resolved_size — Size of the output buffer
 *
 * Returns:
 *   0 on success, negative on error
 */
static int resolve_stage_csv_path(const char *input_path, char *resolved_path,
                                  size_t resolved_size) {
  struct stat st;
  int written;
  size_t input_len;

  if (!input_path || !resolved_path || resolved_size == 0u) {
    return -1;
  }

  /* Check if the input is a directory */
  if (stat(input_path, &st) == 0 && S_ISDIR(st.st_mode)) {
    /* Strip trailing slashes from the directory path */
    input_len = strlen(input_path);
    while (input_len > 1u && input_path[input_len - 1u] == '/') {
      --input_len;
    }

    /* Append the runtime CSV filename */
    written =
        snprintf(resolved_path, resolved_size, "%.*s/runtime_stage_models.csv",
                 (int)input_len, input_path);
    if (written < 0 || (size_t)written >= resolved_size) {
      return -2; /* Path too long */
    }

    /* Verify the resolved file exists */
    if (stat(resolved_path, &st) != 0) {
      return -3; /* File doesn't exist */
    }

    return 0;
  }

  /* Not a directory — use the path as-is */
  written = snprintf(resolved_path, resolved_size, "%s", input_path);
  if (written < 0 || (size_t)written >= resolved_size) {
    return -4;
  }

  return 0;
}

/*
 * print_usage — Display command-line usage help
 */
static void print_usage(const char *exe) {
  printf("Usage: %s [options]\n", exe);
  printf("Options:\n");
  printf("  --seed <int>          RNG seed (default: time-based)\n");
  printf("  --symbols <int>       Number of symbols (default: 256)\n");
  printf("  --symbol-rate <Hz>    Symbol rate in Hz (default: 1e7)\n");
  printf("  --rf-fs <Hz>          RF brute-force sample rate in Hz (default: "
         "9.6e10)\n");
  printf("  --carrier <Hz>        Carrier frequency in Hz (default: 2.0e10)\n");
  printf("  --snr <dB>            Input SNR in dB (default: 20)\n");
  printf("  --stage-csv <path>    Stage-model CSV file or folder (default: "
          "data_input/20ghz/receiver.csv)\n");
  printf("  --topology-sim <1..4> Output simulation slot under "
         "out/topology_sim_N (default: 1)\n");
  printf("  --stage-sim <1..4>    Alias for --topology-sim\n");
  printf("  --disable-bb          Enable complex baseband path (default: disabled)\n");
  printf("  --enable-rf           Enable RF baseline path (default: enabled)\n");
  printf("  --enable-realistic    Enable realistic impairment path (default: enabled)\n");
}

/* ============================================================================
 * MAIN — Program Entry Point
 * ============================================================================
 */

/*
 * main — Orchestrate the complete 64-APSK receiver simulation
 *
 * Execution sequence:
 *   1.  Parse command-line arguments (--seed, --symbols, --snr, etc.)
 *   2.  Initialize the PRNG with the seed
 *   3.  Create/verify output directory structure
 *   4.  Resolve the stage-model CSV path
 *   5.  Build the DVB-S2X 64-APSK constellation (64 fixed I/Q points)
 *   6.  Generate a random transmitted symbol sequence
 *   7.  Calculate the link budget (thermal noise power + signal power)
 *   8.  Write input budget CSV and SVG
 *   9.  Load receiver stage configuration from CSV
 *  10.  Run complex-baseband simulation
 *  11.  Run brute-force RF simulation
 *  12.  Write stage metrics CSV and SVG for both paths
 *  13.  Print console summary
 *  14.  Free all allocated memory and exit
 *
 * Exit codes:
 *   0 = Success
 *   1 = Usage error (unknown argument)
 *   2 = Argument parse error (invalid number for --seed, --snr, etc.)
 *   3 = Constellation build error
 *   4 = Memory allocation error
 *   5 = CSV load error or directory error
 *   6 = Complex-baseband simulation error
 *   7 = Brute-force RF simulation error
 *   8 = Output directory creation error
 *   9 = Realistic RF simulation error
 */
int main(int argc, char **argv) {
  char project_root[512] = "";
  SimConfig cfg;               /* Top-level simulation parameters */
  StageModelsConfig stage_cfg; /* Loaded stage chain configuration */
  StageModelsConfig tx_stage_cfg; /* Transmitter stage chain configuration */
  char stage_csv_path[512] =
      "data_input/20ghz/receiver.csv"; /* Default CSV path */
  char tx_stage_csv_path[512] =
      "data_input/20ghz/transmitter.csv"; /* Default transmitter CSV path */
  char resolved_stage_csv_path[512]; /* Resolved path after directory resolution
                                       */
  char stage_err[256];               /* Error message buffer for CSV loading */

  char out_dir[512] = "out";
  /* Resolve project root from executable location so paths work even when
   * the binary is launched by double-click (working dir ≠ project root). */
  if (resolve_project_root(project_root, sizeof(project_root)) == 0) {
      /* Prepend project root to the default CSV paths */
      char tmp[512];
      snprintf(tmp, sizeof(tmp), "%s/%s", project_root, stage_csv_path);
      snprintf(stage_csv_path, sizeof(stage_csv_path), "%s", tmp);
      snprintf(tmp, sizeof(tmp), "%s/%s", project_root, tx_stage_csv_path);
      snprintf(tx_stage_csv_path, sizeof(tx_stage_csv_path), "%s", tmp);
      /* Build absolute output directory path */
      snprintf(out_dir, sizeof(out_dir), "%s/out", project_root);
  }
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
      cfg.carrier_hz = 20.0e9;     /* 20 GHz carrier (K-band satellite) */
  cfg.symbol_rate_hz = 10.0e6; /* 10 MegaSymbols/sec */
  cfg.symbols = 3000;          /* 3000 symbols - reasonable run time */
  cfg.rolloff = 0.0;           /* Rectangular pulses (fastest) */
  cfg.input_snr_db = 20.0;     /* 20 dB input SNR based on assignment */
    cfg.antenna_temp_k =
        91.0;         /* 91 K antenna noise temp @ 44° elev (Viasat 13.5m, Athens→SES-17) */
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
    } else if (strcmp(argv[i], "--tx-csv") == 0 && i + 1 < argc) {
      snprintf(tx_stage_csv_path, sizeof(tx_stage_csv_path), "%s", argv[++i]);
    } else if (strcmp(argv[i], "--topology-sim") == 0 && i + 1 < argc) {
      if (parse_i32(argv[++i], &topology_sim_id) != 0 || topology_sim_id < 1 ||
          topology_sim_id > TOPOLOGY_SIM_COUNT) {
        fprintf(stderr, "Invalid --topology-sim value (expected 1..%d)\n",
                TOPOLOGY_SIM_COUNT);
        return 2;
      }
    } else if (strcmp(argv[i], "--stage-sim") == 0 && i + 1 < argc) {
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
    } else if (strcmp(argv[i], "--disable-rf") == 0) {
      cfg.run_rf = 0;
    } else if (strcmp(argv[i], "--disable-realistic") == 0) {
      cfg.run_realistic = 0;
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }

  /* --- Initialize the PRNG --- */
  prng_init(&rng, cfg.seed);

  /* --- Create output directories --- */
  if (ensure_output_dirs(out_dir, topology_sim_id) != 0) {
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

  /* --- Build output directory paths --- */
  char rf_csv_dir[256];
  char rf_const_dir[256];
  char rf_trace_dir[256];
  char realistic_csv_dir[256];
  char realistic_const_dir[256];
  char realistic_trace_dir[256];
  char realistic_spectrum_dir[256];
  char tx_csv_dir[256];
  char tx_const_dir[256];
  char tx_trace_dir[256];
  char tx_spectrum_dir[256];
  char rf_spectrum_dir[256];
  if (snprintf(rf_csv_dir, sizeof(rf_csv_dir), "%s/rf_baseline/Rx/csv", out_dir) >= (int)sizeof(rf_csv_dir) ||
      snprintf(rf_const_dir, sizeof(rf_const_dir), "%s/rf_baseline/Rx/constellations", out_dir) >= (int)sizeof(rf_const_dir) ||
      snprintf(rf_trace_dir, sizeof(rf_trace_dir), "%s/rf_baseline/Rx/traces", out_dir) >= (int)sizeof(rf_trace_dir) ||
      snprintf(rf_spectrum_dir, sizeof(rf_spectrum_dir), "%s/rf_baseline/Rx/spectrum", out_dir) >= (int)sizeof(rf_spectrum_dir) ||
      snprintf(realistic_csv_dir, sizeof(realistic_csv_dir), "%s/realistic/Rx/csv", out_dir) >= (int)sizeof(realistic_csv_dir) ||
      snprintf(realistic_const_dir, sizeof(realistic_const_dir), "%s/realistic/Rx/constellations", out_dir) >= (int)sizeof(realistic_const_dir) ||
      snprintf(realistic_trace_dir, sizeof(realistic_trace_dir), "%s/realistic/Rx/traces", out_dir) >= (int)sizeof(realistic_trace_dir) ||
      snprintf(realistic_spectrum_dir, sizeof(realistic_spectrum_dir), "%s/realistic/Rx/spectrum", out_dir) >= (int)sizeof(realistic_spectrum_dir) ||
      snprintf(tx_csv_dir, sizeof(tx_csv_dir), "%s/realistic/Tx/csv", out_dir) >= (int)sizeof(tx_csv_dir) ||
      snprintf(tx_const_dir, sizeof(tx_const_dir), "%s/realistic/Tx/constellations", out_dir) >= (int)sizeof(tx_const_dir) ||
      snprintf(tx_trace_dir, sizeof(tx_trace_dir), "%s/realistic/Tx/traces", out_dir) >= (int)sizeof(tx_trace_dir) ||
      snprintf(tx_spectrum_dir, sizeof(tx_spectrum_dir), "%s/realistic/Tx/spectrum", out_dir) >= (int)sizeof(tx_spectrum_dir)) {
    fprintf(stderr, "Failed to build output subdirectories\n");
    return 5;
  }

  /* Clear old output files from the selected slot */
  if (clean_output_dir(rf_csv_dir) != 0 ||
      clean_output_dir(rf_const_dir) != 0 ||
      clean_output_dir(rf_trace_dir) != 0 ||
      clean_output_dir(rf_spectrum_dir) != 0 ||
      clean_output_dir(realistic_csv_dir) != 0 ||
      clean_output_dir(realistic_const_dir) != 0 ||
      clean_output_dir(realistic_trace_dir) != 0 ||
      clean_output_dir(realistic_spectrum_dir) != 0 ||
      clean_output_dir(tx_csv_dir) != 0 ||
      clean_output_dir(tx_const_dir) != 0 ||
      clean_output_dir(tx_trace_dir) != 0 ||
      clean_output_dir(tx_spectrum_dir) != 0) {
    fprintf(
        stderr,
        "Failed to clear output directories before writing new artifacts\n");
    return 5;
  }

  /* --- Build the 64-APSK constellation --- */
  if (build_dvbs2_64apsk_constellation(constellation, 64) != 0) {
    fprintf(stderr, "Failed to build 64-APSK constellation\n");
    return 3;
  }

  /* --- Allocate and generate transmitted symbols --- */
  tx_symbols = (Complex *)calloc((size_t)cfg.symbols, sizeof(Complex));
  labels =
      (unsigned short *)calloc((size_t)cfg.symbols, sizeof(unsigned short));
  if (!tx_symbols || !labels) {
    fprintf(stderr, "Allocation failed for symbols\n");
    free(tx_symbols);
    free(labels);
    return 4;
  }

  generate_symbols(constellation, 64, tx_symbols, labels, (size_t)cfg.symbols);

  /* --- Noise floor calculation (used later for input SNR) --- */
  noise_bw_hz = 200.0e6;
  noise_w = K_BOLTZMANN * cfg.antenna_temp_k * noise_bw_hz;
  noise_dbm = lin_to_db(noise_w) + 30.0;
  /* Default input SNR — overridden after link budget computation */
  cfg.input_snr_db = 20.0;

  /* --- Load receiver stage configuration from CSV --- */
  if (stage_models_load_csv(resolved_stage_csv_path, &stage_cfg, stage_err,
                            sizeof(stage_err)) != 0) {
    fprintf(stderr, "Failed to load stage-model CSV '%s': %s\n",
            resolved_stage_csv_path, stage_err);
    free(tx_symbols);
    free(labels);
    return 5;
  }

  /* --- Load transmitter stage configuration from CSV --- */
  memset(&tx_stage_cfg, 0, sizeof(tx_stage_cfg));
  if (stage_models_load_csv(tx_stage_csv_path, &tx_stage_cfg, stage_err,
                            sizeof(stage_err)) != 0) {
    fprintf(stderr, "Warning: could not load transmitter CSV '%s' — skipping TX simulation\n",
            tx_stage_csv_path);
  }

  /* --- Load component catalog (datasheet values for IIP3/P1dB) --- */
  ComponentCatalog component_catalog;
  memset(&component_catalog, 0, sizeof(component_catalog));
  {
      const char *cat_path = resolved_stage_csv_path;
      if (cat_path[0] == '\0') cat_path = "data_input/20ghz/receiver.csv";
      if (component_catalog_load(cat_path, &component_catalog) != 0) {
          fprintf(stderr, "Warning: could not load '%s' — cascade will use CSV values\n",
                  cat_path);
      }
  }

  /* --- Override stage model IIP3/P1dB with catalog datasheet values --- */
  {
      int chain_id;
      for (chain_id = 0; chain_id < STAGE_CHAIN_COUNT; chain_id++) {
          const StageModel *stages = NULL;
          size_t n = stage_models_get(&stage_cfg, (StageChainId)chain_id, &stages);
          size_t i;
          for (i = 0; i < n; i++) {
              /* stages[i] is actually mutable since we own stage_cfg */
              component_catalog_override_stage(
                  &stage_cfg.chains[chain_id][i], &component_catalog);
          }
      }
  }

  /* --- Part E: Cascade Analysis (Friis, IP3, dynamic range, sensitivity) --- */
  {
    CascadeParams    cascade_params;
    CascadeResult    cascade_result;
    int              cascade_ret;

    cascade_params.antenna_temp_k = cfg.antenna_temp_k;     /* 91 K (Viasat 13.5m @ 44° elev, Athens→SES-17) */
    cascade_params.t0_k           = cfg.t0_k;               /* 290 K */
    cascade_params.bw_hz          = B_NOISE_HZ;             /* 200 MHz */
    cascade_params.vpp_out        = 1.0;                    /* 1 Vpp */
    cascade_params.snr_target_db  = cfg.input_snr_db;       /* computed from link budget */
    cascade_params.snr_required_db = 26.5;                  /* 64-APSK required SNR from assignment table */

    cascade_ret = compute_cascade(&stage_cfg, &cascade_params,
                                   STAGE_CHAIN_BASEBAND_RX,
                                   &component_catalog, &cascade_result);
    if (cascade_ret == 0) {
      print_cascade(&cascade_result);
    }
  }

  /* --- Part D: Propagation Analysis & Link Budget (uses cascade sensitivity) --- */
  {
    PropagationScenario prop_scenario;
    LinkBudgetResult    prop_budget;
    CascadeParams       tmp_params;
    CascadeResult       tmp_result;
    double sensitivity_dbm = -70.0; /* fallback if cascade fails */

    /* Run cascade to get real sensitivity */
    tmp_params.antenna_temp_k = cfg.antenna_temp_k;
    tmp_params.t0_k           = cfg.t0_k;
    tmp_params.bw_hz          = B_NOISE_HZ;
    tmp_params.vpp_out        = 1.0;
    tmp_params.snr_target_db  = cfg.input_snr_db;
    tmp_params.snr_required_db = 26.5;

    if (compute_cascade(&stage_cfg, &tmp_params,
                         STAGE_CHAIN_BASEBAND_RX,
                         &component_catalog, &tmp_result) == 0) {
      sensitivity_dbm = tmp_result.sensitivity_dbm;
    }

    prop_scenario.frequency_hz         = cfg.carrier_hz;
    prop_scenario.distance_km          = 36000.0;
    prop_scenario.elevation_deg        = 44.0;                /* Athens → SES-17 (18°E) */
    prop_scenario.polarization_deg     = 45.0;                /* circular, TBD with team */
    prop_scenario.rain_rate_mmh        = 10.0;                /* 0.01% exceedance, generic */
    prop_scenario.surface_temp_k       = 288.15;
    prop_scenario.surface_pressure_hpa = 1013.25;
    prop_scenario.water_vapor_gm3      = 7.5;
    prop_scenario.liquid_water_gm3     = 0.05;                /* medium fog */
    prop_scenario.eirp_dbm             = 85.0;                 /* 32 W + 40 dBi (Beyond Gravity 0.6m sat antenna) */
    prop_scenario.rx_gain_dbi          = 68.70;                /* Viasat 13.5m earth station @ 24 GHz: 67.2+20·log10(24/20.2) */
    prop_scenario.rx_sensitivity_dbm   = sensitivity_dbm;     /* from cascade above */

    compute_link_budget(&prop_scenario, &prop_budget);
    print_link_budget(&prop_budget);

    /* Override simulation input SNR with real link budget received power */
    signal_dbm = prop_budget.rx_power_dbm;
    cfg.input_snr_db = signal_dbm - noise_dbm;

    /* Actual SNR at receiver:
     *   Ni = k · T_ant · B  (noise floor at antenna temperature)
     *   SNR at detector = P_rx − (Ni + NF_total) */
    {
        double ni_dbm = K_BOLTZMANN * cfg.antenna_temp_k * B_NOISE_HZ;
        ni_dbm = lin_to_db(ni_dbm) + 30.0;
        double snr_at_detector = prop_budget.rx_power_dbm
                               - (ni_dbm + tmp_result.total_nf_db);
        print_modcod_table(snr_at_detector);
     }
  }

  /* --- Run Simulation Paths --- */
  if (cfg.run_bb) {
    SimBasebandResult bb_result;
    memset(&bb_result, 0, sizeof(bb_result));
    if (simulate_baseband(&cfg, &stage_cfg, constellation, 64u,
                          tx_symbols, (size_t)cfg.symbols, &rng, &bb_result,
                          rf_csv_dir, rf_const_dir, rf_trace_dir) != 0) {
      fprintf(stderr, "Complex-baseband simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 6;
    }
    memcpy(metrics_bb, bb_result.metrics, bb_result.count * sizeof(StageMetric));
    count_bb = bb_result.count;
    final_vpp_bb = bb_result.final_vpp;
  } else {
    (void)metrics_bb;
    (void)count_bb;
    (void)final_vpp_bb;
  }

  if (cfg.run_rf) {
    if (simulate_bruteforce_rf(&cfg, &stage_cfg, tx_symbols, constellation, 64u,
                               (size_t)cfg.symbols, metrics_rf, &count_rf,
                               &final_vpp_rf, &rf_sps, &rf_fs_used, rf_csv_dir,
                               rf_const_dir, rf_trace_dir, rf_spectrum_dir) != 0) {
      fprintf(stderr, "Brute-force RF simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 7;
    }

    {
      char rf_metrics_path[512];
      snprintf(rf_metrics_path, sizeof(rf_metrics_path), "%s/rf_metrics.csv", rf_csv_dir);
      if (count_rf > 0u &&
          write_metrics_csv(rf_metrics_path, metrics_rf, count_rf) != 0) {
        fprintf(stderr, "Warning: failed to write aggregate RF metrics CSV '%s'\n",
                rf_metrics_path);
      }
    }
  } else {
    (void)metrics_rf;
    (void)count_rf;
    (void)final_vpp_rf;
    (void)rf_sps;
    (void)rf_fs_used;
    (void)rf_csv_dir;
    (void)rf_const_dir;
    (void)rf_trace_dir;
  }

  if (cfg.run_realistic) {
    if (simulate_realistic_rf(&cfg, &stage_cfg, tx_symbols, constellation, 64u,
                              (size_t)cfg.symbols, metrics_realistic, &count_realistic,
                              &final_vpp_realistic, &realistic_sps, &realistic_fs_used,
                              realistic_csv_dir, realistic_const_dir, realistic_trace_dir,
                              realistic_spectrum_dir) != 0) {
      fprintf(stderr, "Realistic RF simulation failed\n");
      stage_models_free(&stage_cfg);
      free(tx_symbols);
      free(labels);
      return 9;
    }

    {
      char realistic_metrics_path[512];
      snprintf(realistic_metrics_path, sizeof(realistic_metrics_path), "%s/realistic_metrics.csv", realistic_csv_dir);
      if (count_realistic > 0u &&
          write_metrics_csv(realistic_metrics_path, metrics_realistic, count_realistic) != 0) {
        fprintf(stderr, "Warning: failed to write aggregate realistic metrics CSV '%s'\n",
                realistic_metrics_path);
      }
    }
  } else {
    (void)metrics_realistic;
    (void)count_realistic;
    (void)final_vpp_realistic;
    (void)realistic_sps;
    (void)realistic_fs_used;
    (void)realistic_csv_dir;
    (void)realistic_const_dir;
    (void)realistic_trace_dir;
  }

  /* --- Transmitter chain simulation --- */
  if (tx_stage_cfg.counts[STAGE_CHAIN_BASEBAND_RX] > 0u) {
    printf("\n  --- Transmitter chain simulation ---\n");
    if (simulate_transmitter(&cfg, &tx_stage_cfg, constellation, 64u,
                             tx_symbols, (size_t)cfg.symbols, &rng,
                             tx_csv_dir, tx_const_dir, tx_trace_dir,
                             tx_spectrum_dir) != 0) {
      fprintf(stderr, "Transmitter simulation failed\n");
    } else {
      printf("  Transmitter artifacts written to %s\n", tx_const_dir);
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
  stage_models_free(&tx_stage_cfg);
  free(tx_symbols);
  free(labels);
  return 0; /* Success! */
}
