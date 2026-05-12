#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "signal_chain.h"
#include "physics.h"
#include "math_utils.h"

#define N_SAMPLES 1024

static int test_complex_stage(void) {
  PrngState rng;
  prng_init(&rng, 42);

  StageModel stg;
  memset(&stg, 0, sizeof(stg));
  stg.name = "TestLNA";
  stg.gain_db = 20.0;
  stg.nf_db = 3.0;
  stg.filter_len = 1;
  stg.is_limiter = 0;
  stg.p1db_dbm = INFINITY;
  stg.ip3_dbm = INFINITY;
  stg.am_pm_coeff = 0.0;

  Complex *ref = (Complex *)calloc(N_SAMPLES, sizeof(Complex));
  Complex *sig = (Complex *)calloc(N_SAMPLES, sizeof(Complex));


  for (int i = 0; i < N_SAMPLES; i++) {
    ref[i].re = 1.0;
    ref[i].im = 0.0;
    sig[i].re = 1.0;
    sig[i].im = 0.0;
  }

  double N_t0_W = K_BOLTZMANN * 290.0 * B_NOISE_HZ;
  double N_current = N_t0_W;
  double Gain_total = 1.0;
  double P_sig_in = 1.0;

  StageMetric m = apply_stage_complex(&stg, ref, sig, N_SAMPLES,
                                      "test", N_t0_W,
                                      &N_current, &Gain_total,
                                      P_sig_in, &rng);

  double g_lin = db_to_lin_power(20.0);

  printf("=== apply_stage_complex test ===\n");
  printf("Stage: %s, gain=%.1f dB, NF=%.1f dB\n", stg.name, stg.gain_db, stg.nf_db);
  printf("Expected Gain_total=%.1f, got %.1f\n", g_lin, Gain_total);
  printf("Output SNR (Friis): %.2f dB\n", m.snr_db);
  printf("Output ref power: %.6f (expect ~%.1f)\n", m.signal_power, g_lin);
  printf("Output noise power: %.6e\n", m.noise_power);

  int ok = 1;
  if (fabs(Gain_total - g_lin) / g_lin > 0.001) {
    printf("FAIL: Gain_total mismatch\n");
    ok = 0;
  }
  if (m.noise_power <= 0.0) {
    printf("FAIL: no noise added\n");
    ok = 0;
  }
  if (m.snr_db < 50.0 || m.snr_db > 100.0) {
    printf("WARN: SNR %.2f dB outside expected range [50, 100]\n", m.snr_db);
  }

  free(ref);
  free(sig);
  return ok;
}

static int test_real_stage(void) {
  PrngState rng;
  prng_init(&rng, 99);

  StageModel stg;
  memset(&stg, 0, sizeof(stg));
  stg.name = "TestMixer";
  stg.gain_db = 10.0;
  stg.nf_db = 5.0;
  stg.filter_len = 1;
  stg.is_limiter = 0;
  stg.p1db_dbm = INFINITY;
  stg.ip3_dbm = INFINITY;

  double *ref = (double *)calloc(N_SAMPLES, sizeof(double));
  double *sig = (double *)calloc(N_SAMPLES, sizeof(double));

  for (int i = 0; i < N_SAMPLES; i++) {
    ref[i] = 0.01;
    sig[i] = 0.01;
  }

  double N_t0_W = K_BOLTZMANN * 290.0 * B_NOISE_HZ;
  double N_current = N_t0_W;
  double Gain_total = 1.0;
  double P_sig_in = 0.0001;
  double fs_hz = 96.0e9;

  StageMetric m = apply_stage_realistic(&stg, ref, sig, N_SAMPLES,
                                        "rf_test", N_t0_W,
                                        fs_hz, 24.0e9,
                                        &N_current, &Gain_total,
                                        P_sig_in, &rng);

  double g_lin = db_to_lin_power(10.0);

  printf("\n=== apply_stage_realistic test ===\n");
  printf("Stage: %s, gain=%.1f dB, NF=%.1f dB\n", stg.name, stg.gain_db, stg.nf_db);
  printf("Expected Gain_total=%.1f, got %.1f\n", g_lin, Gain_total);
  printf("Output SNR: %.2f dB\n", m.snr_db);
  printf("Output noise power: %.6e\n", m.noise_power);

  int ok = 1;
  if (fabs(Gain_total - g_lin) / g_lin > 0.001) {
    printf("FAIL: Gain_total mismatch\n");
    ok = 0;
  }
  if (m.noise_power <= 0.0) {
    printf("FAIL: no noise added\n");
    ok = 0;
  }
  if (isnan(m.evm_pct)) {
    printf("OK: EVM is NaN for real signal (expected)\n");
  }

  free(ref);
  free(sig);
  return ok;
}

int main(void) {
  int r1 = test_complex_stage();
  int r2 = test_real_stage();

  printf("\n=== SUMMARY ===\n");
  printf("apply_stage_complex:  %s\n", r1 ? "PASS" : "FAIL");
  printf("apply_stage_realistic: %s\n", r2 ? "PASS" : "FAIL");

  return (r1 && r2) ? 0 : 1;
}
