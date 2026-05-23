#ifndef RF_SETUP_H
#define RF_SETUP_H

#include "sim_types.h"
#include "stage_models.h"

/* Working buffers — SoA layout for large Complex arrays */
typedef struct RfSimBuffers {
  double *env_re;        /* Upsampled envelope real part */
  double *env_im;        /* Upsampled envelope imag part */
  double *rf_ref;        /* Reference RF waveform (clean) */
  double *rf_sig;        /* Received RF waveform (noisy) */
  double *bb_ref_re;     /* Baseband reference real part */
  double *bb_ref_im;     /* Baseband reference imag part */
  double *bb_sig_re;     /* Baseband signal real part */
  double *bb_sig_im;     /* Baseband signal imag part */
  Complex *ref_sym;      /* Small: stays as Complex */
  Complex *sig_sym;      /* Small: stays as Complex */
  double *temp_bb_ref_re;
  double *temp_bb_ref_im;
  double *temp_bb_sig_re;
  double *temp_bb_sig_im;
  Complex *temp_ref_sym;
  Complex *temp_sig_sym;
  Complex *temp_complex_buf;
  double *i_raw;
  double *q_raw;
} RfSimBuffers;

typedef struct RfSimParams {
  int sps;
  double fs_hz;
  size_t nrf;
  size_t nbb;
  size_t bb_sps;
  size_t dec_factor;
  size_t pulse_len;
  
  const StageModel *rf_stages;
  const StageModel *bb_stages;
  size_t rf_stage_count;
  size_t bb_stage_count;
} RfSimParams;

/* Initialize derived RF parameters */
int rf_setup_compute_params(RfSimParams *params, const SimConfig *cfg, const StageModelsConfig *stage_cfg, size_t nsym);

/* Allocate all required simulation buffers */
int rf_setup_allocate(RfSimBuffers *bufs, const SimConfig *cfg, const StageModelsConfig *stage_cfg, size_t nsym);

/* Free all buffers inside RfSimBuffers */
void rf_setup_free(RfSimBuffers *bufs);

/* Build the initial transmitter RF envelope */
int rf_setup_build_envelope(RfSimBuffers *bufs, const Complex *tx_symbols, size_t nsym, const SimConfig *cfg, const RfSimParams *params);

/* RRC helpers */
double rrc_tap_value(double t, double rolloff);
void rrc_build_pulse(double rolloff, int sps, int span, double *pulse_out);
void rrc_apply_pulse(const Complex *symbols, size_t nsym, const double *pulse, int pulse_len, int sps, double *env_re, double *env_im);

#endif /* RF_SETUP_H */
