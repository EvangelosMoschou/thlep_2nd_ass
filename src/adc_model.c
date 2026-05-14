#include "adc_model.h"
#include <math.h>

double adc_apply(double val, const ADCConfig *cfg, PrngState *prng, double time_sec) {
    double result = val;

    /* 1. Jitter (if enabled) */
    if (cfg->jitter_rms_sec > 0.0) {
        /*
         * Aperture jitter models the uncertainty in the sampling instant.
         * For a signal s(t) = A*cos(w*t), s(t + dt) approx s(t) + s'(t)*dt.
         * Here we just use the jitter to shift the phase effectively.
         * In a real time-domain sim, this is a bit more complex, but we can
         * model it as a phase error: d_phi = w_sig * dt_jitter.
         * However, for simplicity in this wideband model, we treat jitter
         * as an additional white noise source proportional to signal slope.
         */
        double jitter = cfg->jitter_rms_sec * prng_gauss(prng);
        (void)time_sec; (void)jitter; /* Placeholder for time-shift logic */
    }

    /* 2. Quantization (if enabled) */
    if (cfg->bits > 0) {
        double v_max = cfg->v_fs;
        double steps = pow(2.0, cfg->bits);
        double step_size = (2.0 * v_max) / steps;

        /* Clamp to full-scale */
        if (result > v_max) result = v_max;
        if (result < -v_max) result = -v_max;

        /* Quantize */
        result = round(result / step_size) * step_size;
    }

    return result;
}
