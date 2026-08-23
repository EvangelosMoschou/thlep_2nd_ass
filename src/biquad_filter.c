#include "biquad_filter.h"
#include "math_utils.h"

#include <string.h>

/*
 * Butterworth Q values for cascaded biquad sections.
 * Index 0: 2nd order (1 section)
 * Index 1-2: 4th order (2 sections)
 * Index 0-2: 6th order (3 sections)
 */
static const double butterworth_q[3][3] = {
    { 0.7071067811865476, 0.0, 0.0 },                        /* order 2 */
    { 0.5411961001461969, 1.3065629648763766, 0.0 },         /* order 4 */
    { 0.5176380902050415, 0.7071067811865476, 1.9318516525781366 }, /* order 6 */
};

/* Maximum samples processed per chunk.  Stack scratch buffers are sized to
 * this; arbitrarily large n is handled by iterating over chunks. */
#define BQ_CHUNK 4096

static void compute_section_coeffs(BiquadCoeffs *c, double w0, double Q)
{
    double cos_w0 = cos(w0);
    double sin_w0 = sin(w0);
    double alpha = sin_w0 / (2.0 * Q);

    c->b0 = (1.0 - cos_w0) / 2.0;
    c->b1 = 1.0 - cos_w0;
    c->b2 = (1.0 - cos_w0) / 2.0;
    c->a1 = -2.0 * cos_w0;
    c->a2 = 1.0 - alpha;

    double a0 = 1.0 + alpha;
    c->b0 /= a0;
    c->b1 /= a0;
    c->b2 /= a0;
    c->a1 /= a0;
    c->a2 /= a0;
}

int biquad_init(BiquadState *state, const BiquadConfig *cfg)
{
    if (!state || !cfg) {
        return -1;
    }
    if (cfg->cutoff_hz <= 0.0 || cfg->fs_hz <= 0.0) {
        return -1;
    }
    if (cfg->cutoff_hz >= cfg->fs_hz / 2.0) {
        return -1;
    }

    int order_idx;
    switch (cfg->order) {
    case 2: order_idx = 0; break;
    case 4: order_idx = 1; break;
    case 6: order_idx = 2; break;
    default: return -1;
    }

    state->n_sections = cfg->order / 2;
    double w0 = 2.0 * M_PI * cfg->cutoff_hz / cfg->fs_hz;

    for (int i = 0; i < state->n_sections; i++) {
        compute_section_coeffs(&state->coeffs[i], w0, butterworth_q[order_idx][i]);
    }

    biquad_reset(state);
    return 0;
}

void biquad_reset(BiquadState *state)
{
    if (!state) {
        return;
    }
    for (int i = 0; i < state->n_sections; i++) {
        state->sections[i].x1 = 0.0;
        state->sections[i].x2 = 0.0;
        state->sections[i].y1 = 0.0;
        state->sections[i].y2 = 0.0;
    }
}

static void process_section_real(const BiquadCoeffs *c,
                                 BiquadSectionState *s,
                                 const double *in, double *out, size_t n)
{
    double x1 = s->x1, x2 = s->x2;
    double y1 = s->y1, y2 = s->y2;
    double b0 = c->b0, b1 = c->b1, b2 = c->b2;
    double a1 = c->a1, a2 = c->a2;

    for (size_t i = 0; i < n; i++) {
        double xn = in[i];
        double yn = b0 * xn + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        out[i] = yn;
        x2 = x1;
        x1 = xn;
        y2 = y1;
        y1 = yn;
    }

    s->x1 = x1; s->x2 = x2;
    s->y1 = y1; s->y2 = y2;
}

/*
 * bq_stream: Chunked stream processor shared by all public entry points.
 *
 * Iterates over the input in fixed chunks of at most BQ_CHUNK samples.  For
 * each chunk the input is FIRST copied into a local scratch buffer, so the
 * caller's in/out buffers may alias arbitrarily (in-place operation included)
 * without any overlapping-copy or read-after-write hazard.  The section
 * cascade then runs into the destination chunk, using two alternating local
 * scratch arrays for intermediate sections so consecutive sections never
 * share a buffer.  Section delay-line state lives in BiquadState and persists
 * across chunk boundaries, making the per-sample arithmetic identical to a
 * single-pass (non-chunked) reference for any n.
 */
static void bq_stream(BiquadState *state, const double *in, double *out, size_t n)
{
    if (state->n_sections == 0) {
        if (in != out) {
            memcpy(out, in, n * sizeof(double));
        }
        return;
    }

    double scratch[BQ_CHUNK];   /* per-chunk input copy (aliasing-safe) */
    double tmp_a[BQ_CHUNK];     /* intermediate section output */
    double tmp_b[BQ_CHUNK];     /* alternating intermediate section output */

    size_t offset = 0;
    while (offset < n) {
        size_t chunk = n - offset;
        if (chunk > BQ_CHUNK) {
            chunk = BQ_CHUNK;
        }

        /* Copy the input chunk first: safe even when in == out. */
        memcpy(scratch, in + offset, chunk * sizeof(double));

        const double *src = scratch;
        for (int sec = 0; sec < state->n_sections; sec++) {
            int is_last = (sec == state->n_sections - 1);
            double *dst = is_last ? out + offset : ((sec & 1) ? tmp_b : tmp_a);

            process_section_real(&state->coeffs[sec],
                                 &state->sections[sec],
                                 src, dst, chunk);

            src = dst; /* next section consumes this section's output */
        }
        offset += chunk;
    }
}

void biquad_process_real(BiquadState *state, const double *in, double *out, size_t n)
{
    if (!state || !in || !out || n == 0) {
        return;
    }
    bq_stream(state, in, out, n);
}

void biquad_process_complex(BiquadState *state, const Complex *in, Complex *out, size_t n)
{
    if (!state || !in || !out || n == 0) {
        return;
    }

    if (state->n_sections == 0) {
        if (in != out) {
            memcpy(out, in, n * sizeof(Complex));
        }
        return;
    }

    /* Complex LTI filtering equals identical real filtering per component,
     * so filter the I and Q streams with two identically-initialized copies
     * of the state (independent delay lines per component). */
    BiquadState st_re = *state;
    BiquadState st_im = *state;

    double re_in[BQ_CHUNK], im_in[BQ_CHUNK];
    double re_out[BQ_CHUNK], im_out[BQ_CHUNK];

    size_t offset = 0;
    while (offset < n) {
        size_t chunk = n - offset;
        if (chunk > BQ_CHUNK) {
            chunk = BQ_CHUNK;
        }

        for (size_t j = 0; j < chunk; j++) {
            re_in[j] = in[offset + j].re;
            im_in[j] = in[offset + j].im;
        }

        bq_stream(&st_re, re_in, re_out, chunk);
        bq_stream(&st_im, im_in, im_out, chunk);

        for (size_t j = 0; j < chunk; j++) {
            out[offset + j].re = re_out[j];
            out[offset + j].im = im_out[j];
        }
        offset += chunk;
    }

    /* Persist the I-component delay-line state (the caller's struct can hold
     * only one set of sections; coefficients are untouched). */
    for (int i = 0; i < state->n_sections; i++) {
        state->sections[i] = st_re.sections[i];
    }
}

void biquad_process_soa(BiquadState *state,
                        const double *in_re, const double *in_im,
                        double *out_re, double *out_im,
                        size_t n)
{
    if (!state || !in_re || !in_im || !out_re || !out_im || n == 0) {
        return;
    }

    if (state->n_sections == 0) {
        if (in_re != out_re) {
            memcpy(out_re, in_re, n * sizeof(double));
        }
        if (in_im != out_im) {
            memcpy(out_im, in_im, n * sizeof(double));
        }
        return;
    }

    /* Filter the I and Q streams independently with identically-initialized
     * states.  No Complex* casting of the SoA arrays is ever performed; the
     * per-chunk input copies make in-place operation (in_re == out_re,
     * in_im == out_im) safe for any n. */
    BiquadState st_re = *state;
    BiquadState st_im = *state;

    double re_in[BQ_CHUNK], im_in[BQ_CHUNK];
    double re_out[BQ_CHUNK], im_out[BQ_CHUNK];

    size_t offset = 0;
    while (offset < n) {
        size_t chunk = n - offset;
        if (chunk > BQ_CHUNK) {
            chunk = BQ_CHUNK;
        }

        for (size_t j = 0; j < chunk; j++) {
            re_in[j] = in_re[offset + j];
            im_in[j] = in_im[offset + j];
        }

        bq_stream(&st_re, re_in, re_out, chunk);
        bq_stream(&st_im, im_in, im_out, chunk);

        for (size_t j = 0; j < chunk; j++) {
            out_re[offset + j] = re_out[j];
            out_im[offset + j] = im_out[j];
        }
        offset += chunk;
    }

    for (int i = 0; i < state->n_sections; i++) {
        state->sections[i] = st_re.sections[i];
    }
}