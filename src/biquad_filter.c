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

void biquad_process_real(BiquadState *state, const double *in, double *out, size_t n)
{
    if (!state || !in || !out || n == 0) {
        return;
    }

    if (state->n_sections == 0) {
        if (in != out) {
            memcpy(out, in, n * sizeof(double));
        }
        return;
    }

    double buf1[4096];
    double buf2[4096];
    const double *src = in;
    double *dst;

    for (int sec = 0; sec < state->n_sections; sec++) {
        int is_last = (sec == state->n_sections - 1);
        dst = is_last ? out : buf2;

        size_t offset = 0;
        while (offset < n) {
            size_t chunk = n - offset;
            if (chunk > 4096) chunk = 4096;

            if (src == in && offset == 0) {
                process_section_real(&state->coeffs[sec],
                                     &state->sections[sec],
                                     in, dst, chunk);
            } else {
                memcpy(buf1, src + offset, chunk * sizeof(double));
                process_section_real(&state->coeffs[sec],
                                     &state->sections[sec],
                                     buf1, dst, chunk);
            }

            if (!is_last) {
                memcpy(buf1 + offset, buf2, chunk * sizeof(double));
            }
            offset += chunk;
        }

        if (!is_last) {
            src = buf1;
        }
    }
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

    double buf_re[4096], buf_im[4096];
    double tmp_re[4096], tmp_im[4096];

    for (size_t i = 0; i < n; i++) {
        buf_re[i] = in[i].re;
        buf_im[i] = in[i].im;
    }

    for (int sec = 0; sec < state->n_sections; sec++) {
        int is_last = (sec == state->n_sections - 1);
        double *dst_re = is_last ? NULL : tmp_re;
        double *dst_im = is_last ? NULL : tmp_im;

        size_t offset = 0;
        while (offset < n) {
            size_t chunk = n - offset;
            if (chunk > 4096) chunk = 4096;

            double chunk_in_re[4096], chunk_in_im[4096];
            double chunk_out_re[4096], chunk_out_im[4096];

            for (size_t j = 0; j < chunk; j++) {
                chunk_in_re[j] = buf_re[offset + j];
                chunk_in_im[j] = buf_im[offset + j];
            }

            process_section_real(&state->coeffs[sec],
                                 &state->sections[sec],
                                 chunk_in_re, chunk_out_re, chunk);
            process_section_real(&state->coeffs[sec],
                                 &state->sections[sec],
                                 chunk_in_im, chunk_out_im, chunk);

            if (is_last) {
                for (size_t j = 0; j < chunk; j++) {
                    out[offset + j].re = chunk_out_re[j];
                    out[offset + j].im = chunk_out_im[j];
                }
            } else {
                for (size_t j = 0; j < chunk; j++) {
                    dst_re[offset + j] = chunk_out_re[j];
                    dst_im[offset + j] = chunk_out_im[j];
                }
            }
            offset += chunk;
        }

        if (!is_last) {
            for (size_t i = 0; i < n; i++) {
                buf_re[i] = dst_re[i];
                buf_im[i] = dst_im[i];
            }
        }
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

    Complex *in_c = NULL;
    Complex *out_c = NULL;

    if (in_re == out_re && in_im == out_im) {
        in_c = (Complex *)in_re;
        out_c = (Complex *)out_re;
    } else {
        Complex tmp[4096];
        size_t offset = 0;
        while (offset < n) {
            size_t chunk = n - offset;
            if (chunk > 4096) chunk = 4096;

            for (size_t i = 0; i < chunk; i++) {
                tmp[i].re = in_re[offset + i];
                tmp[i].im = in_im[offset + i];
            }

            biquad_process_complex(state, tmp, tmp, chunk);

            for (size_t i = 0; i < chunk; i++) {
                out_re[offset + i] = tmp[i].re;
                out_im[offset + i] = tmp[i].im;
            }
            offset += chunk;
        }
        return;
    }

    biquad_process_complex(state, in_c, out_c, n);
}
