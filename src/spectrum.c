#include "spectrum.h"
#include "fft.h"
#include "stage_artifacts.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

void write_stage_spectrum(const char *dir, const char *prefix,
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
