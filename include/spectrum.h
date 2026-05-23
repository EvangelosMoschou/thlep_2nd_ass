#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <stddef.h>

/*
 * write_stage_spectrum - Generates spectrum SVG files for receiver stages.
 *
 * @dir:        Output directory path.
 * @prefix:     Filename prefix (e.g. "rf_baseline" or "realistic").
 * @num:        Stage number.
 * @name:       Name of the stage.
 * @signal:     Input real-valued time-domain samples.
 * @nsamp:      Number of samples.
 * @fs_hz:      Sample rate in Hz.
 * @center_hz:  Center frequency of display window.
 * @span_hz:    Span of display window (frequencies outside are filtered).
 */
void write_stage_spectrum(const char *dir, const char *prefix,
                          size_t num, const char *name,
                          const double *signal, size_t nsamp,
                          double fs_hz,
                          double center_hz, double span_hz);

#endif /* SPECTRUM_H */
