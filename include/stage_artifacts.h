#ifndef STAGE_ARTIFACTS_H
#define STAGE_ARTIFACTS_H

#include <stddef.h>
#include "sim_types.h"
#include "metrics.h"

void write_constellation_artifact(
    const char* run_dir,
    const char* file_prefix,
    size_t stage_number,
    int is_input,
    const char* raw_stage_name,
    const StageMetric* metric,
    const Complex* constellation_template,
    size_t constellation_count,
    const Complex* ref,
    const Complex* sig,
    size_t nsym);

void write_trace_stage_artifacts(
    const char* csv_run_dir,
    const char* svg_run_dir,
    const char* file_prefix,
    size_t stage_number,
    int is_input,
    const char* raw_stage_name,
    const StageMetric* metric,
    const char* title_prefix,
    const double* ref,
    const double* sig,
    size_t n,
    size_t max_points,
    double fs_hz,
    double signal_hz);

#endif
