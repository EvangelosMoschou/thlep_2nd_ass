#ifndef STAGE_ARTIFACTS_H
#define STAGE_ARTIFACTS_H

#include <stddef.h>

#include "sim_types.h"

void humanize_stage_name(const char* raw, char* out, size_t out_size);

int write_metrics_csv(const char* path, const StageMetric* metrics, size_t count);
int write_input_budget_csv(const char* path, const SimConfig* cfg, double noise_w, double noise_dbm, double signal_dbm, double bandwidth_hz);
int write_metrics_svg(const char* path, const StageMetric* metrics, size_t count, const char* title);
int write_budget_svg(const char* path, const SimConfig* cfg, double noise_w, double noise_dbm, double signal_dbm, double bandwidth_hz);

void write_constellation_stage_artifacts(
    const char* csv_run_dir,
    const char* svg_run_dir,
    const char* file_prefix,
    size_t stage_number,
    int is_input,
    const char* raw_stage_name,
    const StageMetric* metric,
    const char* title_prefix,
    const Complex* constellation_template,
    size_t constellation_count,
    const Complex* ref,
    const Complex* sig,
    size_t nsym);

void write_complex_trace_stage_artifacts(
    const char* svg_run_dir,
    const char* file_prefix,
    size_t stage_number,
    int is_input,
    const char* raw_stage_name,
    const StageMetric* metric,
    const Complex* sig,
    size_t nsym,
    double fs_hz);

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
    double fs_hz);

int write_chain_architecture_mermaid(
    const char* path,
    const void* cfg_ptr,
    double final_snr_db,
    double target_vpp,
    double input_snr_db);

#endif
