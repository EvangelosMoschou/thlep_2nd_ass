#ifndef METRICS_H
#define METRICS_H

#include "sim_types.h"

StageMetric compute_metric_complex(const char* stage_name, const char* domain, const Complex* ref, const Complex* sig, size_t n);
StageMetric compute_metric_real(const char* stage_name, const char* domain, const double* ref, const double* sig, size_t n);

#endif
