#include "perf_timer.h"
#include <stdio.h>

void perf_timer_start(PerfTimer *timer) {
    if (!timer) return;
    clock_gettime(CLOCK_MONOTONIC, &timer->start);
}

void perf_timer_stop(PerfTimer *timer) {
    if (!timer) return;
    clock_gettime(CLOCK_MONOTONIC, &timer->stop);
}

double perf_timer_elapsed_ms(const PerfTimer *timer) {
    if (!timer) return 0.0;
    double start_sec = (double)timer->start.tv_sec + (double)timer->start.tv_nsec * 1e-9;
    double stop_sec = (double)timer->stop.tv_sec + (double)timer->stop.tv_nsec * 1e-9;
    return (stop_sec - start_sec) * 1000.0;
}

void perf_timer_report(const char *path_name, const PerfTimer *timer, size_t num_symbols) {
    if (!timer || !path_name) return;
    double ms = perf_timer_elapsed_ms(timer);
    double sec = ms / 1000.0;
    double throughput = (sec > 0.0) ? ((double)num_symbols / sec) : 0.0;
    printf("  %-25s : %10.2f ms (%8.1f symbols/sec)\n", path_name, ms, throughput);
}
