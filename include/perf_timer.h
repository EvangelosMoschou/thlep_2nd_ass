#ifndef PERF_TIMER_H
#define PERF_TIMER_H

#include <time.h>
#include <stddef.h>

/**
 * @file perf_timer.h
 * @brief High-precision performance timing infrastructure for benchmarking.
 *
 * This module provides wall-clock time measurement using CLOCK_MONOTONIC.
 *
 * --- Educational Note on Benchmarking Times ---
 * 1. Wall-Clock Time (Real Time):
 *    The actual elapsed physical time between two events. It includes time spent
 *    waiting for I/O, other processes, or thread context switches. Measured here
 *    via clock_gettime(CLOCK_MONOTONIC).
 * 2. CPU Time:
 *    The total time the CPU spent executing instructions for this process.
 *    Divided into User CPU time (app code) and System CPU time (OS kernel calls).
 *    If an app is multi-threaded (e.g. OpenMP), CPU time accumulates across
 *    all threads, meaning CPU time can be much larger than wall-clock time.
 * 3. Why clock() is unsuitable:
 *    The standard C clock() function measures total CPU time used by the process
 *    (scaled by CLOCKS_PER_SEC). In multi-threaded programs, it counts all cores
 *    separately and sums them up, making it useless for measuring real-world
 *    elapsed wall-clock duration of parallel regions. Furthermore, CLOCK_MONOTONIC
 *    is guaranteed to be non-decreasing and immune to system clock adjustments
 *    (unlike CLOCK_REALTIME which can jump forward or backward due to NTP sync).
 */

typedef struct {
    struct timespec start;
    struct timespec stop;
} PerfTimer;

/*
 * perf_timer_start - Start the timer.
 */
void perf_timer_start(PerfTimer *timer);

/*
 * perf_timer_stop - Stop the timer.
 */
void perf_timer_stop(PerfTimer *timer);

/*
 * perf_timer_elapsed_ms - Returns elapsed time in milliseconds.
 */
double perf_timer_elapsed_ms(const PerfTimer *timer);

/*
 * perf_timer_report - Print formatted throughput and timing statistics.
 */
void perf_timer_report(const char *path_name, const PerfTimer *timer, size_t num_symbols);

#endif /* PERF_TIMER_H */
