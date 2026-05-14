#ifndef OUTPUT_MGR_H
#define OUTPUT_MGR_H

#include <stddef.h>

/**
 * clean_output_dir — Remove all files in a directory (non-recursive)
 *
 * Returns:
 *   0 on success
 *   -1 on error (e.g. directory not found)
 */
int clean_output_dir(const char *path);

/**
 * get_run_dir — Construct a path string for a specific simulation run
 */
int get_run_dir(char *buf, size_t n, const char *base, int topology_id,
                const char *path_type);

#endif
