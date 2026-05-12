#ifndef OUTPUT_MGR_H
#define OUTPUT_MGR_H

#include <stddef.h> /* for size_t */

/*
 * clean_output_dir — Delete all files inside a directory (but not the
 * directory itself)
 *
 * Parameters:
 *   path — Directory to clear (e.g., "out/csv")
 *
 * Returns:
 *   0 on success, negative on failure
 */
int clean_output_dir(const char *path);

/*
 * ensure_output_dirs — Create the complete output directory hierarchy
 *
 * Parameters:
 *   base         — Root directory path (e.g., "out")
 *   topology_id  — Simulation slot identifier (reserved for future per-slot
 *                  directory support)
 *
 * What it does:
 *   Creates the root directory and all subdirectories needed for CSV,
 *   constellation, trace, RF baseline, and realistic simulation outputs.
 *
 * Returns:
 *   0 on success, negative on failure
 *   (the specific negative value indicates which mkdir step failed)
 */
int ensure_output_dirs(const char *base, int topology_id);

/*
 * get_run_dir — Build a subdirectory path under the output root
 *
 * Parameters:
 *   buf          — Output buffer for the constructed path
 *   n            — Size of the output buffer
 *   base         — Root directory path (e.g., "out")
 *   topology_id  — Simulation slot identifier (1-based)
 *   path_type    — Subdirectory type (e.g., "csv", "constellations",
 *                  "traces")
 *
 * Returns:
 *   Number of bytes written (excluding NUL) on success,
 *   -1 on truncation or snprintf error
 */
int get_run_dir(char *buf, size_t n, const char *base, int topology_id,
                const char *path_type);

#endif /* OUTPUT_MGR_H */
