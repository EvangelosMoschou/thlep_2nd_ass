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

#endif /* OUTPUT_MGR_H */
