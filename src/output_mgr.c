#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "output_mgr.h"

/*
 * ensure_dir_exists — Internal helper to check or create a single directory
 *
 * Checks if the given path exists as a directory. If it does not, attempts
 * to create it with default permissions (0777 & ~umask).
 *
 * Returns 0 on success, -1 on failure.
 */
static int ensure_dir_exists(const char *path) {
  struct stat st;

  if (!path) {
    return -1;
  }

  if (stat(path, &st) == 0) {
    return S_ISDIR(st.st_mode) ? 0 : -1;
  }

  if (mkdir(path, 0777) != 0 && errno != EEXIST) {
    return -1;
  }

  return 0;
}

int clean_output_dir(const char *path) {
  DIR *dir;
  struct dirent *entry;
  char entry_path[1024];
  int written;

  if (!path) {
    return -1;
  }

  dir = opendir(path);
  if (!dir) {
    return -2;
  }

  while ((entry = readdir(dir)) != NULL) {
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }

    written =
        snprintf(entry_path, sizeof(entry_path), "%s/%s", path, entry->d_name);
    if (written < 0 || (size_t)written >= sizeof(entry_path)) {
      closedir(dir);
      return -3;
    }

    if (remove(entry_path) != 0) {
      closedir(dir);
      return -4;
    }
  }

  closedir(dir);
  return 0;
}

int ensure_output_dirs(const char *base, int topology_id) {
  char sub[1024];

  (void)topology_id; /* reserved for future per-slot directory support */

  if (ensure_dir_exists(base) != 0) {
    return -1;
  }

  /* RF baseline subdirectories */
  snprintf(sub, sizeof(sub), "%s/rf_baseline", base);
  if (ensure_dir_exists(sub) != 0) return -2;
  snprintf(sub, sizeof(sub), "%s/rf_baseline/Rx", base);
  if (ensure_dir_exists(sub) != 0) return -3;
  snprintf(sub, sizeof(sub), "%s/rf_baseline/Rx/csv", base);
  if (ensure_dir_exists(sub) != 0) return -4;
  snprintf(sub, sizeof(sub), "%s/rf_baseline/Rx/constellations", base);
  if (ensure_dir_exists(sub) != 0) return -5;
  snprintf(sub, sizeof(sub), "%s/rf_baseline/Rx/traces", base);
  if (ensure_dir_exists(sub) != 0) return -6;
  snprintf(sub, sizeof(sub), "%s/rf_baseline/Rx/spectrum", base);
  if (ensure_dir_exists(sub) != 0) return -7;

  /* Realistic impairment subdirectories */
  snprintf(sub, sizeof(sub), "%s/realistic", base);
  if (ensure_dir_exists(sub) != 0) return -8;
  snprintf(sub, sizeof(sub), "%s/realistic/Rx", base);
  if (ensure_dir_exists(sub) != 0) return -9;
  snprintf(sub, sizeof(sub), "%s/realistic/Rx/csv", base);
  if (ensure_dir_exists(sub) != 0) return -10;
  snprintf(sub, sizeof(sub), "%s/realistic/Rx/constellations", base);
  if (ensure_dir_exists(sub) != 0) return -11;
  snprintf(sub, sizeof(sub), "%s/realistic/Rx/traces", base);
  if (ensure_dir_exists(sub) != 0) return -12;
  snprintf(sub, sizeof(sub), "%s/realistic/Rx/spectrum", base);
  if (ensure_dir_exists(sub) != 0) return -13;
  snprintf(sub, sizeof(sub), "%s/realistic/Tx", base);
  if (ensure_dir_exists(sub) != 0) return -14;
  snprintf(sub, sizeof(sub), "%s/realistic/Tx/csv", base);
  if (ensure_dir_exists(sub) != 0) return -15;
  snprintf(sub, sizeof(sub), "%s/realistic/Tx/constellations", base);
  if (ensure_dir_exists(sub) != 0) return -16;
  snprintf(sub, sizeof(sub), "%s/realistic/Tx/traces", base);
  if (ensure_dir_exists(sub) != 0) return -17;
  snprintf(sub, sizeof(sub), "%s/realistic/Tx/spectrum", base);
  if (ensure_dir_exists(sub) != 0) return -18;

  return 0;
}

int get_run_dir(char *buf, size_t n, const char *base, int topology_id,
                const char *path_type) {
  int written = snprintf(buf, n, "%s/topology_sim_%d/%s", base, topology_id,
                         path_type);
  if (written < 0) return -1;
  if ((size_t)written >= n) return -1;
  return written;
}
