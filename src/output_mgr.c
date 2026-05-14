#include "output_mgr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>

int clean_output_dir(const char *path) {
    DIR *d = opendir(path);
    if (!d) return 0; // Not exist is fine
    struct dirent *dir;
    char buf[512];
    while ((dir = readdir(d)) != NULL) {
        if (dir->d_type == DT_REG) {
            snprintf(buf, sizeof(buf), "%s/%s", path, dir->d_name);
            unlink(buf);
        }
    }
    closedir(d);
    return 0;
}

int get_run_dir(char *buf, size_t n, const char *base, int topology_id, const char *path_type) {
    return snprintf(buf, n, "%s/%s_%d", base, path_type, topology_id);
}
