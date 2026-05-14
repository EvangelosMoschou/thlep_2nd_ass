#ifndef OUTPUT_MGR_H
#define OUTPUT_MGR_H

#include <stddef.h>

#define TOPOLOGY_SIM_COUNT 4
#define OUTPUT_ROOT_DIR "out"

int ensure_output_dirs(const char* root, int slot_id);
int clean_output_dir(const char* path);

#endif
