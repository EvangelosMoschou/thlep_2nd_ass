#ifndef CLI_ARGS_H
#define CLI_ARGS_H

#include "sim_types.h"

int parse_cli_args(int argc, char** argv, SimConfig* cfg, char* stage_csv_path, size_t csv_path_size, int* topology_sim_id);
void print_usage_cli(const char* exe);

#endif
