#ifndef PARSER_H
#define PARSER_H
#include "network.h"

//#ifdef NPU
#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <dirent.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
//#endif

#ifdef __cplusplus
extern "C" {
#endif
network parse_network_cfg(char *filename);
network parse_network_cfg_custom(char *filename, int batch, int time_steps);
void save_network(network net, char *filename);
void save_weights(network net, char *filename);
void save_weights_upto(network net, char *filename, int cutoff, int save_ema);
void save_weights_double(network net, char *filename);
void load_weights(network *net, char *filename);
void load_weights_upto(network *net, char *filename, int cutoff);


void parse_network_cfg_npu(network* net, char *filename);
#ifdef __cplusplus
}
#endif
#endif
