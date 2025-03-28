#ifndef DMA_UTILS_H
#define DMA_UTILS_H

#include "network.h"


uint64_t getopt_integer(char *optarg);

ssize_t read_to_buffer(char *fname, int fd, char *buffer, uint64_t size,
			uint64_t base);

ssize_t write_from_buffer(char *fname, int fd, char *buffer, uint64_t size,
			uint64_t base);


static int timespec_check(struct timespec *t);      
void timespec_sub(struct timespec *t1, struct timespec *t2);


int NPU_Run_layer(network* net, NPU_LayerConfig *cur_layer);
#endif