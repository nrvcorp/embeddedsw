/*
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "dma_utils.h"

/*
 * man 2 write:
 * On Linux, write() (and similar system calls) will transfer at most
 * 	0x7ffff000 (2,147,479,552) bytes, returning the number of bytes
 *	actually transferred.  (This is true on both 32-bit and 64-bit
 *	systems.)
 */

#define RW_MAX_SIZE	0x7ffff000

int verbose = 0;

uint64_t getopt_integer(char *optarg)
{
	int rc;
	uint64_t value;

	rc = sscanf(optarg, "0x%lx", &value);
	if (rc <= 0)
		rc = sscanf(optarg, "%lu", &value);
	//printf("sscanf() = %d, value = 0x%lx\n", rc, value);

	return value;
}

ssize_t read_to_buffer(char *fname, int fd, char *buffer, uint64_t size,
			uint64_t base)
{
	ssize_t rc;
	uint64_t count = 0;
	char *buf = buffer;
	off_t offset = base;
	int loop = 0;

	while (count < size) {
		uint64_t bytes = size - count;

		if (bytes > RW_MAX_SIZE)
			bytes = RW_MAX_SIZE;

		if (offset) {
			rc = lseek(fd, offset, SEEK_SET);
			if (rc != offset) {
				fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n",
					fname, rc, offset);
				perror("seek file");
				return -EIO;
			}
		}

		/* read data from file into memory buffer */
		rc = read(fd, buf, bytes);
		if (rc < 0) {
			fprintf(stderr, "%s, read 0x%lx @ 0x%lx failed %ld.\n",
				fname, bytes, offset, rc);
			perror("read file");
			return -EIO;
		}

		count += rc;
		if (rc != bytes) {
			fprintf(stderr, "%s, read underflow 0x%lx/0x%lx @ 0x%lx.\n",
				fname, rc, bytes, offset);
			break;
		}

		buf += bytes;
		offset += bytes;
		loop++;
	}

	if (count != size && loop)
		fprintf(stderr, "%s, read underflow 0x%lx/0x%lx.\n",
			fname, count, size);
	return count;
}

ssize_t write_from_buffer(char *fname, int fd, char *buffer, uint64_t size,
			uint64_t base)
{
	ssize_t rc;
	uint64_t count = 0;
	char *buf = buffer;
	off_t offset = base;
	int loop = 0;

	while (count < size) {
		uint64_t bytes = size - count;

		if (bytes > RW_MAX_SIZE)
			bytes = RW_MAX_SIZE;

		if (offset) {
			rc = lseek(fd, offset, SEEK_SET);
			if (rc != offset) {
				fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n",
					fname, rc, offset);
				perror("seek file");
				return -EIO;
			}
		}

		/* write data to file from memory buffer */
		rc = write(fd, buf, bytes);
		if (rc < 0) {
			fprintf(stderr, "%s, write 0x%lx @ 0x%lx failed %ld.\n",
				fname, bytes, offset, rc);
			perror("write file");
			return -EIO;
		}

		count += rc;
		if (rc != bytes) {
			fprintf(stderr, "%s, write underflow 0x%lx/0x%lx @ 0x%lx.\n",
				fname, rc, bytes, offset);
			break;
		}
		buf += bytes;
		offset += bytes;

		loop++;
	}	

	if (count != size && loop)
		fprintf(stderr, "%s, write underflow 0x%lx/0x%lx.\n",
			fname, count, size);

	return count;
}


/* Subtract timespec t2 from t1
 *
 * Both t1 and t2 must already be normalized
 * i.e. 0 <= nsec < 1000000000
 */
static int timespec_check(struct timespec *t)
{
	if ((t->tv_nsec < 0) || (t->tv_nsec >= 1000000000))
		return -1;
	return 0;

}

void timespec_sub(struct timespec *t1, struct timespec *t2)
{
	if (timespec_check(t1) < 0) {
		fprintf(stderr, "invalid time #1: %lld.%.9ld.\n",
			(long long)t1->tv_sec, t1->tv_nsec);
		return;
	}
	if (timespec_check(t2) < 0) {
		fprintf(stderr, "invalid time #2: %lld.%.9ld.\n",
			(long long)t2->tv_sec, t2->tv_nsec);
		return;
	}
	t1->tv_sec -= t2->tv_sec;
	t1->tv_nsec -= t2->tv_nsec;
	if (t1->tv_nsec >= 1000000000) {
		t1->tv_sec++;
		t1->tv_nsec -= 1000000000;
	} else if (t1->tv_nsec < 0) {
		t1->tv_sec--;
		t1->tv_nsec += 1000000000;
	}
}

int NPU_Run_layer(network* net, NPU_LayerConfig *cur_layer)
{
	*((uint32_t *) (net->user_base + AXILITE_CONV_MODE               )) = cur_layer->conv_mode;
	*((uint32_t *) (net->user_base + AXILITE_IFM_CASE                )) = cur_layer->ifm_case;
	*((uint32_t *) (net->user_base + AXILITE_IFM_DTYPE               )) = cur_layer->ifm_dtype;
	*((uint32_t *) (net->user_base + AXILITE_IFM_IS_PADDING          )) = cur_layer->ifm_is_padding;
	*((uint32_t *) (net->user_base + AXILITE_IFM_IS_MAXPOOL          )) = cur_layer->ifm_is_maxpool;
	*((uint32_t *) (net->user_base + AXILITE_IFM_IS_LEAKY_RELU       )) = cur_layer->ifm_is_leaky_relu;
	*((uint32_t *) (net->user_base + AXILITE_IS_CONCAT               )) = cur_layer->is_concat;
	*((uint32_t *) (net->user_base + AXILITE_IS_UPSAMPLE       	     )) = cur_layer->is_upsample;

	*((uint64_t *) (net->user_base + AXILITE_IFM_BASEADDR            )) = cur_layer->ifm_baseaddr;
	*((uint64_t *) (net->user_base + AXILITE_WEIGHT_BASEADDR         )) = cur_layer->weight_baseaddr;
	*((uint64_t *) (net->user_base + AXILITE_OFM_BASEADDR            )) = cur_layer->ofm_baseaddr;
	*((uint32_t *) (net->user_base + AXILITE_IN_WIDTH                )) = cur_layer->in_width;
	*((uint32_t *) (net->user_base + AXILITE_IN_HEIGHT               )) = cur_layer->in_height;
	*((uint32_t *) (net->user_base + AXILITE_IN_CH                   )) = cur_layer->in_ch;
	*((uint32_t *) (net->user_base + AXILITE_OUT_CH                  )) = cur_layer->out_ch;
	*((uint32_t *) (net->user_base + AXILITE_POOL_STRIDE             )) = cur_layer->pool_stride;
	*((uint32_t *) (net->user_base + AXILITE_CONV_STRIDE             )) = cur_layer->conv_stride;
	*((uint32_t *) (net->user_base + AXILITE_SCALE_SHIFT             )) = cur_layer->scale_shift;
	*((uint32_t *) (net->user_base + AXILITE_BIAS_SHIFT              )) = cur_layer->bias_shift;
	*((uint32_t *) (net->user_base + AXILITE_SCALE_BIAS_1_NUM        )) = cur_layer->is_single_scale_bias;
	*((uint32_t *) (net->user_base + AXILITE_SINGLE_SCALE            )) = cur_layer->single_scale;
	*((uint32_t *) (net->user_base + AXILITE_SINGLE_BIAS             )) = cur_layer->single_bias;
	*((uint64_t *) (net->user_base + AXILITE_CONCAT_BASEADDR         )) = cur_layer->concat_baseaddr;
	*((uint32_t *) (net->user_base + AXILITE_CONCAT_CH       	     )) = cur_layer->concat_ch;
	// printf("current layer bias_baseaddr 1: %lx\r\n",cur_layer->bias_baseaddr);
	*((uint64_t *) (net->user_base + AXILITE_BIAS_BASEADDR           )) = cur_layer->bias_baseaddr;
	// printf("current layer bias_baseaddr 2: %lx\r\n", cur_layer->bias_baseaddr);
	msync(net->user_base + AXILITE_CONV_MODE, 128, MS_SYNC);
	// sleep(1);
	*((uint32_t *) (net->user_base + AXILITE_DATA_VSYNC              )) = (uint32_t)(1);
	usleep(1);
	return 0;
}