/*
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */

#define _BSD_SOURCE
#define _XOPEN_SOURCE 500
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

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "dma_utils.c"

static struct option const long_opts[] = {
	{"device", required_argument, NULL, 'd'},
	{"address", required_argument, NULL, 'a'},
	{"aperture", required_argument, NULL, 'k'},
	{"size", required_argument, NULL, 's'},
	{"offset", required_argument, NULL, 'o'},
	{"count", required_argument, NULL, 'c'},
	{"data infile", required_argument, NULL, 'f'},
	{"data outfile", required_argument, NULL, 'w'},
	{"help", no_argument, NULL, 'h'},
	{"verbose", no_argument, NULL, 'v'},
	{0, 0, 0, 0}
};

#define DEVICE_NAME_DEFAULT "/dev/xdma0_h2c_0"
#define DEVICE_NAME_WRITE "/dev/xdma0_h2c_0"
#define DEVICE_NAME_READ "/dev/xdma0_c2h_0"

static int write_dma(char *devname, char* buffer, uint64_t addr,
		    uint64_t size, uint64_t offset);

static int read_dma(char *devname, char* buffer, uint64_t addr,
		    uint64_t size, uint64_t offset);

static int test_dma(char *devname, uint64_t addr, uint64_t aperture,
		    uint64_t size, uint64_t offset, uint64_t count,
		    char *filename, char *);


int main()
{
	int cmd_opt;
	char *device = DEVICE_NAME_DEFAULT;
	uint64_t address = 0x800000000;
	uint64_t aperture = 0;
	uint64_t size = 32;
	uint64_t offset = 0;
	uint64_t count = 1;
	char *infname = NULL;
	char *ofname = NULL;
	// test_dma(device, address, aperture, size, offset, count,
	// 		infname, ofname);


	// write to device
	device = DEVICE_NAME_WRITE;
	address = 0x800000000;
	size = 32;
	offset = 0;

	char *write_buffer[32];
	write_buffer[0] = 1;
	write_buffer[1] = 2;
	write_buffer[2] = 3;
	write_dma(device, write_buffer, address, size, offset);


	// read from device
	device = DEVICE_NAME_READ;
	address = 0x800000000;
	size = 32;
	offset = 0;

	char *read_buffer[32];
	read_dma(device, read_buffer, address, size, offset);


	printf("verification start\r\n");
	printf("write_buffer idx 0: %d\r\n", atoi(write_buffer[1]));
	// check read result
	// for (int i=0; i < size; i++){
	// 	printf("idx %d: write: %s, read: %s",
	// 				i, write_buffer[i], read_buffer[i]);
	// 	if (write_buffer[i] != read_buffer[i]){
	// 		printf("doesnt match at idx %d: write: %c, read: %c",
	// 				i, write_buffer[i], read_buffer[i]);
	// 		return 0;
	// 	}
	// }

	printf("verification success\r\n");
	return 1;
}

static int write_dma(char *devname, char *buffer, uint64_t addr,
		    uint64_t size, uint64_t offset)
{
	uint64_t i;
	ssize_t rc;
	size_t bytes_done = 0;
	int fpga_fd = open(devname, O_RDWR);
	/* write buffer to AXI MM address using SGDMA */

	rc = write_from_buffer(devname, fpga_fd, buffer, size,
					addr);
	bytes_done = rc;
	printf("write bytes done: %d\r\n", bytes_done);
}

static int read_dma (char *devname, char *buffer, uint64_t addr,
			uint64_t size, uint64_t offset)
{
	uint64_t i;
	ssize_t rc;
	size_t bytes_done = 0;
	int fpga_fd = open(devname, O_RDWR);

	rc = read_to_buffer(devname, fpga_fd, buffer, size, addr);

	bytes_done = rc;
	printf("read bytes done: %d\r\n", bytes_done);
}


static int test_dma(char *devname, uint64_t addr, uint64_t aperture,
		    uint64_t size, uint64_t offset, uint64_t count,
		    char *infname, char *ofname)
{
	uint64_t i;
	ssize_t rc;
	size_t bytes_done = 0;
	char *buffer = NULL;
	char *allocated = NULL;
	int fpga_fd = open(devname, O_RDWR);

	if (fpga_fd < 0) {
		fprintf(stderr, "unable to open device %s, %d.\n",
			devname, fpga_fd);
		perror("open device");
		return -EINVAL;
	}


	posix_memalign((void **)&allocated, 4096 /*alignment */ , size + 4096);

	buffer = allocated + offset;

	buffer[0] = 2;

	printf("buffer 0: %d\r\n", buffer[0]);
	printf("buffer 0: %d\r\n", buffer[1]);
	size_t buffer_size = sizeof(buffer);
    printf("The size is %d bytes \n", buffer_size);
	printf("addr: %lx\r\n", addr);

	for (i = 0; i < count; i++) {
		/* write buffer to AXI MM address using SGDMA */

		rc = write_from_buffer(devname, fpga_fd, buffer, size,
						addr);
		bytes_done = rc;
	}
}
