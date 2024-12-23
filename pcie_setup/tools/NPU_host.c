/*
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under BSD-style license (found in the
 * LICENSE file in the root directory of this source tree)
 */
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

#define MAP_SIZE (32*1024UL)
#define DEVICE_NAME_DEFAULT "/dev/xdma0_h2c_0"
#define DEVICE_NAME_WRITE 	"/dev/xdma0_h2c_0"
#define DEVICE_NAME_READ 	"/dev/xdma0_c2h_0"
#define H2C_DEVICE			"/dev/xdma0_h2c_0"
#define C2H_DEVICE			"/dev/xdma0_c2h_0"
#define INPUT_FILENAME		"../DATASET/MaskYOLO/resized_example_test_mask_084.png"
#define OUTPUT_FILENAME 	"../DATASET/MaskYOLO/npu_out.png"

#define IFM_ADDR		0x800000000
#define IFM_READY_ADDR	0x810000000
#define OFM_ADDR		0x900000000
#define OFM_READY_ADRR	0x900000000

#include "dma_utils.c"

struct pcie_transfer {
	int infile_fd;
	int h2c_fd;
	int c2h_fd;
	int reg_fd;
	int ofile_fd;
	void *map_base;
	char *read_buffer;
	char *write_buffer;
	char *infname;
	char *ofname;
} trans;

static struct option const long_opts[] = {
	{"data infile", required_argument, NULL, 'i'},
	{"data outfile", required_argument, NULL, 'o'},
	{"help", no_argument, NULL, 'h'},
	{0, 0, 0, 0}
};

int test_run_app(char *h2c_device, char *c2h_device,
			char *infname, char *outfname);

int write_dma(char *devname, char* buffer, uint64_t addr,
		    uint64_t size, uint64_t offset);

int read_dma(char *devname, char* buffer, uint64_t addr,
		    uint64_t size, uint64_t offset);

int main(int argc, char *argv[])
{
	char *write_device = DEVICE_NAME_WRITE;
	char *read_device = DEVICE_NAME_READ;
	uint64_t address = 0x800000000;
	uint64_t aperture = 0;
	uint64_t size = 32;
	uint64_t offset = 0;
	uint64_t count = 1;

	int cmd_opt;
	char *h2c_device = H2C_DEVICE;
	char *c2h_device = C2H_DEVICE;
	char *outfname = OUTPUT_FILENAME;
	char *infname = INPUT_FILENAME;
	while ((cmd_opt = getopt_long(argc, argv, "vhc:i:o:h", long_opts, NULL)) != -1) {
		switch (cmd_opt) {
			case 0:
				/* long option */
				break;
			case 'i':
				infname = strdup(optarg);
				break;
			case 'o':
				outfname = strdup(optarg);
				break;
			default:
				exit(0);
				break;
		}
	}

	int result = test_run_app(h2c_device, c2h_device, infname, outfname);
	return result;
}


int test_run_app(char *h2c_device, char *c2h_device, char *infname, char *outfname)
{
	printf("************************************************\r\n");
	printf("*******                              ***********\r\n");
	printf("*******       CAPP NPU Application   ***********\r\n");
	printf("*******                              ***********\r\n");
	printf("************************************************\r\n");
	printf("infile name: %s\r\n", infname);
	printf("outfile name: %s\r\n", outfname);

	ssize_t rc;
	volatile unsigned int *infile_len;
	unsigned long int file_len;

	pthread_t thread1, thread2;

	/* check h2c functionality */
	trans.h2c_fd = open(h2c_device, O_RDWR);
	if (trans.h2c_fd < 0) {
		fprintf(stderr, "unable to open device %s, %d.\r\n", h2c_device, trans.h2c_fd);
		perror("open device");
		return -EINVAL;
	}

	/* check c2h functionality */
	trans.c2h_fd = open(c2h_device, O_RDWR);
	if (trans.c2h_fd < 0) {
		fprintf(stderr, "unable to open device %s, %d.\r\n", c2h_device, trans.c2h_fd);
		perror("open device");
		goto h2c_out;
	}

	/* open input file */
	trans.infname = infname;
	trans.infile_fd = open(infname, O_RDONLY);
	if (trans.infile_fd < 0) {
		fprintf(stderr, "unable to open input file %s, %d.\n",
			infname, trans.infile_fd);
		printf("open input file failed");
		rc = -EINVAL;
	} else {
		printf("open input file success: %s,\n", infname);
	}

	/* open or create output file */
	trans.ofname = outfname;
	trans.ofile_fd = open(outfname, O_RDWR | O_CREAT | O_TRUNC | O_SYNC,
			0666);
	if (trans.ofile_fd < 0) {
		fprintf(stderr, "unable to open input file %s, %d.\n",
			outfname, trans.ofile_fd);
		perror("open input file");
		rc = -EINVAL;
	}


	/* Get the input file length  */
	if (trans.infile_fd > 0) {
		file_len = lseek(trans.infile_fd, 0, SEEK_END);
		if (file_len == (off_t)-1)
		{
			printf("failed to lseek %s numbytes %lu\n", trans.infname, file_len);
			exit(EXIT_FAILURE);
		}
		printf("File length is :%d\n",file_len);

		/* reset the file position indicator to
		the beginning of the file */

		lseek(trans.infile_fd, 0L, SEEK_SET);
	}


c2h_out:
	close(trans.c2h_fd);

h2c_out:
	close(trans.h2c_fd);
	return rc;
}



int write_dma(char *devname, char *buffer, uint64_t addr,
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

int read_dma (char *devname, char *buffer, uint64_t addr,
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