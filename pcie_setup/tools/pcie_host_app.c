/*
 * This file is part of the Xilinx DMA IP Core driver tools for Linux
 *
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under both the BSD-style license (found in the
 * LICENSE file in the root directory of this source tree) and the GPLv2 (found
 * in the COPYING file in the root directory of this source tree).
 * You may select, at your option, one of the above-listed licenses.
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
#include <pthread.h>

#include "dma_utils.c"
#define MAP_SIZE (32*1024UL)
#define MAP_MASK (MAP_SIZE - 1)

#define PCIRC_GET_FILE_LENGTH      0x04
#define PCIRC_READ_BUFFER_TRANSFER_DONE 0x2c
#define PCIRC_WRITE_BUFFER_TRANSFER_DONE 0x30
#define PCIRC_HOST_DONE			  0x34
#define PCIRC_ENC_PARAMS_1        0x10
#define PCIRC_ENC_PARAMS_2        0x14
#define PCIRC_RAW_RESOLUTION      0x18
#define PCIRC_GET_USECASE_TYPE    0x1c
#define PCIRC_ENC_PARAMS_3        0x20
#define PCIRC_ENC_PARAMS_4        0x24
#define PCIRC_ENC_PARAMS_5        0x28

#define PCIEP_READ_BUFFER_READY   0x3c
#define PCIEP_READ_BUFFER_ADDR   0x40
#define PCIEP_READ_BUFFER_OFFSET 0x44
#define PCIEP_READ_BUFFER_SIZE   0x48
#define PCIEP_WRITE_BUFFER_READY   0x4c
#define PCIEP_WRITE_BUFFER_ADDR   0x50
#define PCIEP_WRITE_BUFFER_OFFSET 0x54
#define PCIEP_WRITE_BUFFER_SIZE   0x58
#define PCIEP_READ_TRANSFER_COMPLETE   0x5c
#define PCIEP_WRITE_TRANSFER_COMPLETE  0x60
#define PCIEP_READ_TRANSFER_CLR  0x64
#define PCIEP_READ_BUFFER_HOST_INTR  0x68
#define PCIEP_WRITE_TRANSFER_CLR  0x6c

#define H2C_DEVICE "/dev/xdma0_h2c_0"
#define C2H_DEVICE "/dev/xdma0_c2h_0"
#define REG_DEVICE_NAME "/dev/xdma0_xvc"
#define OUTPUT_FILENAME "out.ts"
#define COUNT_DEFAULT 		  (1)
#define SIZE_DEFAULT		  (32)

/* Encoder parameter options */
#define L2_CACHE_ENABLE			 (1)
#define L2_CACHE_DISABLE		 (0)
#define LOW_BANDWIDTH_ENABLE	 (1)
#define LOW_BANDWIDTH_DISABLE	 (0)
#define FILLER_DATA_ENABLE		 (1)
#define FILLER_DATA_DISABLE		 (0)
#define BITRATE_MIN				 (1)
#define BITRATE_MAX				 (60000)
#define GOP_LENGTH_MIN			 (1)
#define GOP_LENGTH_MAX			 (1000)
#define B_FRAME_MIN				 (0)
#define B_FRAME_MAX				 (4)
#define SLICE_MIN				 (4)
#define SLICE_MAX				 (22)
#define QP_MODE_UNIFORM			 (0)
#define QP_MODE_AUTO			 (2)
#define RATE_CONTROL_MODE_VBR	 (1)
#define RATE_CONTROL_MODE_CBR	 (2)
#define ENC_TYPE_AVC			 (0)
#define ENC_TYPE_HEVC			 (1)
#define GOP_MODE_BASIC			 (0)
#define GOP_MODE_LOW_DELAY_P	 (3)
#define GOP_MODE_LOW_DELAY_B	 (4)
#define FPS_MIN					 (30)
#define FPS_MAX					 (60)
#define MIN_QP					 (10)
#define MAX_QP					 (51)
#define MAX_PICTURE_SIZE_DISABLE (0)
#define MAX_PICTURE_SIZE_ENABLE  (1)
#define QP_PARAM_HIGH			 (51)
#define QP_PARAM_LOW			 (0)
#define CPB_SIZE                 (1000)
#define INITIAL_DELAY            (500)
#define PERIODICITY_IDR          (60)
#define ENC_PARAM_HIGH           (4294967295)
#define ENC_PARAM_LOW			 (0)

/*Encoder params */
#define CACHE_DEFAULT            (L2_CACHE_ENABLE)
#define BANDWIDTH_DEFAULT        (LOW_BANDWIDTH_DISABLE)
#define FILLER_DATA_DEFAULT      (FILLER_DATA_DISABLE)
#define BITRATE_DEFAULT 	     (BITRATE_MAX)
#define GOP_LEN_DEFAULT	         (60)
#define B_FRAME_DEFAULT 	     (B_FRAME_MIN)
#define SLICE_DEFAULT		     (8)
#define QP_MODE_DEFAULT		     (QP_MODE_AUTO)
#define RC_MODE_DEFAULT		     (RATE_CONTROL_MODE_CBR)
#define ENC_TYPE_DEFAULT	     (ENC_TYPE_AVC)
#define GOP_MODE_DEFAULT	     (GOP_MODE_BASIC)
#define FPS_DEFAULT              (FPS_MIN)
#define MIN_QP_DEFAULT		     (MIN_QP)
#define MAX_QP_DEFAULT		     (MAX_QP)
#define MAX_PICTURE_SIZE_DEFAULT (MAX_PICTURE_SIZE_DISABLE)
#define CPB_SIZE_DEFAULT         (CPB_SIZE)
#define INITIAL_DELAY_DEFAULT    (INITIAL_DELAY)
#define PERIODICITY_IDR_DEFAULT  (PERIODICITY_IDR)

/* Input use case type params */
#define USE_CASE_TYPE_ENCODE       (0)
#define USE_CASE_TYPE_DECODE       (1)
#define USE_CASE_TYPE_TRANSCODE    (2)
#define USE_CASE_TYPE_DEFAULT      (USE_CASE_TYPE_TRANSCODE)

/*Input Format type params*/
#define FORMAT_TYPE_NV12           (0)
#define FORMAT_TYPE_NV16           (1)
#define FORMAT_TYPE_XV15           (2)
#define FORMAT_TYPE_XV20           (3)
#define FORMAT_TYPE_DEFAULT        (FORMAT_TYPE_NV12)

/*Profile Type*/
#define BASELINE_PROFILE           (0)
#define HIGH_PROFILE               (1)
#define MAIN_PROFILE               (2)
#define DEFAULT_PROFILE_AVC        (HIGH_PROFILE)
#define DEFAULT_PROFILE_HEVC       (MAIN_PROFILE)

/* Input file params */
#define INPUT_WIDTH_DEFAULT        (1920)
#define INPUT_HEIGHT_DEFAULT       (1080)

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

int cache = CACHE_DEFAULT;
int bandwidth = BANDWIDTH_DEFAULT;
int filler_data = FILLER_DATA_DEFAULT;
int bitrate = BITRATE_DEFAULT;
int gop_len = GOP_LEN_DEFAULT;
int b_frame = B_FRAME_DEFAULT;
int slice = SLICE_DEFAULT;
int qp_mode = QP_MODE_DEFAULT;
int rc_mode = RC_MODE_DEFAULT;
int enc_type = ENC_TYPE_DEFAULT;
int gop_mode = GOP_MODE_DEFAULT;
int format_type = FORMAT_TYPE_DEFAULT;
int profile=DEFAULT_PROFILE_AVC;
int max_picture_size = MAX_PICTURE_SIZE_DEFAULT;
unsigned int use_case_type = USE_CASE_TYPE_DEFAULT;
unsigned int in_width = INPUT_WIDTH_DEFAULT;
unsigned int in_height = INPUT_HEIGHT_DEFAULT;
unsigned int fps = FPS_DEFAULT;
unsigned int min_qp = MIN_QP_DEFAULT;
unsigned int max_qp = MAX_QP_DEFAULT;
unsigned int cpb_size = CPB_SIZE_DEFAULT;
unsigned int initial_delay = INITIAL_DELAY_DEFAULT;
unsigned int periodicity_idr = PERIODICITY_IDR_DEFAULT;

static int test_dma(char *h2c_device, char *c2h_device, char *infname, char *outfname);
void *pcie_dma_read(void * vargp);
void *pcie_dma_write(void * vargp);

static struct option const long_opts[] = {
	{"input file name", required_argument, NULL, 'i'},
	{"output file name.\n \
	Default: out.ts", required_argument, NULL, 'o'},
	{"encoder's l2 cache. It's a boolean parameter. Range of this value is [0-1].\n \
	Default:1 [Enable]", required_argument, NULL, 'c'},
	{"encoder's low bandwidth. It's a boolean parameter. Range of this value is [0-1].\n \
	Default:0 [Disable]", required_argument, NULL, 'w'},
	{"encoder's filler data. It's a boolean parameter. Range of this value is [0-1]. \n \
	Default:0 [Disable]", required_argument, NULL, 'f'},
	{"encoder's bitrate. Range of this value is [1-60000]. Unit is in Kbps.\n \
	Default:60000", required_argument, NULL, 'r'},
	{"encoder's GOP length. Range of this value is [1-1000].\n \
	Default:60", required_argument, NULL, 'g'},
	{"encoder's b-frames. Range of this value is [0-4].\n \
	Default:0", required_argument, NULL, 'b'},
	{"encoder's slice value. Range of this value is [4-22].\n \
	Default:8", required_argument, NULL, 's'},
	{"encoder's QP mode. Range of this value is [0 or 2]. O stands for uniform and 2 stands for auto.\n \
	Default:2 [auto]", required_argument, NULL, 'q'},
	{"encoder's rate control mode. Range of this value is [1-2]. 1 stands for VBR and 2 stands for CBR.\n \
	Default:2 [CBR]", required_argument, NULL, 'm'},
	{"encoder's encoder type. Range of this value is [0-1]. O stands for AVC and 1 stands for HEVC.\n \
	Default:0 [AVC]", required_argument, NULL, 'e'},
	{"encoder's min-qp parameter. Range of this value is [0-51].\n \
	Default:", required_argument, NULL, 'a'},
	{"encoder's max-qp parameter. Range of this value is [0-51].\n \
	Default:0", required_argument, NULL, 'j'},
	{"encoder's max-picture-size parameter. Range of this value is [0-1]. O stands for Disale and 1 stands for Enable.\n \
	Default:0", required_argument, NULL, 'k'},
	{"encoder's cpb-size. Range of this value is [0-4294967295].\n \
	Default:500", required_argument, NULL, 'x'},
	{"encoder's initial-delay. Range of this value is [0-4294967295].\n \
	Default:500", required_argument, NULL, 'y'},
	{"encoder's periodicity-idr. Range of this value is [0-4294967295].\n \
	Default:240", required_argument, NULL, 'z'},
	{"encoder's GOP mode. Range if this value is [0, 3 or 4]: O stands for basic, 3 stands for low-delay-p and 4 stands for low-delay-b.\n \
    Default:0 [BASIC]", required_argument, NULL, 'p'},
	{"use case type. Range of this value is [0, 1 or 2]: O stands for encode, 1 stands for decode and 2 stands for transcode.\n \
	Default:2 [TRANSCODE]", required_argument, NULL, 'u'},
	{"input resolution. Range of this value is [1920x1080 or 3840x2160]: 1920x1080 represents width x height of input video.\n \
	Default:1920x1080", required_argument, NULL, 'd'},
	{"input FPS. Range of this value is [30 or 60]: this parameter is used for encode use case.\n \
	Default:30", required_argument, NULL, 'F'},
	{"input Format. Range of this value is [0(NV12), 1(NV16), 2(XV15) or 3(XV20)]: this parameter is used for encode/transcode use case.\n \
	Default:0", required_argument, NULL, 'n'},
	{"input profile. Range of this value is [0, 1 or 2]: 0 is for BASE line, 1 for high and 2 is for main profle.\n \
	Default:1 for AVC and 2 for HEVC\n.", required_argument, NULL, 't'},
	{"help", no_argument, NULL, 'h'},
	{"verbose", no_argument, NULL, 'v'},
	{0, 0, 0, 0}
};

static void usage(const char *name)
{
	int i = 0;

	fprintf(stdout, "%s\n\n", name);
	fprintf(stdout, "usage: %s [OPTIONS]\n\n", name);

	for (i=0 ; i<(sizeof(long_opts)/sizeof(long_opts[0])) - 2; i++)
		fprintf(stdout, "  -%c represents %s.\n",
			long_opts[i].val, long_opts[i].name);
	fprintf(stdout, "  -%c (%s) print usage help and exit\n",
		long_opts[i].val, long_opts[i].name);
	i++;
}

int main(int argc, char *argv[])
{
	int cmd_opt;
	char *h2c_device = H2C_DEVICE;
	char *c2h_device = C2H_DEVICE;
	char *outfname = OUTPUT_FILENAME;
	char *infname;
	char cross_sign = 0;

	while ((cmd_opt =
		getopt_long(argc, argv, "vhc:i:o:c:w:f:r:g:b:a:j:k:s:q:m:e:p:u:d:F:v:n:t:x:y:z:", long_opts,
			    NULL)) != -1) {
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

		case 'c':
			/* L2 prefetch cache enable */
			cache = getopt_integer(optarg);
			if (cache < L2_CACHE_DISABLE || cache > L2_CACHE_ENABLE) {
				cache = CACHE_DEFAULT;
				printf(" -c Cache value should be 0-1, setting to default %d\n", cache);
			}
			break;

		case 'w':
			/* Bandwidth value */
			bandwidth = getopt_integer(optarg);
			if (bandwidth < LOW_BANDWIDTH_DISABLE || bandwidth > LOW_BANDWIDTH_ENABLE) {
				bandwidth = BANDWIDTH_DEFAULT;
				printf("-w Bandwidth value should be 0-1, setting to default %d\n", bandwidth);
			}
			break;

		case 'f':
			/*Set Filler data */
			filler_data = getopt_integer(optarg);
			if (filler_data < FILLER_DATA_DISABLE || filler_data > FILLER_DATA_ENABLE) {
				filler_data = FILLER_DATA_DEFAULT;
				printf("-f Filler data value should be 0-1, setting to default %d\n", filler_data);
			}
			break;

		case 'r':
			/* Set Bitrate */
			bitrate = getopt_integer(optarg);
			if (bitrate < BITRATE_MIN || bitrate > BITRATE_MAX) {
				bitrate = BITRATE_DEFAULT;
				printf("-r Bitrate value should be 1-60000, setting to default %d\n", bitrate);
			}
			break;

		case 'g':
			/* Set gop length */
			gop_len = getopt_integer(optarg);
			if (gop_len < GOP_LENGTH_MIN || gop_len >= GOP_LENGTH_MAX) {
				gop_len = GOP_LEN_DEFAULT;
				printf("-g Gop length value should be 1-1000, setting to default %d\n", gop_len);
			}
			break;

		case 'b':
			/* Set B-frames */
			b_frame = getopt_integer(optarg);
			if (b_frame < B_FRAME_MIN || b_frame > B_FRAME_MAX) {
				b_frame = B_FRAME_DEFAULT;
				printf("-b B-frames value should be 0-4, setting to default %d\n", b_frame);
			}
			break;

		case 's':
			/* Set slice value */
			slice = getopt_integer(optarg);
			if (slice < SLICE_MIN || slice > SLICE_MAX) {
				slice = SLICE_DEFAULT;
				printf("-s Slice value should be 4-22, setting to default %d\n", slice);
			}
			break;

		case 'q':
			/* Set qp mode value*/
			qp_mode = getopt_integer(optarg);
			if (qp_mode != QP_MODE_UNIFORM && qp_mode != QP_MODE_AUTO) {
				qp_mode = QP_MODE_DEFAULT;
				printf("-q QP mode value should be 0 or 2, setting to default %d\n", qp_mode);
			}
			break;

		case 'm':
			/* Set rate control mode */
			rc_mode = getopt_integer(optarg);
			if (rc_mode < RATE_CONTROL_MODE_VBR || rc_mode > RATE_CONTROL_MODE_CBR) {
				rc_mode = RC_MODE_DEFAULT;
				printf("-m Rate control mode should be 1-2 , setting to default %d\n 1-VBR 2-CBR \n", rc_mode);
			}
			break;

		case 'e':
			/* Set Encoder type */
			enc_type = getopt_integer(optarg);
			if (enc_type < ENC_TYPE_AVC || enc_type > ENC_TYPE_HEVC) {
				enc_type = ENC_TYPE_DEFAULT;
				printf("-e  Encoder type should be 0-1, setting to default %d \n 0-AVC 1-HEVC \n", enc_type);
			}
			if (enc_type == ENC_TYPE_AVC)
				profile = DEFAULT_PROFILE_AVC;
			else
				profile = DEFAULT_PROFILE_HEVC;
			break;

		case 'p':
			/* Set gop mode */
			gop_mode = getopt_integer(optarg);
			if (gop_mode != GOP_MODE_BASIC && gop_mode != GOP_MODE_LOW_DELAY_P && gop_mode != GOP_MODE_LOW_DELAY_B) {
				gop_mode = GOP_MODE_DEFAULT;
				printf("-p Gop mode should be 0, 3 or 4, setting to default %d \n", gop_mode);
			}
			break;

		case 'a':
			/* Set mip-qp */
			min_qp = getopt_integer(optarg);
			if (min_qp < QP_PARAM_LOW || min_qp > QP_PARAM_HIGH) {
				min_qp = MIN_QP_DEFAULT;
				printf("-a min-qp value should be in between 0 to 51, setting to default  %d \n", min_qp);
			}
			break;

		case 'j':
			/* Set max-qp */
			max_qp = getopt_integer(optarg);
			if (max_qp < QP_PARAM_LOW || max_qp > QP_PARAM_HIGH) {
				max_qp = MAX_QP_DEFAULT;
				printf("-j max-qp value should be in between 0 to 51, setting to default  %d \n", max_qp);
			}
			break;

		case 'k':
			/* max-picture-size enable */
			max_picture_size = getopt_integer(optarg);
			if (max_picture_size < MAX_PICTURE_SIZE_DISABLE || max_picture_size > MAX_PICTURE_SIZE_ENABLE) {
				max_picture_size = MAX_PICTURE_SIZE_DEFAULT;
				printf(" -c max-picture-size value should be 0-1, setting to default %d\n", max_picture_size);
			}
			break;

		case 'x':
			/* Set cpb-size */
			cpb_size = getopt_integer(optarg);
			if (cpb_size < ENC_PARAM_LOW || cpb_size > ENC_PARAM_HIGH) {
				cpb_size = CPB_SIZE_DEFAULT;
				printf("-x cpb-size  value should be in between 0 to 4294967295, setting to default  %d \n", cpb_size);
			}
			break;

		case 'y':
			/* Set initial-delay */
			initial_delay = getopt_integer(optarg);
			if (initial_delay < ENC_PARAM_LOW || initial_delay > ENC_PARAM_HIGH) {
				initial_delay = INITIAL_DELAY_DEFAULT;
				printf("-y initial-delay  value should be in between 0 to 4294967295, setting to default  %d \n", initial_delay);
			}
			break;

		case 'z':
			/* Set periodicity-idr */
			periodicity_idr = getopt_integer(optarg);
			if (periodicity_idr < ENC_PARAM_LOW || periodicity_idr > ENC_PARAM_HIGH) {
				periodicity_idr = PERIODICITY_IDR_DEFAULT;
				printf("-y periodicity-idr  value should be in between 0 to 4294967295, setting to default  %d \n", periodicity_idr);
			}
			break;
		case 'u':
			/* Set use case type */
			use_case_type = getopt_integer(optarg);
			if (use_case_type != USE_CASE_TYPE_ENCODE && use_case_type != USE_CASE_TYPE_DECODE && use_case_type != USE_CASE_TYPE_TRANSCODE) {
				use_case_type = USE_CASE_TYPE_DEFAULT;
				printf("-u Use case type should be 0, 1 or 2, setting to default %d \n", use_case_type);
			}
			break;

		case 'd':
			/* Set input resolution */
			sscanf(optarg, "%u%c%u", &in_width, &cross_sign, &in_height);

			if (!((in_width == 1920 && in_height == 1080) || (in_width == 3840 && in_height == 2160))) {
				in_width = 1920;
				in_height = 1080;
				printf("-d Dimensions should be 1920x1080 or 3840x2160, setting to default 1920x1080\n");
			}
			break;

		case 'F':
			/* Set input FPS */
			fps = getopt_integer(optarg);
			if (fps != FPS_MIN && fps != FPS_MAX) {
				fps = FPS_DEFAULT;
				printf("-F Input FPS should be 30 or 60, setting to default %d \n", fps);
			}
			break;

		case 'n':
			/* Set input format */
			format_type = getopt_integer(optarg);
			if (format_type != FORMAT_TYPE_NV12 && format_type != FORMAT_TYPE_NV16 && format_type != FORMAT_TYPE_XV15 && format_type != FORMAT_TYPE_XV20) {
				format_type = FORMAT_TYPE_DEFAULT;
				printf("-fmt Input Format should be 0,1,2 or 3, setting to default %d \n", format_type);
			}
			break;

		case 't':
			/* Set input profile */
			profile = getopt_integer(optarg);
			if ((enc_type == ENC_TYPE_AVC) && (profile != HIGH_PROFILE) && (format_type != FORMAT_TYPE_NV12)) {
				profile = DEFAULT_PROFILE_AVC;
				printf("Only High profile suppoted for H264 and NV16/XV15/XV20, setting to default %d \n", profile);
			} else if ((enc_type == ENC_TYPE_AVC) && (profile != BASELINE_PROFILE) && (profile != MAIN_PROFILE) && (profile != HIGH_PROFILE)) {
				profile = DEFAULT_PROFILE_AVC;
				printf("Profile can be Baseline, High or main for H264 codec type, setting to default %d \n", profile);
			} else if ((enc_type == ENC_TYPE_HEVC) && (profile != MAIN_PROFILE)) {
				profile = DEFAULT_PROFILE_HEVC;
				printf("Only Main profile supported for H265 codec type, setting to default %d \n", profile);
			}
            break;
		/* print usage help and exit */
		case 'v':
			verbose = 1;
			break;

		case 'h':
	default:
			usage(argv[0]);
			exit(0);
			break;
		}
	}

	return test_dma(h2c_device, c2h_device, infname, outfname);
}

static int test_dma(char *h2c_device, char *c2h_device, char *infname, char *outfname)
{
	ssize_t rc;
	volatile unsigned int *encoder_params_1, *encoder_params_2, *encoder_params_3, *encoder_params_4, *encoder_params_5;
	volatile unsigned int *host_done;
	unsigned int enc_param1;
	unsigned int enc_param2;
	unsigned int enc_param3;
	unsigned int enc_param4;
	unsigned int enc_param5;
	volatile unsigned int *infile_len;
	unsigned long int file_len;
	volatile unsigned int *input_res = NULL;
	volatile unsigned int *use_case = NULL;

	pthread_t thread1, thread2;

	trans.h2c_fd = open(h2c_device, O_RDWR);
	if (trans.h2c_fd < 0) {
		fprintf(stderr, "unable to open device %s, %d.\n",
			h2c_device, trans.h2c_fd);
		perror("open device");
		return -EINVAL;
	}

	trans.c2h_fd = open(c2h_device, O_RDWR);
	if (trans.c2h_fd < 0) {
		fprintf(stderr, "unable to open device %s, %d.\n",
			c2h_device, trans.c2h_fd);
		perror("open device");
		goto h2c_out;
	}


	trans.reg_fd = open(REG_DEVICE_NAME, O_RDWR);
	if (trans.reg_fd < 0) {
        fprintf(stderr, "unable to open device %s, %d.\n",
            REG_DEVICE_NAME, trans.reg_fd);
        perror("open reg device failed");
		goto c2h_out;
    }

	trans.infname = infname;
	trans.infile_fd = open(infname, O_RDONLY);
	if (trans.infile_fd < 0) {
		fprintf(stderr, "unable to open input file %s, %d.\n",
			infname, trans.infile_fd);
		printf("open input file failed");
		rc = -EINVAL;
		goto reg_out;
	}

	trans.ofname = outfname;
	trans.ofile_fd = open(outfname, O_RDWR | O_CREAT | O_TRUNC | O_SYNC,
			0666);

	if (trans.ofile_fd < 0) {
		fprintf(stderr, "unable to open input file %s, %d.\n",
			outfname, trans.ofile_fd);
		perror("open input file");
		rc = -EINVAL;
		goto infile_out;
	}

	/* map one page */
	trans.map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, trans.reg_fd, 0);
	if (trans.map_base == (void *)-1) {
		printf("error unmap\n");
		printf("Memory mapped at address %p.\n", trans.map_base);
		fflush(stdout);
		goto outfile_out;
	}

	/* Get the input file length  */
	if (trans.infile_fd > 0) {
		file_len = lseek(trans.infile_fd, 0, SEEK_END);
		if (file_len == (off_t)-1)
		{
			printf("failed to lseek %s numbytes %lu\n", trans.infname, file_len);
			exit(EXIT_FAILURE);
		}
		printf("File lenght is :%d\n",file_len);

		/* reset the file position indicator to
		the beginning of the file */

		lseek(trans.infile_fd, 0L, SEEK_SET);

		infile_len = ((uint32_t *)(trans.map_base + PCIRC_GET_FILE_LENGTH));
		*infile_len = (unsigned int)(file_len & 0xFFFFFFFF);

		infile_len = ((uint32_t *)(trans.map_base + PCIRC_GET_FILE_LENGTH - 4));
		*infile_len = (unsigned int)(file_len >> 32 & 0xFFFFFFFF);
	}

	/* setting use case type and FPS */
	use_case = ((uint32_t *)(trans.map_base + PCIRC_GET_USECASE_TYPE));
	*use_case = (use_case_type | (format_type << 2) | (fps << 5));


	/* setting input resolution */
	input_res = ((uint32_t *)(trans.map_base + PCIRC_RAW_RESOLUTION));
	*input_res = (in_height << 16) | in_width;

	/* setting encoder parameters */
	enc_param1 = (cache) | (bandwidth << 1) | (filler_data << 2) | (bitrate << 4) | (gop_len << 20) | (max_picture_size << 30);
	encoder_params_1 = ((uint32_t *)(trans.map_base + PCIRC_ENC_PARAMS_1));
	*encoder_params_1 = enc_param1;

	enc_param2 = (b_frame) | (slice << 3) | (qp_mode << 9) | (rc_mode << 11 ) | (enc_type << 13)
                                            | (gop_mode << 15) | (profile << 18) | (min_qp << 20) | (max_qp << 26);
	encoder_params_2 = ((uint32_t *)(trans.map_base + PCIRC_ENC_PARAMS_2));
	*encoder_params_2 = enc_param2;

	enc_param3 = (cpb_size);
	encoder_params_3 = ((uint32_t *)(trans.map_base + PCIRC_ENC_PARAMS_3));
	*encoder_params_3 = enc_param3;

	enc_param4 = (initial_delay);
	encoder_params_4 = ((uint32_t *)(trans.map_base + PCIRC_ENC_PARAMS_4));
	*encoder_params_4 = enc_param4;

	enc_param5 = (periodicity_idr);
	encoder_params_5 = ((uint32_t *)(trans.map_base + PCIRC_ENC_PARAMS_5));
	*encoder_params_5 = enc_param5;

	pthread_create( &thread1, NULL, &pcie_dma_read, NULL);
	pthread_create( &thread2, NULL, &pcie_dma_write, NULL);
	pthread_join( thread1, NULL);
	pthread_join( thread2, NULL);

	/*set host done interrupt */
	host_done = ((uint32_t *)(trans.map_base + PCIRC_HOST_DONE));
	*host_done = 0x1;

	/* clear file length and encoder parameters */
	*infile_len = 0x0;
	*encoder_params_1 = 0x0;
	*encoder_params_2 = 0x0;

reg_out:
	if (munmap(trans.map_base, MAP_SIZE) == -1)
		printf("error unmap\n");
	close(trans.reg_fd);

outfile_out:
	if (trans.ofile_fd >= 0)
		close(trans.ofile_fd);


infile_out:
	if (trans.infile_fd >= 0)
		close(trans.infile_fd);

c2h_out:
	close(trans.c2h_fd);

h2c_out:
	close(trans.h2c_fd);
	return rc;
}

void *pcie_dma_read(void *vargp)
{
	struct timespec ts_start;
	int rc, i;
	int count = COUNT_DEFAULT;
	volatile unsigned int addr, size, buffer_ready, read_complete;
	volatile unsigned long int offset;
	volatile unsigned int lsb_offset, msb_offset;
	volatile unsigned int *transfer_done;
	char *read_allocated = NULL;

	printf("Running use case\n");
	while (1) {

		transfer_done = ((uint32_t *)(trans.map_base + PCIRC_READ_BUFFER_TRANSFER_DONE));
		buffer_ready = *((uint32_t *)(trans.map_base + PCIEP_READ_BUFFER_READY));
		read_complete = *((uint32_t *)(trans.map_base + PCIEP_READ_TRANSFER_COMPLETE));

		while (!(buffer_ready & 0x1))
		{
			buffer_ready = *((uint32_t *)(trans.map_base + PCIEP_READ_BUFFER_READY));
			read_complete = *((uint32_t *)(trans.map_base + PCIEP_READ_TRANSFER_COMPLETE));
			if (read_complete == 0xef)
				break;
		}

		if (read_complete == 0xef)
			break;

		addr = *((uint32_t *)(trans.map_base + PCIEP_READ_BUFFER_ADDR));
		size = *((uint32_t *) (trans.map_base + PCIEP_READ_BUFFER_SIZE));
		lsb_offset = *((uint32_t *) (trans.map_base + PCIEP_READ_BUFFER_OFFSET));
		msb_offset = *((uint32_t *) (trans.map_base + PCIEP_READ_BUFFER_READY));

		offset = lsb_offset | ((unsigned long int)(msb_offset & 0xFFFF0000) << 16);

		posix_memalign((void **)&read_allocated, 4096 /*alignment */ , size + 4096);
		if (!read_allocated) {
			fprintf(stderr, "OOM %u.\n", size + 4096);
			rc = -ENOMEM;
			goto read_out;
		}

		trans.read_buffer = read_allocated;
		if (verbose)
			fprintf(stdout, "host buffer 0x%x = %p\n",
								size + 4096, trans.read_buffer);

		printf("#");
		fflush(stdout);
		if (trans.infile_fd > 0) {
			rc = read_to_buffer(trans.infname, trans.infile_fd, trans.read_buffer, size, offset);

			if (rc < 0) {
				printf("read to buffer failed size %d rc %d", size, rc);
				goto out;
			}
		}

		for (i = 0; i < count; i++) {
		/* write buffer to AXI MM address using SGDMA */
			rc = clock_gettime(CLOCK_MONOTONIC, &ts_start);

			rc = write_from_buffer(H2C_DEVICE, trans.h2c_fd, trans.read_buffer, size, addr);
			if (rc < 0) {
				printf("write from buffer failed size %d rc %d", size, rc);
				goto out;
			}

			*transfer_done = 0x1;
			while ((buffer_ready & 0x1)) {
				buffer_ready = *((uint32_t *)(trans.map_base + PCIEP_READ_BUFFER_READY));
			}
		}

		if (read_allocated) {
			free(read_allocated);
			read_allocated = NULL;
		}
	}

out:
	printf("\n** Read done\n");
read_out:
	if (read_allocated) {
		free(read_allocated);
		read_allocated = NULL;
	}
	return NULL;
}

void  *pcie_dma_write(void * argp)
{
	int rc, i;
	int count = COUNT_DEFAULT;
	volatile unsigned int addr, size, offset, write_buffer_ready, write_complete;
	volatile unsigned int *transfer_done;
	int var = 0;
	char *write_allocated = NULL;

	while (1) {
		transfer_done = ((uint32_t *)(trans.map_base + PCIRC_WRITE_BUFFER_TRANSFER_DONE));
		write_buffer_ready = *((uint32_t *)(trans.map_base + PCIEP_WRITE_BUFFER_READY));
		write_complete = *((uint32_t *)(trans.map_base + PCIEP_WRITE_TRANSFER_COMPLETE));
		*transfer_done = 0x0;

		while (!(write_buffer_ready & 0x1))
		{
			write_buffer_ready = *((uint32_t *)(trans.map_base + PCIEP_WRITE_BUFFER_READY));
			write_complete = *((uint32_t *)(trans.map_base + PCIEP_WRITE_TRANSFER_COMPLETE));
			if (write_complete == 0xef)	{
				break;
			}
		}

		if (write_complete == 0xef) {
			break;
		}

		addr = *((uint32_t *)(trans.map_base + PCIEP_WRITE_BUFFER_ADDR));
		size = *((uint32_t *) (trans.map_base + PCIEP_WRITE_BUFFER_SIZE));
		offset = *((uint32_t *) (trans.map_base + PCIEP_WRITE_BUFFER_OFFSET));
		var ++;
		printf("#");
		fflush(stdout);
		posix_memalign((void **)&write_allocated, 4096 /*alignment */ , size + 4096);
		if (!write_allocated) {
			fprintf(stderr, "OOM %u.\n", size + 4096);
			rc = -ENOMEM;
			goto out;
		}

		trans.write_buffer = write_allocated;
		if (verbose)
			fprintf(stdout, "host buffer 0x%x = %p\n",
				size + 4096, trans.write_buffer);

		if (trans.c2h_fd) {
			rc = read_to_buffer(C2H_DEVICE, trans.c2h_fd, trans.write_buffer, size, addr);
			if (rc < 0) {
				printf("write function read to buffer failed size %d rc %d\n", size, rc);
				goto out;
			}
		}

		for (i = 0; i < count; i++) {
		/* write buffer to AXI MM address using SGDMA */
			/* file argument given? */
			if (trans.ofile_fd >= 0) {
				rc = write_from_buffer(trans.ofname, trans.ofile_fd, trans.write_buffer,
						 size, offset);
				if (rc < 0) {
					printf("In write function write from buffer failed with size %d rc %d\n", size, rc);
					goto out;
				}
			}
		}

		*transfer_done = 0x1;
		while(write_buffer_ready) {
			write_buffer_ready = *((uint32_t *)(trans.map_base + PCIEP_WRITE_BUFFER_READY));
		}

		if (write_allocated) {
			free(write_allocated);
			write_allocated = NULL;
		}
	}
out:
	printf("** Write done\n");
	if (write_allocated) {
		free(write_allocated);
		write_allocated = NULL;
	}

	return NULL;
}
