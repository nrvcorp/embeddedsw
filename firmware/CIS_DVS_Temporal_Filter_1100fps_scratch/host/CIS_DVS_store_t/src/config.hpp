#ifndef CONFIG_H
#define CONFIG_H

// <Additional configuraitons>
// ANSI escape codes for text colors
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

// #define DEBUG

#define DDR_BASEADDR 0x10000000

/******************* CIS Setting **********************************/
#define CIS_FRAME_W 1920
#define CIS_FRAME_H 1080
#define CIS_BUFFER_NUM 5
#define CIS_FRAME_RDY_BASEADDR (DDR_BASEADDR + 0x1000000)
#define CIS_FRAME_BASEADDR (DDR_BASEADDR + 0x10000000)

/******************* DVS Setting **********************************/
#define DVS_FRAME_W 960
#define DVS_FRAME_H 720
#define FRAME_HEADER_BYTES 8
#define DVS_BUFFER_NUM 16
#define DVS_FRAME_RDY_BASEADDR (DDR_BASEADDR + 0x2500000)
#define DVS_FRAME_BASEADDR (DDR_BASEADDR + 0x40000000)

/******************* DISPLAY Setting ******************************/
#define DVS_FPS 2000
#define DISPLAY_FPS 60
#define SAVE_FPS 20
#define ROI_THRESH 7
#define ROI_INFLATION (float)(1.4)
#define ROI_MIN_SIZE 416
#define CIS_DVS_OFFSET_X 720
#define CIS_DVS_OFFSET_Y 330
#define CIS_DVS_SCALE_X 0.35
#define CIS_DVS_SCALE_Y 0.35
#define DMA_BUFFER_GRP_NUM (DVS_FPS / DISPLAY_FPS)
#define waitkey_delay (1000 / DISPLAY_FPS)

/******************* PCIE Setting ******************************/
#define H2C_DEVICE_DVS "/dev/xdma_custom_driver_1_0_h2c_0"
#define C2H_DEVICE_DVS "/dev/xdma_custom_driver_1_0_c2h_0"
#define H2C_DEVICE_CIS "/dev/xdma_custom_driver_1_0_h2c_1"
#define C2H_DEVICE_CIS "/dev/xdma_custom_driver_1_0_c2h_1"
#define REG_DEVICE "/dev/xdma_custom_driver_1_0_xvc"
#define USER_DEVICE "/dev/xdma_custom_driver_1_0_user"
#define MAP_SIZE (32 * 1024UL)
// #define MAP_MASK (MAP_SIZE - 1)
// #define COUNT_DEFAULT (1)


#endif
