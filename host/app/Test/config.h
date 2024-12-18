#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG

// ANSI escape codes for text colors
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

#define AXI_STREAM_LEN (960 * 720 * 2) / 8 + 4
#define DVS_FPS 2000
#define DISPLAY_FPS 100
#define DMA_BUFFER_GRP_NUM DVS_FPS/DISPLAY_FPS
#define waitkey_delay 1000/DISPLAY_FPS

#define H2C_DEVICE "/dev/xdma_zcu1060_h2c_0"
#define C2H_DEVICE "/dev/xdma_zcu1060_c2h_0"
#define REG_DEVICE "/dev/xdma_zcu1060_xvc"
#define USER_DEVICE "/dev/xdma_zcu1060_user"
#define MAP_SIZE (32 * 1024UL)
#define MAP_MASK (MAP_SIZE - 1)
#define COUNT_DEFAULT (1)

#define PCIEP_READ_BUFFER_PTR 0x3c
#define PCIEP_WRITE_BUFFER_PTR 0x40

#define MAX_FILES 100
#define MAX_PATH_LENGTH 1024

#endif