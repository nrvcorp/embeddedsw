#ifndef CONFIG_H
#define CONFIG_H

// #define DEBUG

#define FRAME_W 960
#define FRAME_H 720
#define FRAME_HEADER_BYTES 8
#define AXI_STREAM_LEN (960 * 720 * 2) / 8 + 8

#define DVS_FPS 2000
#define DISPLAY_FPS 20
#define DMA_BUFFER_GRP_NUM DVS_FPS/DISPLAY_FPS
#define waitkey_delay 1000/DISPLAY_FPS

#define H2C_DEVICE "/dev/xdma_zcu1060_h2c_0"
#define C2H_DEVICE "/dev/xdma_zcu1060_c2h_0"
#define REG_DEVICE "/dev/xdma_zcu1060_xvc"
#define USER_DEVICE "/dev/xdma_zcu1060_user"
// #define MAP_SIZE (32 * 1024UL)
// #define MAP_MASK (MAP_SIZE - 1)
// #define COUNT_DEFAULT (1)

typedef enum
{
    COLOR_SCALE,
    GRAY_SCALE,
    SPEED_CHECK
} MODE;

typedef struct
{
    char *h2c_device;
    char *c2h_device;
    char *reg_device;
    char *user_device;
    int h2c_fd;
    int c2h_fd;
    int reg_fd;
    int user_fd;
} pcie_trans;



// <Additional configuraitons>
// ANSI escape codes for text colors
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"


#endif