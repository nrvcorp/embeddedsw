#ifndef DEBUG_CONFIG_H_
#define DEBUG_CONFIG_H_

#define CHECK_multiple_buf_display_fps_render 0

#define CHECK_multiple_buf_display_fps_pcie_reader 0

#define CHECK_multiple_buf_pcie_reader 0

#define CHECK_MULTIPLE_BUF_BIN_WRITER 0

#define CHECK_READ_FRAME 1
#define CHECK_READ_HEADER 1

#define CHECK_READ_FLAG 0

#if (CHECK_READ_FLAG == 0)
#define LOG_ALL_INTERVALS 0
#endif

#define COMPARE_FIL_DVS 0

#endif