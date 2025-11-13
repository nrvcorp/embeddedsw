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

/******************* HOST Setting ******************************/
#define CPU_CORE_BUF_PRODUCER 2
#define CPU_CORE_BUF_CONSUMER 3
#define CPU_CORE_BUF_CONSUMER_2 4
#define CPU_CORE_BUF_CONSUMER_3 5

#define CPU_CORE_COMPARATOR 5

#define HOST_FRAME_BATCH_BUFFER_NUM 512
#define NUM_FRAMES_PER_TRANSFER 1

#define READ_HEADER_ONLY_IF_NOT_NEEDED 1
// ** pcie 지연이 줄어드는만큼 폴링 루프가 쌩쌩 돌아가기 때문에 헤더만 읽는 비율이 늘어날수록 cpu 부담이 커짐
/* READ_HEADER_ONLY_IF_NOT_NEEDED:
  When displaying PCIe‐read frames with downsampling, frames that
  are not required for the downsampled output will not incur the
  full PCIe transfer. Instead, only their header is fetched to
  minimize bandwidth and latency.
 */

#define CHECK_ALL_READY_FLAGS 0
/*CHECK_ALL_READY_FLAGS:
   When we fetch *N* consecutive frames from the PCIe circular
   buffer, there are two ways to make sure those frames are really
   available:

      1.  Poll **every** READY flag          (strict / safest)
      0.  Poll **only the last** READY flag  (fast / implicit)

   • The firmware sets READY flags strictly in order.
     Therefore, if the LAST flag in the group is high, all previous
     flags **must** already be high as well.
     ──>   Reading just the final flag cuts the number of PCIe
           MMIO reads roughly in half and reduces latency.

   • In abnormal situations (e.g. firmware bug, flag skipped),
     checking every flag will detect the error immediately, whereas
     “last‑only” mode might miss it.
*/

//
#define CV_IMWRITE_PNG_COMPRESSION_LEVEL 3 // 1  ~ 9
#define CV_WORKER_CORES_NUM 12
#define CV_WORKER_CORES_LIST 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15

/******************* FIRMWARE Settings ******************************/
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

#define FRAME_HEADER_BYTES 16 // 확장헤더 16바이트 //기본 8

#define ID_DVS 0
#define ID_FIL 1

// raw
#define DVS_BUFFER_NUM 256 // has to be divisible by NUM_FRAMES_PER_TRANSFER
#define DVS_BUFFER_NUM_BURST (DVS_BUFFER_NUM / NUM_FRAMES_PER_TRANSFER)
#define DVS_FRAME_RDY_BASEADDR (DDR_BASEADDR + 0x2000000)
#define DVS_COMMIT_INDEX_ADDR (DDR_BASEADDR + 0x1500000)
#define DVS_FRAME_BASEADDR (DDR_BASEADDR + 0x30000000)

#if DVS_BUFFER_NUM % NUM_FRAMES_PER_TRANSFER != 0
#error "DVS_BUFFER_NUM must be a multiple of NUM_FRAMES_PER_TRANSFER"
#endif

// filtered
#define FIL_BUFFER_NUM 256 // has to be divisible by NUM_FRAMES_PER_TRANSFER
#define FIL_BUFFER_NUM_BURST (FIL_BUFFER_NUM / NUM_FRAMES_PER_TRANSFER)
#define FIL_FRAME_RDY_BASEADDR (DDR_BASEADDR + 0x2500000)
#define FIL_COMMIT_INDEX_ADDR (DVS_COMMIT_INDEX_ADDR + 0x10)
#define FIL_FRAME_BASEADDR (DDR_BASEADDR + 0x40000000)

#if FIL_BUFFER_NUM % NUM_FRAMES_PER_TRANSFER != 0
#error "FIL_BUFFER_NUM must be a multiple of NUM_FRAMES_PER_TRANSFER"
#endif

/******************* DISPLAY Setting ******************************/
#define DVS_FPS 3500
#define DISPLAY_FPS 60
#define DISPLAY_DOWNSAMPLE_NUM 53 // has to be divisible by NUM_FRAMES_PER_TRANSFER, >= 3
#define FRAME_ACCUM_COUNT__ ((DVS_FPS) / ((DISPLAY_FPS) * (DISPLAY_DOWNSAMPLE_NUM)))

#if FRAME_ACCUM_COUNT__ < 1
#define FRAME_ACCUM_COUNT 1
#else
#define FRAME_ACCUM_COUNT FRAME_ACCUM_COUNT__
#endif

#if DISPLAY_DOWNSAMPLE_NUM % NUM_FRAMES_PER_TRANSFER != 0
#error "DISPLAY_DOWNSAMPLE_NUM must be a multiple of NUM_FRAMES_PER_TRANSFER"
#endif

// static_assert(FRAME_ACCUM_COUNT >= HOST_FRAME_BATCH_BUFFER_NUM,
//               "HOST_FRAME_BATCH_BUFFER_NUM (" STR(HOST_FRAME_BATCH_BUFFER_NUM) ") must be >= FRAME_ACCUM_COUNT (" STR(FRAME_ACCUM_COUNT) ")");
#if FRAME_ACCUM_COUNT > HOST_FRAME_BATCH_BUFFER_NUM
#error "HOST_FRAME_BATCH_BUFFER_NUM must be greater than FRAME_ACCUM_COUNT "
#endif

#define SAVE_FPS 2000
#define ROI_EVENT_SCORE 5
#define ROW_SCORE_THRESHOLD 25
#define ROI_HEIGHT_MIN_THRESHOLD 10
#define ROI_INFLATION (float)(1)
#define DVS_ROI_MIN_SIZE 100
#define CIS_ROI_MIN_SIZE 224
// #define CIS_DVS_OFFSET_X 0.315
// #define CIS_DVS_OFFSET_Y -0.1
// #define CIS_DVS_SCALE_X 0.8
// #define CIS_DVS_SCALE_Y 1.1
#define CIS_DVS_OFFSET_X 0.580
#define CIS_DVS_OFFSET_Y 0.350
#define CIS_DVS_SCALE_X 0.3
#define CIS_DVS_SCALE_Y 0.38
#define DVS_WEIGHT_ALPHA 0.5
#define CIS_DVS_CALIBRATION_GRID_NUM 20
#define DMA_BUFFER_GRP_NUM (DVS_FPS / DISPLAY_FPS)
#define waitkey_delay (1000 / DISPLAY_FPS)

/******************* PCIE Setting ******************************/
#define H2C_DEVICE_DVS "/dev/xdma_custom_driver_10_h2c_0"
#define C2H_DEVICE_DVS "/dev/xdma_custom_driver_10_c2h_0"
#define H2C_DEVICE_CIS "/dev/xdma_custom_driver_10_h2c_1"
#define C2H_DEVICE_CIS "/dev/xdma_custom_driver_10_c2h_1"
#define REG_DEVICE "/dev/xdma_custom_driver_10_xvc"
#define USER_DEVICE "/dev/xdma_custom_driver_10_user"
#define MAP_SIZE (32 * 1024UL)
// #define MAP_MASK (MAP_SIZE - 1)
// #define COUNT_DEFAULT (1)

#endif
