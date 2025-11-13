
#ifndef DEBUG_CONFIG_H_
#define DEBUG_CONFIG_H_

/*
 *  Larger debugging overhead → allocate larger sensor/filter buffers.
 */


#define USE_EXTENDED_DVS_FRAME_HEADER 1

#define ENABLE_DVS_FILTER 1

#define ENABLE_DVS_RESET 1

#define ENABLE_HOST_DIRECT_DVS_ACCESS 1

// Periodically resets the DVS to a new configuration based on predefined rules.
// skipping/ignoring all user input (fully unattended mode).
#define AUTO_TEST_MODE 0
#define AUTO_RESET_ITV_US 4000000

#define RESET_DMA_BEFORE_PROGRAM 0 // reset dmas, clear buffers, reset ptrs

#define ENABLE_MENU 1

#define ENABLE_CIS 1 // TODO

#define ENABLE_MAIN_DEBUG_OUTPUT 1
#define MAIN_DEBUG_OUTPUT_INTERVAL_SEC 1 //  >= 1

#define CHECK_FRAME_COUNT 1

#define CHECK_DVS_ISR_INTERVAL 0
#define CHECK_FIL_ISR_INTERVAL 0  //  sub-option of ENABLE_DVS_FILTER
#define CHECK_ALL_ISR_INTERVAL (CHECK_DVS_ISR_INTERVAL && CHECK_FIL_ISR_INTERVAL)

#define CHECK_MAX_INTERVAL_SUM 0
#define CHECK_MAX_INTERVAL_SURROUNDINGS 0

#define CHECK_DVS_FRAME_DROP 1
#define CHECK_DVS_MULTIPLE_FRAME_DROP 1
#define DVS_FRAME_DROP_LOG_IMMEDIATE 0
//#define CHECK_DVS_FRAME_DROP_LOW_EVENT 1 //

#define CHECK_FIL_EVENT_COUNT 1  //  sub-option of ENABLE_DVS_FILTER


/* CHECK_DVS_BUFFER_HOST_DRAIN is a sub-option of ENABLE_HOST_DIRECT_DVS_ACCESS
 * and is ignored if ENABLE_HOST_DIRECT_DVS_ACCESS is not set.
 * */
#define CHECK_DVS_BUFFER_HOST_DRAIN 0
#define CHECK_FIL_BUFFER_HOST_DRAIN 0 //  sub-option of ENABLE_DVS_FILTER

#define FILTER_IGNORE_DVS_READY 0 //  sub-option of ENABLE_DVS_FILTER
#define CHECK_FILTER_REG_VALUES 0 //  sub-option of ENABLE_DVS_FILTER
#define CHECK_FILTER_DMA_READ_WRITE_SYNC 0 //  sub-option of ENABLE_DVS_FILTER




#define DEBUG    0
#define COMP 1
#define INFO     2
#define WARNING  4
#define ERROR    6
#define CRITICAL 8
#define MESSAGE 10

#define CURRENT_LOG_LEVEL WARNING

#define LOG_LEVEL_STR(level) \
    ((level) == DEBUG ? "DEBUG: " : \
    (level) == INFO ? "INFO: " : \
    (level) == WARNING ? "WARNING: " : \
    (level) == ERROR ? "ERROR: " : \
    (level) == CRITICAL ? "CRITICAL: " :  \
	(level) == COMP ? "COMPARE: " : "MESSAGE: ")

#define DEBUG_PRINT(level, fmt, ...) \
    do { \
        if (CURRENT_LOG_LEVEL <= level) { \
            xil_printf("%s" fmt, LOG_LEVEL_STR(level), ##__VA_ARGS__); \
        } \
    } while (0)



#endif


