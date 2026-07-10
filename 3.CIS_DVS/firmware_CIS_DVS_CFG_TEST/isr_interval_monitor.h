#ifndef ISR_INTERVAL_MONITOR_H_
#define ISR_INTERVAL_MONITOR_H_

#include "xtime_l.h"
#include "debug_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_HISTORY_SIZE  2510
#define GAP_FIFO_SIZE 10

typedef enum {
    INTERVAL_DVS_DMA_DONE_ISR = 0,
    INTERVAL_FIL_DMA_DONE_ISR,
    INTERVAL_TAG_COUNT
} interval_tag_t;

typedef struct {
    u64 last_time;
    u64 min_interval;
    u64 max_interval;
    u64 sum_interval;
    u32 count;

#if(CHECK_MAX_INTERVAL_SURROUNDINGS)
    u64 intervals_history[MAX_HISTORY_SIZE];  // 최근 ISR 진입 시각 저장
    int history_index;                    // 현재 저장 인덱스
    int max_index_in_history;             // max_interval 발생 인덱스
#endif
#if(CHECK_MAX_INTERVAL_SUM)
    u64 gap_fifo[GAP_FIFO_SIZE];      // 최근 gap 저장용 FIFO
    int fifo_index;                   // FIFO용 인덱스
    int fifo_count;                   // FIFO에 저장된 실제 gap 수
    u64 curr_sum;                     // 현재 FIFO의 gap 합
    u64 best_sum;                     // 역대 최고 gap 합
    u64 best_gap_window[GAP_FIFO_SIZE]; // 최고 gap합이 발생한 구간
#endif
} interval_stat_t;

// 두 개의 더블 버퍼 중 현재 기록 중인 버퍼 index
extern volatile int active_buf_index;

// 더블 버퍼 (0: 현재 기록용, 1: 이전 통계 출력용)
extern volatile interval_stat_t intervals_buf[2][INTERVAL_TAG_COUNT];

// 태그에 대응하는 문자열 (출력용)
extern const char* interval_names[INTERVAL_TAG_COUNT];

// ISR 내부에서 간격 기록
void IntervalRecordStart(interval_tag_t tag);

// 모든 통계를 출력하고 리셋 (swap 후 사용)
void PrintAndResetAllIntervals(void);

// 단일 태그만 출력하고 리셋 (swap 후 사용)
void PrintAndResetInterval(interval_tag_t tag);


// 기록용/출력용 버퍼 전환
void SwapIntervalBuffers(void);


#ifdef __cplusplus
}
#endif

#endif // ISR_INTERVAL_MONITOR_H_



/*#ifndef INTERVAL_STATS_H_
#define INTERVAL_STATS_H_

#include "xtime_l.h"


typedef enum {
	INTERVAL_DVS_DMA_DONE_ISR = 0,
    INTERVAL_FIL_DMA_DONE_ISR,
    INTERVAL_TAG_COUNT
} interval_tag_t;

typedef struct {
    u64 last_time;
    u64 min_interval;
    u64 max_interval;
    u64 sum_interval;
    u32 count;
} interval_stat_t;

extern volatile interval_stat_t intervals[INTERVAL_TAG_COUNT];


void IntervalRecordStart(interval_tag_t tag);
void PrintAndResetAllIntervals(void);

#endif*/
