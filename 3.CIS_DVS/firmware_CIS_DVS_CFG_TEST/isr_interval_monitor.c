#include "isr_interval_monitor.h"
#include "xil_printf.h"
#include "debug_config.h"

volatile interval_stat_t intervals_buf[2][INTERVAL_TAG_COUNT];
volatile int active_buf_index = 0;

const char* interval_names[INTERVAL_TAG_COUNT] = {
    "DVS_DMA_DONE_ISR",
    "FIL_DMA_DONE_ISR",
};

// 시스템 틱을 us 단위로 변환
static inline u64 TicksToUs(u64 ticks) {
    return ticks * 1000000ULL / (u64)COUNTS_PER_SECOND;
}

// ISR 진입 시 호출: 시간 기록 및 통계 갱신
void IntervalRecordStart(interval_tag_t tag) {
    int index = active_buf_index;
    volatile interval_stat_t* stat = &intervals_buf[index][tag];

    static int state = 0;
    if (state == 1) {
        DEBUG_PRINT(DEBUG, "Warning: ISR interval statistics may be incomplete due to re-entrant interrupt.\r\n");
    }
    state = 1;

    XTime now;
    XTime_GetTime(&now);

    u64 last = stat->last_time;
    stat->last_time = now;

    if (last != 0) {
        u64 gap = now - last;
#if(CHECK_MAX_INTERVAL_SURROUNDINGS)
        stat->intervals_history[stat->history_index] = gap;

        if (stat->count == 0) {
            stat->min_interval = gap;
            stat->max_interval = gap;
            stat->max_index_in_history = stat->history_index;
        } else {
            if (gap < stat->min_interval) stat->min_interval = gap;
            if (gap > stat->max_interval) {
                stat->max_interval = gap;
                stat->max_index_in_history = stat->history_index;
            }
        }
#else
		if (stat->count == 0) {
			stat->min_interval = gap;
			stat->max_interval = gap;
		} else {
			if (gap < stat->min_interval) stat->min_interval = gap;
			if (gap > stat->max_interval) {
				stat->max_interval = gap;
			}
		}
#endif
        stat->sum_interval += gap;
        stat->count++;

#if(CHECK_MAX_INTERVAL_SUM)
        // gap FIFO 처리
		if (stat->fifo_count < GAP_FIFO_SIZE) {
			// 아직 FIFO가 가득 차지 않은 경우
			stat->curr_sum += gap;
			stat->gap_fifo[stat->fifo_index] = gap;
			stat->fifo_index = (stat->fifo_index + 1) % GAP_FIFO_SIZE;
			stat->fifo_count++;
		} else {
			// FIFO가 가득 찬 경우: 가장 오래된 gap 제거 후 새 gap 추가
			int oldest_idx = stat->fifo_index;
			u64 old_gap = stat->gap_fifo[oldest_idx];

			stat->curr_sum = stat->curr_sum - old_gap + gap;
			stat->gap_fifo[oldest_idx] = gap;
			stat->fifo_index = (stat->fifo_index + 1) % GAP_FIFO_SIZE;
		}

		// 최대 gap합 갱신
		if (stat->fifo_count == GAP_FIFO_SIZE && stat->curr_sum > stat->best_sum) {
			stat->best_sum = stat->curr_sum;
			for (int i = 0; i < GAP_FIFO_SIZE; ++i) {
				int idx = (stat->fifo_index + i) % GAP_FIFO_SIZE;
				stat->best_gap_window[i] = stat->gap_fifo[idx];
			}
		}
#endif

    }
#if(CHECK_MAX_INTERVAL_SURROUNDINGS)
    stat->history_index = (stat->history_index + 1) % MAX_HISTORY_SIZE;
#endif
    state = 0;
}

// 더블 버퍼 교환
void SwapIntervalBuffers(void) {
    active_buf_index ^= 1;
}


// 최대 간격 주변의 기록된 시간 출력 (±10)
#if(CHECK_MAX_INTERVAL_SURROUNDINGS)
static void PrintAndResetHistoryAroundMax(volatile interval_stat_t* stat) {
    xil_printf("Timestamps around max interval (±10):\r\n");

    for (int offset = -10; offset <= 10; ++offset) {
        int idx = stat->max_index_in_history + offset;
        if(stat->count > idx && idx >= 0){
              u64 ts = stat->intervals_history[idx];
              xil_printf("  [%05d] %llu us\r\n", idx, TicksToUs(ts));
        }
    }
    stat->history_index = 0;
}
#endif
#if(CHECK_MAX_INTERVAL_SUM)
void PrintAndResetBestGapWindow(volatile interval_stat_t* stat) {
    if (stat->best_sum == 0) {
        xil_printf("Best 10-gap window not yet available.\r\n");
        return;
    }

    xil_printf("Best 10-gap window (max sum = %llu us):\r\n", TicksToUs(stat->best_sum));
    for (int i = 0; i < GAP_FIFO_SIZE; ++i) {
        xil_printf("  [%2d] %llu us\r\n", i, TicksToUs(stat->best_gap_window[i]));
    }
    stat->curr_sum = 0;
    stat->fifo_count = 0;
    stat->fifo_index = 0;
    stat->best_sum = 0;
}
#endif
// 특정 태그의 통계 출력 및 리셋
void PrintAndResetInterval(interval_tag_t tag) {
    int index = active_buf_index ^ 1;
    volatile interval_stat_t* stat = &intervals_buf[index][tag];

    if (tag >= INTERVAL_TAG_COUNT) {
        xil_printf("Invalid interval tag: %d\r\n", tag);
        return;
    }

    if (stat->count == 0) {
        xil_printf("%s: no data\r\n", interval_names[tag]);
        return;
    }

    u64 min_us = TicksToUs(stat->min_interval);
    u64 max_us = TicksToUs(stat->max_interval);
    u64 avg_us = TicksToUs(stat->sum_interval / stat->count);

    xil_printf("%s: interval min=%llu us, max=%llu us, avg=%llu us, count=%u\r\n",
               interval_names[tag], min_us, max_us, avg_us, stat->count);
#if(CHECK_MAX_INTERVAL_SURROUNDINGS)
    PrintAndResetHistoryAroundMax(stat);
#endif
#if(CHECK_MAX_INTERVAL_SUM)
    PrintAndResetBestGapWindow(stat);
#endif
    // 리셋
    stat->last_time = 0;
    stat->min_interval = ~(u64)0;
    stat->max_interval = 0;
    stat->sum_interval = 0;
    stat->count = 0;

}

// 모든 태그에 대해 통계 출력 및 리셋
void PrintAndResetAllIntervals(void) {
    int index = active_buf_index ^ 1;

    for (int i = 0; i < INTERVAL_TAG_COUNT; ++i) {
        volatile interval_stat_t* stat = &intervals_buf[index][i];

        if (stat->count == 0) continue;

        u64 min_us = TicksToUs(stat->min_interval);
        u64 max_us = TicksToUs(stat->max_interval);
        u64 avg_us = TicksToUs(stat->sum_interval / stat->count);

        xil_printf("%s: interval min=%llu us, max=%llu us, avg=%llu us, count=%u\r\n",
                   interval_names[i], min_us, max_us, avg_us, stat->count);
#if(CHECK_MAX_INTERVAL_SURROUNDINGS)
        PrintAndResetHistoryAroundMax(stat);
#endif
#if(CHECK_MAX_INTERVAL_SUM)
        PrintAndResetBestGapWindow(stat);
#endif
         // 리셋
        stat->last_time = 0;
        stat->min_interval = ~(u64)0;
        stat->max_interval = 0;
        stat->sum_interval = 0;
        stat->count = 0;
    }

    xil_printf("----------------------------------\r\n");
}



/*#include "isr_interval_monitor.h"
#include "xil_printf.h"
#include "debug_config.h"

volatile interval_stat_t intervals_buf[2][INTERVAL_TAG_COUNT];
volatile int active_buf_index = 0;

const char* interval_names[INTERVAL_TAG_COUNT] = {
    "DVS_DMA_DONE_ISR",
    "FIL_DMA_DONE_ISR",
};

void IntervalRecordStart(interval_tag_t tag) {
    int index = active_buf_index;
    interval_stat_t* stat = &intervals_buf[index][tag];

    static int state = 0;
    if (state == 1) {
        DEBUG_PRINT(DEBUG, "Warning: ISR interval statistics may be incomplete due to re-entrant interrupt.\r\n");
    }
    state = 1;

    XTime now;
    XTime_GetTime(&now);

    u64 last = stat->last_time;
    stat->last_time = now;

    if (last != 0) {
        u64 gap = now - last;

        if (stat->count == 0) {
            stat->min_interval = gap;
            stat->max_interval = gap;
        } else {
            if (gap < stat->min_interval) stat->min_interval = gap;
            if (gap > stat->max_interval) stat->max_interval = gap;
        }

        stat->sum_interval += gap;
        stat->count++;
    }

    state = 0;
}

void SwapIntervalBuffers(void) {
    active_buf_index ^= 1;
}

void PrintAndResetInterval(interval_tag_t tag) {
    int index = active_buf_index ^ 1;
    interval_stat_t* stat = &intervals_buf[index][tag];

    if (tag >= INTERVAL_TAG_COUNT) {
        xil_printf("Invalid interval tag: %d\r\n", tag);
        return;
    }

    if (stat->count == 0) {
        xil_printf("%s: no data\r\n", interval_names[tag]);
        return;
    }

    u64 min_us = stat->min_interval / (COUNTS_PER_SECOND / 1000000);
    u64 max_us = stat->max_interval / (COUNTS_PER_SECOND / 1000000);
    u64 avg_us = (stat->sum_interval / stat->count) / (COUNTS_PER_SECOND / 1000000);

    xil_printf("%s: interval min=%llu us, max=%llu us, avg=%llu us, count=%u\r\n",
               interval_names[tag], min_us, max_us, avg_us, stat->count);

    stat->last_time = 0;
    stat->min_interval = ~(u64)0;
    stat->max_interval = 0;
    stat->sum_interval = 0;
    stat->count = 0;
}

void PrintAndResetAllIntervals(void) {
    int index = active_buf_index ^ 1;

    for (int i = 0; i < INTERVAL_TAG_COUNT; ++i) {
        interval_stat_t* stat = &intervals_buf[index][i];

        if (stat->count == 0) continue;

        u64 min_us = stat->min_interval / (COUNTS_PER_SECOND / 1000000);
        u64 max_us = stat->max_interval / (COUNTS_PER_SECOND / 1000000);
        u64 avg_us = (stat->sum_interval / stat->count) / (COUNTS_PER_SECOND / 1000000);

        xil_printf("%s: interval min=%llu us, max=%llu us, avg=%llu us, count=%u\r\n",
                   interval_names[i], min_us, max_us, avg_us, stat->count);

        stat->last_time = 0;
        stat->min_interval = ~(u64)0;
        stat->max_interval = 0;
        stat->sum_interval = 0;
        stat->count = 0;
    }
    xil_printf("----------------------------------\r\n");
}
*/

