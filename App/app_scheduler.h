#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

/* 1 ms ISR release flags for the cooperative main-loop scheduler. */

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

typedef enum
{
    APP_SCHEDULER_TASK_NONE  = 0U,
    APP_SCHEDULER_TASK_500HZ = (1UL << 0),
    APP_SCHEDULER_TASK_250HZ = (1UL << 1),
    APP_SCHEDULER_TASK_100HZ = (1UL << 2),
    APP_SCHEDULER_TASK_50HZ  = (1UL << 3),
    APP_SCHEDULER_TASK_10HZ  = (1UL << 4),
    APP_SCHEDULER_TASK_ALL   = APP_SCHEDULER_TASK_500HZ |
                               APP_SCHEDULER_TASK_250HZ |
                               APP_SCHEDULER_TASK_100HZ |
                               APP_SCHEDULER_TASK_50HZ |
                               APP_SCHEDULER_TASK_10HZ
} AppSchedulerTask_t;

typedef struct
{
    uint32_t tick_ms;
    uint32_t max_main_loop_time_us;
    uint32_t missed_deadline_count;
    bool healthy;
} AppSchedulerStats_t;

FcStatus_t App_SchedulerInit(void);

/* Call only from the selected 1 kHz HAL timer callback. */
void App_Scheduler1msTick(void);

/* Atomically copies and clears all currently ready task bits. */
uint32_t App_SchedulerFetchReadyTasks(void);
uint32_t App_SchedulerGetTickMs(void);
void App_SchedulerReportMainLoopTimeUs(uint32_t elapsed_us);
FcStatus_t App_SchedulerGetStats(AppSchedulerStats_t *stats);

#endif /* APP_SCHEDULER_H */
