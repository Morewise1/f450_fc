/* Host test for 1 ms release rates, atomic fetch semantics, and overruns. */

#include <stdint.h>
#include "app_scheduler.h"

int main(void)
{
    AppSchedulerStats_t stats;
    uint32_t count_500hz = 0U;
    uint32_t count_250hz = 0U;
    uint32_t count_100hz = 0U;
    uint32_t count_50hz = 0U;
    uint32_t count_10hz = 0U;
    uint32_t tick;

    if (App_SchedulerGetStats(0) != FC_STATUS_INVALID_ARGUMENT) { return 1; }
    if (App_SchedulerInit() != FC_STATUS_OK) { return 2; }

    for (tick = 0U; tick < 100U; ++tick)
    {
        uint32_t tasks;
        App_Scheduler1msTick();
        tasks = App_SchedulerFetchReadyTasks();
        if ((tasks & APP_SCHEDULER_TASK_500HZ) != 0U) { ++count_500hz; }
        if ((tasks & APP_SCHEDULER_TASK_250HZ) != 0U) { ++count_250hz; }
        if ((tasks & APP_SCHEDULER_TASK_100HZ) != 0U) { ++count_100hz; }
        if ((tasks & APP_SCHEDULER_TASK_50HZ) != 0U) { ++count_50hz; }
        if ((tasks & APP_SCHEDULER_TASK_10HZ) != 0U) { ++count_10hz; }
        if ((tasks & ~((uint32_t)APP_SCHEDULER_TASK_ALL)) != 0U) { return 3; }
    }

    if ((count_500hz != 50U) || (count_250hz != 25U) ||
        (count_100hz != 10U) || (count_50hz != 5U) ||
        (count_10hz != 1U)) { return 4; }
    if (App_SchedulerGetTickMs() != 100U) { return 5; }
    if (App_SchedulerFetchReadyTasks() != APP_SCHEDULER_TASK_NONE) { return 6; }

    App_SchedulerReportMainLoopTimeUs(80U);
    App_SchedulerReportMainLoopTimeUs(125U);
    App_SchedulerReportMainLoopTimeUs(100U);
    if (App_SchedulerGetStats(&stats) != FC_STATUS_OK) { return 7; }
    if (!stats.healthy || (stats.missed_deadline_count != 0U) ||
        (stats.max_main_loop_time_us != 125U)) { return 8; }

    if (App_SchedulerInit() != FC_STATUS_OK) { return 9; }
    for (tick = 0U; tick < 4U; ++tick)
    {
        App_Scheduler1msTick();
    }
    if (App_SchedulerGetStats(&stats) != FC_STATUS_OK) { return 10; }
    if (stats.healthy || (stats.missed_deadline_count != 1U)) { return 11; }
    if (App_SchedulerFetchReadyTasks() !=
        (APP_SCHEDULER_TASK_500HZ | APP_SCHEDULER_TASK_250HZ)) { return 12; }
    if (App_SchedulerFetchReadyTasks() != APP_SCHEDULER_TASK_NONE) { return 13; }
    return 0;
}
