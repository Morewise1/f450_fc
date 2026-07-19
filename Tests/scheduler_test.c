/* Host test for periodic flags, atomic fetch semantics, and deadline health. */

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
    uint32_t millisecond;

    App_Scheduler1msTick();
    if (App_SchedulerFetchReadyTasks() != APP_SCHEDULER_TASK_NONE) { return 1; }
    if (App_SchedulerGetTickMs() != 0U) { return 2; }
    if (App_SchedulerInit() != FC_STATUS_OK) { return 3; }

    for (millisecond = 0U; millisecond < 100U; ++millisecond)
    {
        uint32_t tasks;
        App_Scheduler1msTick();
        tasks = App_SchedulerFetchReadyTasks();
        if ((tasks & ~((uint32_t)APP_SCHEDULER_TASK_ALL)) != 0U) { return 4; }
        if ((tasks & APP_SCHEDULER_TASK_500HZ) != 0U) { ++count_500hz; }
        if ((tasks & APP_SCHEDULER_TASK_250HZ) != 0U) { ++count_250hz; }
        if ((tasks & APP_SCHEDULER_TASK_100HZ) != 0U) { ++count_100hz; }
        if ((tasks & APP_SCHEDULER_TASK_50HZ) != 0U) { ++count_50hz; }
        if ((tasks & APP_SCHEDULER_TASK_10HZ) != 0U) { ++count_10hz; }
    }

    if ((count_500hz != 50U) || (count_250hz != 25U) ||
        (count_100hz != 10U) || (count_50hz != 5U) ||
        (count_10hz != 1U)) { return 5; }
    if (App_SchedulerFetchReadyTasks() != APP_SCHEDULER_TASK_NONE) { return 6; }
    if (App_SchedulerGetTickMs() != 100U) { return 7; }

    App_SchedulerReportMainLoopTimeUs(123U);
    App_SchedulerReportMainLoopTimeUs(50U);
    if (App_SchedulerGetStats(&stats) != FC_STATUS_OK) { return 8; }
    if (!stats.healthy || (stats.max_main_loop_time_us != 123U)) { return 9; }

    /* Leave one 500 Hz bit pending until the next release of the same bit. */
    App_Scheduler1msTick();
    App_Scheduler1msTick();
    App_Scheduler1msTick();
    App_Scheduler1msTick();
    if (App_SchedulerGetStats(&stats) != FC_STATUS_OK) { return 10; }
    if (stats.healthy || (stats.missed_deadline_count != 1U)) { return 11; }
    if ((App_SchedulerFetchReadyTasks() & APP_SCHEDULER_TASK_500HZ) == 0U) { return 12; }
    if (App_SchedulerGetStats(NULL) != FC_STATUS_INVALID_ARGUMENT) { return 13; }
    return 0;
}
