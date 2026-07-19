/* The 1 ms ISR only advances counters and releases cooperative task bits. */

#include <stddef.h>
#include "app_scheduler.h"
#include "fc_config.h"

#if FC_USE_STM32_HAL
#include "main.h"
#endif

#define TASK_500HZ_PERIOD_TICKS (FC_SCHEDULER_TICK_HZ / FC_CONTROL_RATE_HZ)
#define TASK_250HZ_PERIOD_TICKS (FC_SCHEDULER_TICK_HZ / FC_ATTITUDE_RATE_HZ)
#define TASK_100HZ_PERIOD_TICKS (FC_SCHEDULER_TICK_HZ / FC_RC_UPDATE_RATE_HZ)
#define TASK_50HZ_PERIOD_TICKS  (FC_SCHEDULER_TICK_HZ / FC_ALTITUDE_RATE_HZ)
#define TASK_10HZ_PERIOD_TICKS  (FC_SCHEDULER_TICK_HZ / FC_HOUSEKEEPING_RATE_HZ)

/* Written by the ISR and read by main context; volatile is intentional. */
static volatile uint32_t s_tick_ms;
static volatile uint32_t s_ready_tasks;
static volatile uint32_t s_missed_deadlines;
static volatile uint32_t s_max_main_loop_time_us;
static volatile uint16_t s_divider_500hz;
static volatile uint16_t s_divider_250hz;
static volatile uint16_t s_divider_100hz;
static volatile uint16_t s_divider_50hz;
static volatile uint16_t s_divider_10hz;
static volatile bool s_initialized;

static uint32_t enter_critical(void)
{
#if FC_USE_STM32_HAL
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    return 0U;
#endif
}

static void exit_critical(uint32_t state)
{
#if FC_USE_STM32_HAL
    if (state == 0U)
    {
        __enable_irq();
    }
#else
    (void)state;
#endif
}

static void release_task(uint32_t task)
{
    if ((s_ready_tasks & task) != 0U)
    {
        if (s_missed_deadlines != UINT32_MAX)
        {
            ++s_missed_deadlines;
        }
    }
    s_ready_tasks |= task;
}

FcStatus_t App_SchedulerInit(void)
{
    uint32_t state = enter_critical();

    s_initialized = false;
    s_tick_ms = 0U;
    s_ready_tasks = APP_SCHEDULER_TASK_NONE;
    s_missed_deadlines = 0U;
    s_max_main_loop_time_us = 0U;
    s_divider_500hz = 0U;
    s_divider_250hz = 0U;
    s_divider_100hz = 0U;
    s_divider_50hz = 0U;
    s_divider_10hz = 0U;
    s_initialized = true;

    exit_critical(state);
    return FC_STATUS_OK;
}

void App_Scheduler1msTick(void)
{
    if (!s_initialized)
    {
        return;
    }

    ++s_tick_ms;

    if (++s_divider_500hz >= TASK_500HZ_PERIOD_TICKS)
    {
        s_divider_500hz = 0U;
        release_task(APP_SCHEDULER_TASK_500HZ);
    }
    if (++s_divider_250hz >= TASK_250HZ_PERIOD_TICKS)
    {
        s_divider_250hz = 0U;
        release_task(APP_SCHEDULER_TASK_250HZ);
    }
    if (++s_divider_100hz >= TASK_100HZ_PERIOD_TICKS)
    {
        s_divider_100hz = 0U;
        release_task(APP_SCHEDULER_TASK_100HZ);
    }
    if (++s_divider_50hz >= TASK_50HZ_PERIOD_TICKS)
    {
        s_divider_50hz = 0U;
        release_task(APP_SCHEDULER_TASK_50HZ);
    }
    if (++s_divider_10hz >= TASK_10HZ_PERIOD_TICKS)
    {
        s_divider_10hz = 0U;
        release_task(APP_SCHEDULER_TASK_10HZ);
    }
}

uint32_t App_SchedulerFetchReadyTasks(void)
{
    uint32_t state;
    uint32_t tasks;

    if (!s_initialized)
    {
        return APP_SCHEDULER_TASK_NONE;
    }

    state = enter_critical();
    tasks = s_ready_tasks;
    s_ready_tasks = APP_SCHEDULER_TASK_NONE;
    exit_critical(state);
    return tasks & APP_SCHEDULER_TASK_ALL;
}

uint32_t App_SchedulerGetTickMs(void)
{
    uint32_t state = enter_critical();
    uint32_t tick_ms = s_tick_ms;
    exit_critical(state);
    return tick_ms;
}

void App_SchedulerReportMainLoopTimeUs(uint32_t elapsed_us)
{
    uint32_t state = enter_critical();

    if (elapsed_us > s_max_main_loop_time_us)
    {
        s_max_main_loop_time_us = elapsed_us;
    }
    exit_critical(state);
}

FcStatus_t App_SchedulerGetStats(AppSchedulerStats_t *stats)
{
    uint32_t state;
    bool initialized;

    if (stats == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    state = enter_critical();
    stats->tick_ms = s_tick_ms;
    stats->max_main_loop_time_us = s_max_main_loop_time_us;
    stats->missed_deadline_count = s_missed_deadlines;
    initialized = s_initialized;
    exit_critical(state);

    stats->healthy = initialized && (stats->missed_deadline_count == 0U);
    return initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}
