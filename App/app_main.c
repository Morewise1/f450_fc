/* Initializes all layers and dispatches task flags from the main loop. */

#include "app_main.h"
#include "app_flight.h"
#include "app_safety.h"
#include "app_scheduler.h"
#include "bsp_battery_adc.h"
#include "bsp_buzzer.h"
#include "bsp_debug_uart.h"
#include "bsp_esc_pwm.h"
#include "bsp_led.h"
#include "ctl_altitude.h"
#include "ctl_attitude.h"
#include "ctl_rate.h"
#include "drv_bmp390.h"
#include "drv_ibus.h"
#include "drv_qmi8658.h"
#include "drv_vl53l1x.h"
#include "est_altitude.h"
#include "est_attitude.h"

static FcStatus_t s_status = FC_STATUS_NOT_INITIALIZED;
static bool s_loop_enabled;

static void record_failure(FcStatus_t candidate)
{
    if ((candidate != FC_STATUS_OK) && (s_status == FC_STATUS_OK))
    {
        s_status = candidate;
    }
}

FcStatus_t App_MainInit(void)
{
    FcStatus_t imu_status;

    s_status = FC_STATUS_OK;
    s_loop_enabled = false;

    record_failure(App_SchedulerInit());
    record_failure(App_SafetyInit());
    record_failure(App_FlightInit());

    record_failure(BSP_EscPwm_Init());
    (void)BSP_EscPwm_StopAll();

    (void)BSP_Led_Init();
    (void)BSP_Buzzer_Init();
    record_failure(BSP_BatteryAdc_Init());
    (void)BSP_DebugUart_Init();

    record_failure(Drv_Ibus_Init());
    imu_status = Drv_Qmi8658_Init();
    record_failure(imu_status);
    if (imu_status == FC_STATUS_OK)
    {
        record_failure(Drv_Qmi8658_CalibrateGyro());
    }
    (void)Drv_Bmp390_Init();
    (void)Drv_Vl53l1x_Init();

    record_failure(Est_AttitudeInit());
    (void)Est_AltitudeInit();
    record_failure(Ctl_RateInit());
    record_failure(Ctl_AttitudeInit());
    (void)Ctl_AltitudeInit();

    App_SafetySetInitializationResult(s_status == FC_STATUS_OK);
    s_loop_enabled = true;
    return s_status;
}

void App_MainLoop(void)
{
    uint32_t tasks;

    if (!s_loop_enabled)
    {
        (void)BSP_EscPwm_StopAll();
        return;
    }

    tasks = App_SchedulerFetchReadyTasks();

    /* Safety/state updates precede control output released on the same tick. */
    if ((tasks & APP_SCHEDULER_TASK_100HZ) != 0U) { App_FlightTask100Hz(); }
    if ((tasks & APP_SCHEDULER_TASK_50HZ) != 0U) { App_FlightTask50Hz(); }
    if ((tasks & APP_SCHEDULER_TASK_250HZ) != 0U) { App_FlightTask250Hz(); }
    if ((tasks & APP_SCHEDULER_TASK_500HZ) != 0U) { App_FlightTask500Hz(); }
    if ((tasks & APP_SCHEDULER_TASK_10HZ) != 0U) { App_FlightTask10Hz(); }
}

FcStatus_t App_MainGetStatus(void)
{
    return s_status;
}

FcStatus_t App_Init(void)
{
    return App_MainInit();
}

void App_Loop(void)
{
    App_MainLoop();
}
