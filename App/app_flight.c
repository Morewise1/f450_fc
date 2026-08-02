/* Cooperative flight tasks and explicit fail-closed state transitions. */

#include "app_flight.h"
#include "app_safety.h"
#include "app_scheduler.h"
#include "bsp_battery_adc.h"
#include "bsp_esc_pwm.h"
#include "ctl_altitude.h"
#include "ctl_attitude.h"
#include "ctl_mixer.h"
#include "ctl_rc_map.h"
#include "ctl_rate.h"
#include "drv_bmi088.h"
#include "drv_bmp390.h"
#include "drv_ibus.h"
#include "est_altitude.h"
#include "est_attitude.h"
#include "fc_config.h"

static FcFlightState_t s_state;
static FcFlightMode_t s_mode;
static FcRcInput_t s_rc;
static FcPilotCommand_t s_pilot;
static FcImuData_t s_imu;
static FcAttitude_t s_attitude;
static FcBatteryStatus_t s_battery;
static FcAltitude_t s_altitude;
static FcVector3f_t s_target_rate_dps;
static FcControlOutput_t s_control_output;
static FcMotorOutput_t s_motor_output;
static AppFlightTaskStats_t s_task_stats;
static float s_altitude_target_m;
static float s_altitude_correction_us;
static bool s_initialized;
volatile AppFlightDebug_t g_fc_flight_debug;

static void publish_debug_snapshot(void)
{
    AppFlightDebug_t snapshot = {0};

    snapshot.imu = s_imu;
    snapshot.attitude = s_attitude;
    snapshot.receiver = s_rc;
    snapshot.pilot = s_pilot;
    snapshot.motors = s_motor_output;
    snapshot.control = s_control_output;
    snapshot.target_rate_dps = s_target_rate_dps;
    snapshot.state = s_state;
    snapshot.mode = s_mode;
    snapshot.motor_safe = App_SafetyMotorOutputAllowed();
    snapshot.publish_count = g_fc_flight_debug.publish_count + 1U;
    (void)App_SafetyGetStatus(&snapshot.safety);
    (void)Drv_Ibus_GetRawChannels(snapshot.raw_channels);
    g_fc_flight_debug = snapshot;
}

static void hold_motors_stopped(void)
{
    Ctl_MixerSetStop(&s_motor_output);
    (void)BSP_EscPwm_StopAll();
}

static void reset_control_pids(void)
{
    Ctl_RateReset();
    Ctl_AttitudeReset();
    Ctl_AltitudeReset();
    s_target_rate_dps = (FcVector3f_t){0};
    s_control_output = (FcControlOutput_t){0};
    s_altitude_correction_us = 0.0f;
}

static bool state_transition_is_allowed(FcFlightState_t current,
                                        FcFlightState_t next)
{
    if (current == next) { return true; }
    if (next == FC_STATE_STOP) { return true; }
    if ((current == FC_STATE_STOP) && (next == FC_STATE_READY)) { return true; }
    if ((current == FC_STATE_READY) && (next == FC_STATE_RUNNING)) { return true; }
    return false;
}

static bool state_entry_conditions_met(FcFlightState_t next_state)
{
    if (next_state == FC_STATE_STOP)
    {
        return true;
    }
    if (next_state == FC_STATE_READY)
    {
        return App_SafetyArmConditionsMet();
    }
    if (next_state == FC_STATE_RUNNING)
    {
        return App_SafetyMotorOutputAllowed() &&
               !s_rc.throttle_low &&
               (s_rc.throttle > FC_RC_THROTTLE_TAKEOFF_MIN);
    }
    return false;
}

static FcStatus_t reject_transition(FcStatus_t reason)
{
    reset_control_pids();
    s_state = FC_STATE_STOP;
    hold_motors_stopped();
    return reason;
}

static FcStatus_t transition_state(FcFlightState_t next_state)
{
    if ((next_state > FC_STATE_RUNNING) ||
        !state_transition_is_allowed(s_state, next_state))
    {
        return reject_transition(FC_STATUS_INVALID_DATA);
    }
    if (!state_entry_conditions_met(next_state))
    {
        return reject_transition(FC_STATUS_NOT_READY);
    }

    if (s_state == next_state)
    {
        if (next_state != FC_STATE_RUNNING)
        {
            hold_motors_stopped();
        }
        return FC_STATUS_OK;
    }

    /* Every actual state entry resets all control PID state. */
    reset_control_pids();
    s_state = next_state;
    if ((next_state == FC_STATE_RUNNING) &&
        (s_mode == FC_MODE_ALT_HOLD) && s_altitude.valid)
    {
        s_altitude_target_m = s_altitude.altitude_m;
    }

    /* READY and newly-entered RUNNING both start from a known 1000 us output. */
    hold_motors_stopped();
    return FC_STATUS_OK;
}

static void force_stop(void)
{
    (void)transition_state(FC_STATE_STOP);
    publish_debug_snapshot();
}

static FcStatus_t change_mode(FcFlightMode_t next_mode)
{
    if ((next_mode != FC_MODE_STABILIZE) && (next_mode != FC_MODE_ALT_HOLD))
    {
        return FC_STATUS_INVALID_DATA;
    }

    /* Check mode prerequisites even when the requested mode did not change. */
    if ((next_mode == FC_MODE_ALT_HOLD) && !s_altitude.valid)
    {
        return FC_STATUS_NOT_READY;
    }
    if (next_mode == s_mode)
    {
        return FC_STATUS_OK;
    }

    reset_control_pids();
    s_mode = next_mode;
    if (next_mode == FC_MODE_ALT_HOLD)
    {
        s_altitude_target_m = s_altitude.altitude_m;
    }
    else
    {
        s_altitude_target_m = 0.0f;
    }
    return FC_STATUS_OK;
}

static uint16_t build_throttle_command_us(void)
{
    float command_span = (float)(FC_ESC_COMMAND_MAX_US - FC_ESC_MIN_US);
    float throttle_us = (float)FC_ESC_MIN_US + (s_pilot.throttle * command_span);

    if (s_mode == FC_MODE_ALT_HOLD)
    {
        throttle_us += s_altitude_correction_us;
    }
    if (throttle_us < (float)FC_ESC_IDLE_US) { throttle_us = (float)FC_ESC_IDLE_US; }
    if (throttle_us > (float)FC_ESC_COMMAND_MAX_US) { throttle_us = (float)FC_ESC_COMMAND_MAX_US; }
    return (uint16_t)(throttle_us + 0.5f);
}

FcStatus_t App_FlightInit(void)
{
    s_initialized = false;
    s_state = FC_STATE_STOP;
    s_mode = FC_MODE_STABILIZE;
    s_rc = (FcRcInput_t){0};
    s_pilot = (FcPilotCommand_t){0};
    s_imu = (FcImuData_t){0};
    s_attitude = (FcAttitude_t){0};
    s_battery = (FcBatteryStatus_t){0};
    s_altitude = (FcAltitude_t){0};
    s_target_rate_dps = (FcVector3f_t){0};
    s_control_output = (FcControlOutput_t){0};
    s_task_stats = (AppFlightTaskStats_t){0};
    s_altitude_target_m = 0.0f;
    s_altitude_correction_us = 0.0f;
    Ctl_MixerSetStop(&s_motor_output);
    g_fc_flight_debug = (AppFlightDebug_t){0};
    (void)BSP_EscPwm_StopAll();
    s_initialized = true;
    publish_debug_snapshot();
    return FC_STATUS_OK;
}

void App_FlightTask500Hz(void)
{
    FcStatus_t status;

    if (!s_initialized)
    {
        return;
    }
    ++s_task_stats.task_500hz_count;

    (void)Drv_Bmi088_SetBiasTrackingEnabled(s_state == FC_STATE_STOP);
    status = Drv_Bmi088_Read(&s_imu);
    if ((status != FC_STATUS_OK) || !s_imu.valid)
    {
        s_imu.valid = false;
        force_stop();
        return;
    }

    if (s_state != FC_STATE_RUNNING)
    {
        hold_motors_stopped();
        publish_debug_snapshot();
        return;
    }
    if (!App_SafetyMotorOutputAllowed())
    {
        force_stop();
        return;
    }
    if (BSP_EscPwm_SetOutputEnabled(true) != FC_STATUS_OK)
    {
        force_stop();
        return;
    }

    status = Ctl_RateUpdate(&s_target_rate_dps, &s_imu, FC_CONTROL_DT_S, &s_control_output);
    if ((status != FC_STATUS_OK) || !s_control_output.valid)
    {
        force_stop();
        return;
    }

    status = Ctl_MixerQuadX(build_throttle_command_us(),
                            s_control_output.roll_cmd_us,
                            s_control_output.pitch_cmd_us,
                            s_control_output.yaw_cmd_us,
                            &s_motor_output);
    if ((status != FC_STATUS_OK) || !s_motor_output.valid)
    {
        force_stop();
        return;
    }
    if (BSP_EscPwm_WriteAll(&s_motor_output) != FC_STATUS_OK)
    {
        force_stop();
        return;
    }
    publish_debug_snapshot();
}

void App_FlightTask250Hz(void)
{
    FcControlTarget_t target = {0};

    if (!s_initialized)
    {
        return;
    }
    ++s_task_stats.task_250hz_count;

    if ((Est_AttitudeUpdate(&s_imu, FC_ATTITUDE_DT_S, &s_attitude) != FC_STATUS_OK) ||
        !s_attitude.valid)
    {
        s_attitude.valid = false;
        force_stop();
        return;
    }

    if (s_state != FC_STATE_RUNNING)
    {
        s_target_rate_dps = (FcVector3f_t){0};
        publish_debug_snapshot();
        return;
    }

    target.throttle_us = build_throttle_command_us();
    target.roll_deg = s_pilot.roll * FC_MAX_TARGET_TILT_DEG;
    target.pitch_deg = s_pilot.pitch * FC_MAX_TARGET_TILT_DEG;
    target.yaw_rate_dps = s_pilot.yaw * FC_MAX_TARGET_YAW_RATE_DPS;
    target.altitude_m = s_altitude_target_m;
    target.mode = s_mode;

    if (Ctl_AttitudeUpdate(&target, &s_attitude, FC_ATTITUDE_DT_S, &s_target_rate_dps) != FC_STATUS_OK)
    {
        s_target_rate_dps = (FcVector3f_t){0};
        force_stop();
        return;
    }
    publish_debug_snapshot();
}

void App_FlightTask100Hz(void)
{
    AppSchedulerStats_t scheduler_stats = {0};
    FcFlightMode_t requested_mode;
    uint32_t now_ms;
    bool scheduler_ok;

    if (!s_initialized)
    {
        return;
    }
    ++s_task_stats.task_100hz_count;

    now_ms = App_SchedulerGetTickMs();
    Drv_Ibus_UpdateTimeout(now_ms);
    if (Drv_Ibus_GetInput(&s_rc) != FC_STATUS_OK)
    {
        s_rc = (FcRcInput_t){0};
        s_rc.failsafe = true;
    }
    if (Ctl_RcMapUpdate(&s_rc, &s_pilot) != FC_STATUS_OK)
    {
        s_pilot = (FcPilotCommand_t){0};
    }
#if FC_ENABLE_BATTERY_MONITOR
    if (BSP_BatteryAdc_Read(&s_battery, now_ms) != FC_STATUS_OK)
    {
        s_battery.valid = false;
        s_battery.critical = true;
    }
#else
    s_battery = (FcBatteryStatus_t){0};
    s_battery.timestamp_ms = now_ms;
#endif

    scheduler_ok = (App_SchedulerGetStats(&scheduler_stats) == FC_STATUS_OK) &&
                   scheduler_stats.healthy;
    App_SafetyEvaluate(&s_rc, &s_imu, &s_attitude, &s_battery, scheduler_ok);
    if (!App_SafetyMotorOutputAllowed() || !s_pilot.valid || !s_pilot.motor_safe)
    {
        force_stop();
        return;
    }

    requested_mode = s_rc.mode_switch ? FC_MODE_ALT_HOLD : FC_MODE_STABILIZE;
    if (change_mode(requested_mode) != FC_STATUS_OK)
    {
        force_stop();
        return;
    }

    switch (s_state)
    {
        case FC_STATE_STOP:
            if (App_SafetyArmConditionsMet())
            {
                if (transition_state(FC_STATE_READY) != FC_STATUS_OK) { force_stop(); }
            }
            else
            {
                hold_motors_stopped();
            }
            break;

        case FC_STATE_READY:
            if (!s_rc.throttle_low &&
                (s_rc.throttle > FC_RC_THROTTLE_TAKEOFF_MIN))
            {
                if (transition_state(FC_STATE_RUNNING) != FC_STATUS_OK) { force_stop(); }
            }
            else
            {
                hold_motors_stopped();
            }
            break;

        case FC_STATE_RUNNING:
            break;

        default:
            force_stop();
            break;
    }
    publish_debug_snapshot();
}

void App_FlightTask50Hz(void)
{
    FcBarometerData_t barometer = {0};
    FcStatus_t barometer_status;
    FcStatus_t estimator_status;

    if (!s_initialized)
    {
        return;
    }
    ++s_task_stats.task_50hz_count;

    barometer_status = Drv_Bmp390_Read(&barometer, App_SchedulerGetTickMs());
    estimator_status = Est_AltitudeUpdate(&barometer, NULL, FC_ALTITUDE_DT_S, &s_altitude);
    if ((barometer_status != FC_STATUS_OK) ||
        (estimator_status != FC_STATUS_OK) || !s_altitude.valid)
    {
        s_altitude.valid = false;
        s_altitude_correction_us = 0.0f;
        if (s_mode == FC_MODE_ALT_HOLD)
        {
            force_stop();
        }
        return;
    }

    if ((s_mode == FC_MODE_ALT_HOLD) && (s_state == FC_STATE_RUNNING))
    {
        if (Ctl_AltitudeUpdate(s_altitude_target_m,
                               &s_altitude,
                               FC_ALTITUDE_DT_S,
                               &s_altitude_correction_us) != FC_STATUS_OK)
        {
            s_altitude_correction_us = 0.0f;
            force_stop();
        }
    }
    else
    {
        s_altitude_correction_us = 0.0f;
    }
}

void App_FlightTask10Hz(void)
{
    if (!s_initialized)
    {
        return;
    }
    ++s_task_stats.task_10hz_count;
    publish_debug_snapshot();
    /* Reserved for bounded housekeeping and deferred log snapshots. */
}

FcFlightState_t App_FlightGetState(void)
{
    return s_state;
}

FcFlightMode_t App_FlightGetMode(void)
{
    return s_mode;
}

FcStatus_t App_FlightGetMotorOutput(FcMotorOutput_t *output)
{
    if (output == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *output = s_motor_output;
    return s_initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}

FcStatus_t App_FlightGetTaskStats(AppFlightTaskStats_t *stats)
{
    if (stats == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *stats = s_task_stats;
    return s_initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}

FcStatus_t App_FlightGetDebug(AppFlightDebug_t *debug)
{
    uint32_t index;

    if (debug == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    debug->imu = g_fc_flight_debug.imu;
    debug->attitude = g_fc_flight_debug.attitude;
    debug->receiver = g_fc_flight_debug.receiver;
    debug->pilot = g_fc_flight_debug.pilot;
    debug->motors = g_fc_flight_debug.motors;
    debug->safety = g_fc_flight_debug.safety;
    debug->control = g_fc_flight_debug.control;
    debug->target_rate_dps = g_fc_flight_debug.target_rate_dps;
    for (index = 0U; index < FC_IBUS_CHANNEL_COUNT; ++index)
    {
        debug->raw_channels[index] = g_fc_flight_debug.raw_channels[index];
    }
    debug->state = g_fc_flight_debug.state;
    debug->mode = g_fc_flight_debug.mode;
    debug->motor_safe = g_fc_flight_debug.motor_safe;
    debug->publish_count = g_fc_flight_debug.publish_count;
    return s_initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}
