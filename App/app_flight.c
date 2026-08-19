/* Cooperative flight tasks and explicit fail-closed state transitions. */

#include <math.h>
#include "app_flight.h"
#include "app_command_mux.h"
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
#include "drv_bmp388.h"
#include "drv_ibus.h"
#include "drv_mmc5983ma.h"
#include "est_altitude.h"
#include "est_attitude.h"
#include "fc_config.h"
#include "fc_params.h"

static FcFlightState_t s_state;
static FcFlightMode_t s_mode;
static FcRcInput_t s_rc;
static FcPilotCommand_t s_pilot;
static FcImuData_t s_imu;
static FcAttitude_t s_attitude;
static FcBarometerData_t s_barometer;
static FcMagnetometerData_t s_magnetometer;
static FcBatteryStatus_t s_battery;
static FcAltitude_t s_altitude;
static FcVector3f_t s_target_rate_dps;
static FcControlOutput_t s_control_output;
static FcMotorOutput_t s_motor_output;
static AppFlightTaskStats_t s_task_stats;
static float s_altitude_target_m;
static float s_altitude_correction_us;
static AppAltitudeHoldPhase_t s_altitude_hold_phase;
static bool s_altitude_throttle_captured;
static bool s_altitude_entry_blend_active;
static uint16_t s_altitude_entry_blend_start_us;
static uint16_t s_altitude_exit_blend_start_us;
static uint32_t s_altitude_entry_blend_start_ms;
static uint32_t s_altitude_exit_blend_start_ms;
static AppTakeoffPhase_t s_takeoff_phase;
static uint32_t s_takeoff_motion_start_ms;
static uint32_t s_takeoff_abort_start_ms;
static uint32_t s_airborne_confirmed_ms;
static bool s_takeoff_motion_timing;
static bool s_takeoff_abort_timing;
static bool s_initialized;
static bool s_altitude_hold_fault_latched;
volatile AppFlightDebug_t g_fc_flight_debug;

static uint16_t build_throttle_command_us(void);

static void reset_takeoff_runtime(void)
{
    s_takeoff_phase = APP_TAKEOFF_PHASE_GROUNDED;
    s_takeoff_motion_start_ms = 0U;
    s_takeoff_abort_start_ms = 0U;
    s_airborne_confirmed_ms = 0U;
    s_takeoff_motion_timing = false;
    s_takeoff_abort_timing = false;
}

static bool altitude_ground_constraint_active(void)
{
    return s_takeoff_phase == APP_TAKEOFF_PHASE_GROUNDED;
}

static float current_barometer_noise_scale(uint32_t now_ms)
{
    if (s_takeoff_phase == APP_TAKEOFF_PHASE_PENDING)
    {
        return FC_TAKEOFF_BARO_STD_SCALE;
    }
    if ((s_takeoff_phase == APP_TAKEOFF_PHASE_AIRBORNE) &&
        ((now_ms - s_airborne_confirmed_ms) < FC_TAKEOFF_BARO_RECOVERY_MS))
    {
        return FC_TAKEOFF_BARO_STD_SCALE;
    }
    return 1.0f;
}

static void update_takeoff_phase(uint32_t now_ms)
{
    uint16_t throttle_us;
    bool upward_motion;
    bool near_ground_still;

    if (s_state != FC_STATE_RUNNING)
    {
        reset_takeoff_runtime();
        return;
    }
    if (s_takeoff_phase == APP_TAKEOFF_PHASE_AIRBORNE)
    {
        /* 空中状态锁存到停机；不能因收油或气压漂移而重新施加地面零约束。 */
        return;
    }

    throttle_us = build_throttle_command_us();
    if (s_takeoff_phase == APP_TAKEOFF_PHASE_GROUNDED)
    {
        if (throttle_us >= FC_TAKEOFF_REFERENCE_FREEZE_US)
        {
            s_takeoff_phase = APP_TAKEOFF_PHASE_PENDING;
            s_takeoff_motion_timing = false;
            s_takeoff_abort_timing = false;
        }
        return;
    }

    upward_motion = s_altitude.valid &&
        (s_altitude.altitude_m >= FC_TAKEOFF_MIN_ALTITUDE_M) &&
        (s_altitude.vertical_velocity_mps >=
         FC_TAKEOFF_MIN_VERTICAL_VELOCITY_MPS);
    if ((throttle_us >= FC_TAKEOFF_AIRBORNE_THRUST_US) && upward_motion)
    {
        if (!s_takeoff_motion_timing)
        {
            s_takeoff_motion_start_ms = now_ms;
            s_takeoff_motion_timing = true;
        }
        if ((now_ms - s_takeoff_motion_start_ms) >=
            FC_TAKEOFF_MOTION_CONFIRM_MS)
        {
            s_takeoff_phase = APP_TAKEOFF_PHASE_AIRBORNE;
            s_airborne_confirmed_ms = now_ms;
            s_takeoff_abort_timing = false;
            return;
        }
    }
    else
    {
        s_takeoff_motion_timing = false;
    }

    near_ground_still = s_altitude.valid &&
        (fabsf(s_altitude.altitude_m) <=
         FC_TAKEOFF_ABORT_MAX_ALTITUDE_M) &&
        (fabsf(s_altitude.vertical_velocity_mps) <=
         FC_TAKEOFF_ABORT_MAX_VERTICAL_VELOCITY_MPS);
    if ((throttle_us < FC_TAKEOFF_PENDING_ABORT_US) && near_ground_still)
    {
        if (!s_takeoff_abort_timing)
        {
            s_takeoff_abort_start_ms = now_ms;
            s_takeoff_abort_timing = true;
        }
        if ((now_ms - s_takeoff_abort_start_ms) >=
            FC_TAKEOFF_PENDING_ABORT_MS)
        {
            reset_takeoff_runtime();
        }
    }
    else
    {
        s_takeoff_abort_timing = false;
    }
}

static uint16_t build_manual_throttle_command_us(void)
{
    float command_span = (float)(FC_ESC_COMMAND_MAX_US - FC_ESC_MIN_US);
    float throttle_us = (float)FC_ESC_MIN_US + (s_pilot.throttle * command_span);

    if (throttle_us < (float)FC_ESC_IDLE_US) { throttle_us = (float)FC_ESC_IDLE_US; }
    if (throttle_us > (float)FC_ESC_COMMAND_MAX_US) { throttle_us = (float)FC_ESC_COMMAND_MAX_US; }
    return (uint16_t)(throttle_us + 0.5f);
}

static uint16_t clamp_throttle_us(float throttle_us)
{
    if (throttle_us < (float)FC_ESC_IDLE_US) { throttle_us = (float)FC_ESC_IDLE_US; }
    if (throttle_us > (float)FC_ESC_COMMAND_MAX_US) { throttle_us = (float)FC_ESC_COMMAND_MAX_US; }
    return (uint16_t)(throttle_us + 0.5f);
}

static uint16_t build_altitude_hold_throttle_command_us(void)
{
    return clamp_throttle_us((float)FC_HOVER_FEEDFORWARD_US +
                             s_altitude_correction_us);
}

static bool throttle_is_in_altitude_capture_window(uint16_t throttle_us)
{
    return (throttle_us >= FC_ALT_HOLD_STICK_CAPTURE_MIN_US) &&
           (throttle_us <= FC_ALT_HOLD_STICK_CAPTURE_MAX_US);
}

static uint16_t throttle_difference_us(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static float handover_progress(uint32_t now_ms, uint32_t start_ms)
{
    uint32_t elapsed_ms = now_ms - start_ms;
    if (elapsed_ms >= FC_ALT_HOLD_HANDOVER_BLEND_TIME_MS) { return 1.0f; }
    return (float)elapsed_ms / (float)FC_ALT_HOLD_HANDOVER_BLEND_TIME_MS;
}

static uint16_t blend_throttle_us(uint16_t from_us, uint16_t to_us, float alpha)
{
    float blended;

    if (alpha <= 0.0f) { return from_us; }
    if (alpha >= 1.0f) { return to_us; }
    blended = (float)from_us + ((float)to_us - (float)from_us) * alpha;
    return clamp_throttle_us(blended);
}

static void reset_altitude_hold_runtime(void)
{
    s_altitude_hold_phase = APP_ALT_HOLD_PHASE_INACTIVE;
    s_altitude_throttle_captured = false;
    s_altitude_entry_blend_active = false;
    s_altitude_entry_blend_start_us = FC_HOVER_FEEDFORWARD_US;
    s_altitude_exit_blend_start_us = FC_HOVER_FEEDFORWARD_US;
    s_altitude_entry_blend_start_ms = 0U;
    s_altitude_exit_blend_start_ms = 0U;
}

static void publish_debug_snapshot(void)
{
    /* 调试快照较大，放静态区可避免占用启动文件中仅1KB的主栈。 */
    static AppFlightDebug_t snapshot;

    snapshot.imu = s_imu;
    snapshot.attitude = s_attitude;
    snapshot.barometer = s_barometer;
    snapshot.magnetometer = s_magnetometer;
    snapshot.altitude = s_altitude;
    snapshot.receiver = s_rc;
    snapshot.pilot = s_pilot;
    snapshot.motors = s_motor_output;
    snapshot.control = s_control_output;
    snapshot.target_rate_dps = s_target_rate_dps;
    snapshot.state = s_state;
    snapshot.mode = s_mode;
    snapshot.takeoff_phase = s_takeoff_phase;
    snapshot.airborne = s_takeoff_phase == APP_TAKEOFF_PHASE_AIRBORNE;
    snapshot.barometer_noise_scale = current_barometer_noise_scale(
        App_SchedulerGetTickMs());
    snapshot.altitude_hold_phase = s_altitude_hold_phase;
    snapshot.altitude_throttle_captured = s_altitude_throttle_captured;
    snapshot.altitude_entry_blend_active = s_altitude_entry_blend_active;
    snapshot.altitude_target_m = s_altitude_target_m;
    snapshot.altitude_correction_us = s_altitude_correction_us;
    snapshot.manual_throttle_command_us = build_manual_throttle_command_us();
    snapshot.automatic_throttle_command_us =
        build_altitude_hold_throttle_command_us();
    snapshot.throttle_command_us = build_throttle_command_us();
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
    reset_altitude_hold_runtime();
    reset_takeoff_runtime();
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
    s_mode = FC_MODE_STABILIZE;
    s_altitude_target_m = 0.0f;
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
    if (next_state == FC_STATE_STOP)
    {
        s_mode = FC_MODE_STABILIZE;
        s_altitude_target_m = 0.0f;
    }
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

    if ((next_mode == FC_MODE_ALT_HOLD) && !s_altitude.valid)
    {
        return FC_STATUS_NOT_READY;
    }
    if (next_mode == s_mode)
    {
        return FC_STATUS_OK;
    }

    /* 垂直模式切换不重置姿态/角速度环，避免集体油门交接引入姿态突跳。 */
    Ctl_AltitudeReset();
    s_altitude_correction_us = 0.0f;
    if (next_mode == FC_MODE_ALT_HOLD)
    {
        s_altitude_target_m = s_altitude.altitude_m;
        s_altitude_throttle_captured = false;
        s_altitude_hold_phase = APP_ALT_HOLD_PHASE_WAIT_CAPTURE;
        s_altitude_entry_blend_start_us = build_manual_throttle_command_us();
        s_altitude_entry_blend_start_ms = App_SchedulerGetTickMs();
        s_altitude_entry_blend_active = true;
    }
    else
    {
        s_altitude_target_m = 0.0f;
        reset_altitude_hold_runtime();
    }
    s_mode = next_mode;
    return FC_STATUS_OK;
}

static uint16_t build_throttle_command_us(void)
{
    uint16_t throttle_us;

    if (s_mode == FC_MODE_ALT_HOLD)
    {
        uint16_t automatic_us = build_altitude_hold_throttle_command_us();
        uint16_t manual_us = build_manual_throttle_command_us();
        uint32_t now_ms = App_SchedulerGetTickMs();

        if (s_altitude_hold_phase == APP_ALT_HOLD_PHASE_EXIT_BLEND)
        {
            throttle_us = blend_throttle_us(
                s_altitude_exit_blend_start_us,
                manual_us,
                handover_progress(now_ms, s_altitude_exit_blend_start_ms));
        }
        else if (s_altitude_entry_blend_active)
        {
            throttle_us = blend_throttle_us(
                s_altitude_entry_blend_start_us,
                automatic_us,
                handover_progress(now_ms, s_altitude_entry_blend_start_ms));
        }
        else
        {
            throttle_us = automatic_us;
        }
    }
    else
    {
        throttle_us = build_manual_throttle_command_us();
    }
    return throttle_us;
}

static void begin_manual_handover(void)
{
    if ((s_altitude_hold_phase != APP_ALT_HOLD_PHASE_EXIT_WAIT) &&
        (s_altitude_hold_phase != APP_ALT_HOLD_PHASE_EXIT_BLEND))
    {
        s_altitude_hold_phase = APP_ALT_HOLD_PHASE_EXIT_WAIT;
    }
}

static void cancel_manual_handover(uint32_t now_ms)
{
    if (s_altitude_hold_phase == APP_ALT_HOLD_PHASE_EXIT_BLEND)
    {
        /* CH6重新拨高时，从当前总油门平滑恢复自动定高。 */
        s_altitude_entry_blend_start_us = build_throttle_command_us();
        s_altitude_entry_blend_start_ms = now_ms;
        s_altitude_entry_blend_active = true;
    }
    s_altitude_hold_phase = s_altitude_throttle_captured ?
                            APP_ALT_HOLD_PHASE_ACTIVE :
                            APP_ALT_HOLD_PHASE_WAIT_CAPTURE;
}

static FcStatus_t update_altitude_mode_handover(uint32_t now_ms)
{
    uint16_t manual_us;

    if (s_mode == FC_MODE_STABILIZE)
    {
        if (s_rc.mode_switch && !s_altitude_hold_fault_latched &&
            (s_state == FC_STATE_RUNNING) && s_altitude.valid)
        {
            return change_mode(FC_MODE_ALT_HOLD);
        }
        return FC_STATUS_OK;
    }
    if (s_mode != FC_MODE_ALT_HOLD) { return FC_STATUS_INVALID_DATA; }

    if (s_altitude_entry_blend_active &&
        (handover_progress(now_ms, s_altitude_entry_blend_start_ms) >= 1.0f))
    {
        s_altitude_entry_blend_active = false;
    }
    if (s_rc.mode_switch)
    {
        if ((s_altitude_hold_phase == APP_ALT_HOLD_PHASE_EXIT_WAIT) ||
            (s_altitude_hold_phase == APP_ALT_HOLD_PHASE_EXIT_BLEND))
        {
            cancel_manual_handover(now_ms);
        }
        return FC_STATUS_OK;
    }

    /* CH6拨低只请求交权；油门未回捕获区前继续保持定高。 */
    begin_manual_handover();
    manual_us = build_manual_throttle_command_us();
    if (s_altitude_hold_phase == APP_ALT_HOLD_PHASE_EXIT_WAIT)
    {
        uint16_t automatic_us;

        if (!throttle_is_in_altitude_capture_window(manual_us))
        {
            return FC_STATUS_OK;
        }
        automatic_us = build_throttle_command_us();
        if (throttle_difference_us(automatic_us, manual_us) <=
            FC_ALT_HOLD_HANDOVER_DIRECT_TOLERANCE_US)
        {
            return change_mode(FC_MODE_STABILIZE);
        }
        s_altitude_exit_blend_start_us = automatic_us;
        s_altitude_exit_blend_start_ms = now_ms;
        s_altitude_entry_blend_active = false;
        s_altitude_hold_phase = APP_ALT_HOLD_PHASE_EXIT_BLEND;
        return FC_STATUS_OK;
    }
    if ((s_altitude_hold_phase == APP_ALT_HOLD_PHASE_EXIT_BLEND) &&
        (handover_progress(now_ms, s_altitude_exit_blend_start_ms) >= 1.0f))
    {
        return change_mode(FC_MODE_STABILIZE);
    }
    return FC_STATUS_OK;
}

FcStatus_t App_FlightInit(void)
{
    s_initialized = false;
    s_altitude_hold_fault_latched = false;
    s_state = FC_STATE_STOP;
    s_mode = FC_MODE_STABILIZE;
    s_rc = (FcRcInput_t){0};
    s_pilot = (FcPilotCommand_t){0};
    s_imu = (FcImuData_t){0};
    s_attitude = (FcAttitude_t){0};
    s_barometer = (FcBarometerData_t){0};
    s_magnetometer = (FcMagnetometerData_t){0};
    s_battery = (FcBatteryStatus_t){0};
    s_altitude = (FcAltitude_t){0};
    s_target_rate_dps = (FcVector3f_t){0};
    s_control_output = (FcControlOutput_t){0};
    s_task_stats = (AppFlightTaskStats_t){0};
    s_altitude_target_m = 0.0f;
    s_altitude_correction_us = 0.0f;
    reset_altitude_hold_runtime();
    reset_takeoff_runtime();
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
    uint32_t now_ms;

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

    /*
     * In Kalman mode this is the high-rate inertial prediction.  In the
     * complementary fallback mode the call only republishes the latest 50 Hz
     * state, so one application task layout safely supports both estimators.
     */
    now_ms = App_SchedulerGetTickMs();
    (void)Est_AltitudePredict(&s_imu,
                              &s_attitude,
                              FC_ATTITUDE_DT_S,
                              now_ms,
                              altitude_ground_constraint_active(),
                              &s_altitude);

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
    FcRcInput_t ibus_input = {0};
    uint32_t now_ms;
    bool scheduler_ok;

    if (!s_initialized)
    {
        return;
    }
    ++s_task_stats.task_100hz_count;

    now_ms = App_SchedulerGetTickMs();
    /* MMC5983MA continuous conversion is configured for 100 Hz.  Read it in
     * the matching task so the attitude estimator receives each fresh sample
     * once; the BMP388 and altitude loop remain at their noise-friendly 50 Hz.
     */
    if (Drv_Mmc5983ma_Read(&s_magnetometer, now_ms) == FC_STATUS_OK)
    {
        (void)Est_AttitudeSetMagnetometer(&s_magnetometer);
    }
    Drv_Ibus_UpdateTimeout(now_ms);
    if (Drv_Ibus_GetInput(&ibus_input) != FC_STATUS_OK)
    {
        ibus_input = (FcRcInput_t){0};
        ibus_input.failsafe = true;
    }
    App_CommandMuxUpdateIbus(&ibus_input);
    if (App_CommandMuxGetInput(now_ms, &s_rc) != FC_STATUS_OK)
    {
        s_rc.link_valid = false;
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

    /* Pulling the switch low acknowledges an altitude-hold sensor fault. */
    if (!s_rc.mode_switch) { s_altitude_hold_fault_latched = false; }
    /* 定高模式只允许在飞行中进入，并在进入/退出时完成油门无扰交接。 */
    if (update_altitude_mode_handover(now_ms) != FC_STATUS_OK)
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
    update_takeoff_phase(now_ms);
    publish_debug_snapshot();
}

void App_FlightTask50Hz(void)
{
    FcStatus_t barometer_status;
    FcStatus_t estimator_status;

    if (!s_initialized)
    {
        return;
    }
    ++s_task_stats.task_50hz_count;

    {
        uint32_t now_ms = App_SchedulerGetTickMs();
        float barometer_noise_scale = current_barometer_noise_scale(now_ms);

        barometer_status = Drv_Bmp388_Read(&s_barometer, now_ms);
        estimator_status = Est_AltitudeUpdate(&s_barometer,
                                              &s_imu,
                                              &s_attitude,
                                              FC_ALTITUDE_DT_S,
                                              altitude_ground_constraint_active(),
                                              barometer_noise_scale,
                                              &s_altitude);
    }
    (void)barometer_status;
    if ((estimator_status != FC_STATUS_OK) || !s_altitude.valid)
    {
        s_altitude.valid = false;
        s_altitude_correction_us = 0.0f;
        if (s_mode == FC_MODE_ALT_HOLD)
        {
            /* Preserve manual stabilized flight; pilot must toggle mode to retry. */
            s_altitude_hold_fault_latched = true;
            (void)change_mode(FC_MODE_STABILIZE);
        }
        return;
    }

    if ((s_mode == FC_MODE_ALT_HOLD) && (s_state == FC_STATE_RUNNING))
    {
        uint16_t manual_us = build_manual_throttle_command_us();

        /*
         * 刚进入定高时先等待油门进入捕获区，避免原来的高/低油门位置
         * 立即被解释为升降高度指令。
         */
        if ((s_altitude_hold_phase == APP_ALT_HOLD_PHASE_WAIT_CAPTURE) &&
            s_rc.mode_switch &&
            throttle_is_in_altitude_capture_window(manual_us))
        {
            s_altitude_throttle_captured = true;
            s_altitude_hold_phase = APP_ALT_HOLD_PHASE_ACTIVE;
        }

        if (s_altitude_hold_phase == APP_ALT_HOLD_PHASE_ACTIVE)
        {
            float normalized_vertical_command = 0.0f;

            if (manual_us > FC_ALT_HOLD_STICK_CAPTURE_MAX_US)
            {
                normalized_vertical_command =
                    (float)(manual_us - FC_ALT_HOLD_STICK_CAPTURE_MAX_US) /
                    (float)(FC_ESC_COMMAND_MAX_US - FC_ALT_HOLD_STICK_CAPTURE_MAX_US);
            }
            else if (manual_us < FC_ALT_HOLD_STICK_CAPTURE_MIN_US)
            {
                normalized_vertical_command =
                    -(float)(FC_ALT_HOLD_STICK_CAPTURE_MIN_US - manual_us) /
                    (float)(FC_ALT_HOLD_STICK_CAPTURE_MIN_US - FC_ESC_IDLE_US);
            }

            /* 捕获区内保持目标高度，捕获区外按比例改变目标高度。 */
            s_altitude_target_m += normalized_vertical_command *
                                   FC_ALTITUDE_MAX_VERTICAL_SPEED_MPS *
                                   FC_ALTITUDE_DT_S;
            if (s_altitude_target_m < FC_ALTITUDE_MIN_TARGET_M)
            {
                s_altitude_target_m = FC_ALTITUDE_MIN_TARGET_M;
            }
            if (s_altitude_target_m > FC_ALTITUDE_MAX_TARGET_M)
            {
                s_altitude_target_m = FC_ALTITUDE_MAX_TARGET_M;
            }
        }
        /* 退出混合阶段已经把总油门平滑移交给手动，不再更新高度 PID。 */
        if (s_altitude_hold_phase != APP_ALT_HOLD_PHASE_EXIT_BLEND)
        {
            if (Ctl_AltitudeUpdate(s_altitude_target_m,
                                   &s_altitude,
                                   FC_ALTITUDE_DT_S,
                                   &s_altitude_correction_us) != FC_STATUS_OK)
            {
                s_altitude_correction_us = 0.0f;
                s_altitude_hold_fault_latched = true;
                (void)change_mode(FC_MODE_STABILIZE);
            }
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
    debug->barometer = g_fc_flight_debug.barometer;
    debug->magnetometer = g_fc_flight_debug.magnetometer;
    debug->altitude = g_fc_flight_debug.altitude;
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
    debug->takeoff_phase = g_fc_flight_debug.takeoff_phase;
    debug->airborne = g_fc_flight_debug.airborne;
    debug->barometer_noise_scale = g_fc_flight_debug.barometer_noise_scale;
    debug->altitude_hold_phase = g_fc_flight_debug.altitude_hold_phase;
    debug->altitude_throttle_captured = g_fc_flight_debug.altitude_throttle_captured;
    debug->altitude_entry_blend_active = g_fc_flight_debug.altitude_entry_blend_active;
    debug->altitude_target_m = g_fc_flight_debug.altitude_target_m;
    debug->altitude_correction_us = g_fc_flight_debug.altitude_correction_us;
    debug->manual_throttle_command_us = g_fc_flight_debug.manual_throttle_command_us;
    debug->automatic_throttle_command_us = g_fc_flight_debug.automatic_throttle_command_us;
    debug->throttle_command_us = g_fc_flight_debug.throttle_command_us;
    debug->motor_safe = g_fc_flight_debug.motor_safe;
    debug->publish_count = g_fc_flight_debug.publish_count;
    return s_initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}
