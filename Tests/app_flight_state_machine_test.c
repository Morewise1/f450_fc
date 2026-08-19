/* Host test for STOP/READY/RUNNING, PID resets, modes, and safety failures. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "app_command_mux.h"
#include "app_flight.h"
#include "app_safety.h"
#include "app_scheduler.h"
#include "bsp_battery_adc.h"
#include "bsp_esc_pwm.h"
#include "ctl_altitude.h"
#include "ctl_attitude.h"
#include "ctl_mixer.h"
#include "ctl_rate.h"
#include "drv_bmp388.h"
#include "drv_ibus.h"
#include "drv_bmi088.h"
#include "drv_mmc5983ma.h"
#include "est_altitude.h"
#include "est_attitude.h"
#include "fc_config.h"
#include "fc_params.h"

static uint32_t s_tick_ms;
static FcRcInput_t s_fake_rc;
static FcImuData_t s_fake_imu;
static FcAttitude_t s_fake_attitude;
static FcBatteryStatus_t s_fake_battery;
static FcAltitude_t s_fake_altitude;
static FcStatus_t s_altitude_status;
static float s_fake_altitude_correction_us;
static float s_last_altitude_target_m;
static bool s_esc_enabled;
static FcMotorOutput_t s_esc_output;
static uint32_t s_rate_reset_count;
static uint32_t s_attitude_reset_count;
static uint32_t s_altitude_reset_count;
static bool s_bias_tracking_enabled;

static void set_motor_stop(FcMotorOutput_t *output)
{
    uint32_t motor;
    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        output->motor_us[motor] = FC_ESC_STOP_US;
    }
    output->valid = true;
}

static void set_pilot_throttle_command_us(uint16_t command_us)
{
    float normalized;
    float input;

    if (command_us <= FC_ESC_MIN_US)
    {
        s_fake_rc.throttle = 0U;
    }
    else if (command_us >= FC_ESC_COMMAND_MAX_US)
    {
        s_fake_rc.throttle = FC_RC_THROTTLE_MAX;
    }
    else
    {
        normalized = (float)(command_us - FC_ESC_MIN_US) /
                     (float)(FC_ESC_COMMAND_MAX_US - FC_ESC_MIN_US);
        input = (float)FC_RC_THROTTLE_DEADBAND +
                normalized * (float)(FC_RC_THROTTLE_MAX - FC_RC_THROTTLE_DEADBAND);
        s_fake_rc.throttle = (uint16_t)(input + 0.5f);
    }
    s_fake_rc.throttle_low = s_fake_rc.throttle <= FC_RC_THROTTLE_ARM_MAX;
}

static bool motors_are_stopped(const FcMotorOutput_t *output)
{
    uint32_t motor;
    if ((output == NULL) || !output->valid) { return false; }
    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        if (output->motor_us[motor] != FC_ESC_STOP_US) { return false; }
    }
    return true;
}

static void prepare_safe_inputs(void)
{
    s_fake_rc = (FcRcInput_t){0};
    s_fake_rc.throttle_low = true;
    s_fake_rc.arm_switch = true;
    s_fake_rc.safety_switch = true;
    s_fake_rc.link_valid = true;

    s_fake_imu = (FcImuData_t){0};
    s_fake_imu.valid = true;
    s_fake_imu.calibrated = true;

    s_fake_attitude = (FcAttitude_t){0};
    s_fake_attitude.valid = true;

    s_fake_battery = (FcBatteryStatus_t){0};
    s_fake_battery.valid = true;
    s_fake_battery.voltage_v = 12.0f;

    s_fake_altitude = (FcAltitude_t){0};
    s_fake_altitude.valid = true;
    s_fake_altitude.altitude_m = 2.0f;
    s_altitude_status = FC_STATUS_OK;
    s_fake_altitude_correction_us = 0.0f;
    s_last_altitude_target_m = 0.0f;
}

uint32_t App_SchedulerGetTickMs(void)
{
    return s_tick_ms;
}

FcStatus_t App_SchedulerGetStats(AppSchedulerStats_t *stats)
{
    if (stats == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *stats = (AppSchedulerStats_t){0};
    stats->tick_ms = s_tick_ms;
    stats->healthy = true;
    return FC_STATUS_OK;
}

void Drv_Ibus_UpdateTimeout(uint32_t now_ms)
{
    (void)now_ms;
}

FcStatus_t Drv_Ibus_GetInput(FcRcInput_t *input)
{
    if (input == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *input = s_fake_rc;
    return FC_STATUS_OK;
}

FcStatus_t Drv_Ibus_GetRawChannels(uint16_t channels[FC_IBUS_CHANNEL_COUNT])
{
    uint32_t channel;
    if (channels == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    for (channel = 0U; channel < FC_IBUS_CHANNEL_COUNT; ++channel)
    {
        channels[channel] = 1500U;
    }
    return FC_STATUS_OK;
}

FcStatus_t Drv_Bmi088_Read(FcImuData_t *imu)
{
    if (imu == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *imu = s_fake_imu;
    imu->timestamp_ms = s_tick_ms;
    return imu->valid ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

FcStatus_t Drv_Bmi088_SetBiasTrackingEnabled(bool enabled)
{
    s_bias_tracking_enabled = enabled;
    return FC_STATUS_OK;
}

FcStatus_t BSP_BatteryAdc_Read(FcBatteryStatus_t *status, uint32_t timestamp_ms)
{
    if (status == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *status = s_fake_battery;
    status->timestamp_ms = timestamp_ms;
    return status->valid ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

FcStatus_t Drv_Bmp388_Read(FcBarometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *data = (FcBarometerData_t){0};
    data->pressure_pa = 101325.0f;
    data->timestamp_ms = timestamp_ms;
    data->valid = s_altitude_status == FC_STATUS_OK;
    return s_altitude_status;
}

FcStatus_t Drv_Mmc5983ma_Read(FcMagnetometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *data = (FcMagnetometerData_t){0};
    data->timestamp_ms = timestamp_ms;
    return FC_STATUS_NOT_READY;
}

FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu, float dt_s, FcAttitude_t *attitude)
{
    (void)dt_s;
    if ((imu == NULL) || (attitude == NULL) || !imu->valid)
    {
        if (attitude != NULL) { *attitude = (FcAttitude_t){0}; }
        return FC_STATUS_INVALID_DATA;
    }
    *attitude = s_fake_attitude;
    attitude->timestamp_ms = s_tick_ms;
    return attitude->valid ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

FcStatus_t Est_AttitudeSetMagnetometer(const FcMagnetometerData_t *magnetometer)
{
    return ((magnetometer != NULL) && magnetometer->valid) ?
           FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcImuData_t *imu,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              bool aircraft_grounded,
                              float barometer_noise_scale,
                              FcAltitude_t *altitude)
{
    (void)imu;
    (void)attitude;
    (void)dt_s;
    (void)aircraft_grounded;
    (void)barometer_noise_scale;
    if ((barometer == NULL) || (altitude == NULL) || !barometer->valid ||
        (s_altitude_status != FC_STATUS_OK))
    {
        if (altitude != NULL) { *altitude = (FcAltitude_t){0}; }
        return FC_STATUS_INVALID_DATA;
    }
    *altitude = s_fake_altitude;
    altitude->timestamp_ms = s_tick_ms;
    return altitude->valid ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

FcStatus_t Est_AltitudePredict(const FcImuData_t *imu,
                               const FcAttitude_t *attitude,
                               float dt_s,
                               uint32_t timestamp_ms,
                               bool aircraft_grounded,
                               FcAltitude_t *altitude)
{
    (void)imu;
    (void)attitude;
    (void)dt_s;
    (void)aircraft_grounded;
    if (altitude == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *altitude = s_fake_altitude;
    altitude->timestamp_ms = timestamp_ms;
    return altitude->valid ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

void Ctl_RateReset(void)
{
    ++s_rate_reset_count;
}

void Ctl_AttitudeReset(void)
{
    ++s_attitude_reset_count;
}

void Ctl_AltitudeReset(void)
{
    ++s_altitude_reset_count;
}

FcStatus_t Ctl_RateUpdate(const FcVector3f_t *target_rate_dps,
                          const FcImuData_t *imu,
                          float dt_s,
                          FcControlOutput_t *output)
{
    (void)target_rate_dps;
    (void)dt_s;
    if ((imu == NULL) || (output == NULL) || !imu->valid) { return FC_STATUS_INVALID_DATA; }
    *output = (FcControlOutput_t){0};
    output->valid = true;
    return FC_STATUS_OK;
}

FcStatus_t Ctl_AttitudeUpdate(const FcControlTarget_t *target,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              FcVector3f_t *target_rate_dps)
{
    (void)target;
    (void)dt_s;
    if ((attitude == NULL) || (target_rate_dps == NULL) || !attitude->valid)
    {
        return FC_STATUS_INVALID_DATA;
    }
    *target_rate_dps = (FcVector3f_t){0};
    return FC_STATUS_OK;
}

FcStatus_t Ctl_AltitudeUpdate(float target_altitude_m,
                              const FcAltitude_t *altitude,
                              float dt_s,
                              float *throttle_correction_us)
{
    (void)dt_s;
    if ((altitude == NULL) || (throttle_correction_us == NULL) || !altitude->valid)
    {
        return FC_STATUS_INVALID_DATA;
    }
    s_last_altitude_target_m = target_altitude_m;
    *throttle_correction_us = s_fake_altitude_correction_us;
    return FC_STATUS_OK;
}

void Ctl_MixerSetStop(FcMotorOutput_t *output)
{
    if (output != NULL) { set_motor_stop(output); }
}

FcStatus_t Ctl_MixerQuadX(uint16_t throttle_us,
                         float roll_cmd_us,
                         float pitch_cmd_us,
                         float yaw_cmd_us,
                         FcMotorOutput_t *output)
{
    uint32_t motor;
    (void)roll_cmd_us;
    (void)pitch_cmd_us;
    (void)yaw_cmd_us;
    if (output == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        output->motor_us[motor] = throttle_us;
    }
    output->valid = true;
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_StopAll(void)
{
    s_esc_enabled = false;
    set_motor_stop(&s_esc_output);
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_SetOutputEnabled(bool enabled)
{
    s_esc_enabled = enabled;
    if (!enabled) { set_motor_stop(&s_esc_output); }
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_WriteAll(const FcMotorOutput_t *out)
{
    if ((out == NULL) || !out->valid) { return FC_STATUS_INVALID_ARGUMENT; }
    if (!s_esc_enabled && !motors_are_stopped(out)) { return FC_STATUS_NOT_READY; }
    s_esc_output = *out;
    return FC_STATUS_OK;
}

int main(void)
{
    FcMotorOutput_t flight_output;
    FcSafetyStatus_t safety_status;
    AppFlightDebug_t debug;
    float hold_target_m;
    float climbed_target_m;

    prepare_safe_inputs();
    if (App_SafetyInit() != FC_STATUS_OK) { return 1; }
    App_SafetySetInitializationResult(true);
    if (App_CommandMuxInit() != FC_STATUS_OK) { return 26; }
    if (App_FlightInit() != FC_STATUS_OK) { return 2; }
    if ((App_FlightGetState() != FC_STATE_STOP) ||
        (App_FlightGetMode() != FC_MODE_STABILIZE) || !motors_are_stopped(&s_esc_output)) { return 3; }

    s_tick_ms = 2U;
    App_FlightTask500Hz();
    if (!s_bias_tracking_enabled) { return 23; }
    s_tick_ms = 4U;
    App_FlightTask250Hz();
    s_tick_ms = 20U;
    App_FlightTask50Hz();
    s_tick_ms = 10U;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_READY) { return 4; }
    if ((s_rate_reset_count != 1U) || (s_attitude_reset_count != 1U) ||
        (s_altitude_reset_count != 1U) || !motors_are_stopped(&s_esc_output)) { return 5; }

    /* 使用较高的手动油门，使进入定高时的混合过程可观测。 */
    set_pilot_throttle_command_us(1800U);
    s_tick_ms = 20U;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_RUNNING) { return 6; }
    if ((s_rate_reset_count != 2U) || (s_attitude_reset_count != 2U) ||
        (s_altitude_reset_count != 2U) || !motors_are_stopped(&s_esc_output)) { return 7; }

    s_tick_ms = 24U;
    App_FlightTask250Hz();
    s_tick_ms = 26U;
    App_FlightTask500Hz();
    if (s_bias_tracking_enabled) { return 24; }
    if (!s_esc_enabled || motors_are_stopped(&s_esc_output)) { return 8; }
    if (App_FlightGetMotorOutput(&flight_output) != FC_STATUS_OK ||
        motors_are_stopped(&flight_output)) { return 9; }
    if ((flight_output.motor_us[0] < 1795U) ||
        (flight_output.motor_us[0] > 1805U)) { return 27; }

    /* CH6高：进入定高，但未捕获前不能把原来的1800us解释为爬升。 */
    s_fake_rc.mode_switch = true;
    s_tick_ms = 30U;
    App_FlightTask100Hz();
    if ((App_FlightGetMode() != FC_MODE_ALT_HOLD) ||
        (App_FlightGetState() != FC_STATE_RUNNING)) { return 10; }
    if ((s_rate_reset_count != 2U) || (s_attitude_reset_count != 2U) ||
        (s_altitude_reset_count != 3U)) { return 11; }
    if (App_FlightGetDebug(&debug) != FC_STATUS_OK) { return 28; }
    if ((debug.altitude_hold_phase != APP_ALT_HOLD_PHASE_WAIT_CAPTURE) ||
        debug.altitude_throttle_captured || !debug.altitude_entry_blend_active ||
        (debug.altitude_target_m != s_fake_altitude.altitude_m)) { return 29; }

    /* 进入瞬间应继续输出当前手动总油门。 */
    s_tick_ms = 30U;
    App_FlightTask500Hz();
    if (App_FlightGetMotorOutput(&flight_output) != FC_STATUS_OK) { return 30; }
    if ((flight_output.motor_us[0] < 1795U) ||
        (flight_output.motor_us[0] > 1805U)) { return 31; }

    /* 400ms混合进行一半时，输出应位于1800和1490之间。 */
    s_tick_ms = 230U;
    App_FlightTask100Hz();
    App_FlightTask500Hz();
    if (App_FlightGetMotorOutput(&flight_output) != FC_STATUS_OK) { return 32; }
    if ((flight_output.motor_us[0] < 1635U) ||
        (flight_output.motor_us[0] > 1655U)) { return 33; }

    /* 混合结束后为悬停前馈1490us加高度PID修正。 */
    s_tick_ms = 430U;
    App_FlightTask100Hz();
    App_FlightTask500Hz();
    if (App_FlightGetMotorOutput(&flight_output) != FC_STATUS_OK) { return 34; }
    if (flight_output.motor_us[0] != FC_HOVER_FEEDFORWARD_US) { return 35; }
    if (App_FlightGetDebug(&debug) != FC_STATUS_OK) { return 36; }
    if (debug.altitude_throttle_captured || debug.altitude_entry_blend_active) { return 37; }

    /* 油门进入1400..1600一次后才取得高度指令权。 */
    set_pilot_throttle_command_us(1500U);
    s_tick_ms = 440U;
    App_FlightTask100Hz();
    App_FlightTask50Hz();
    s_tick_ms = 442U;
    App_FlightTask500Hz();
    if (App_FlightGetDebug(&debug) != FC_STATUS_OK) { return 38; }
    if (!debug.altitude_throttle_captured ||
        (debug.altitude_hold_phase != APP_ALT_HOLD_PHASE_ACTIVE)) { return 39; }
    hold_target_m = s_last_altitude_target_m;

    /* 捕获区内任意位置都保持目标高度。 */
    set_pilot_throttle_command_us(1550U);
    s_tick_ms = 450U;
    App_FlightTask100Hz();
    App_FlightTask50Hz();
    if (s_last_altitude_target_m != hold_target_m) { return 40; }
    set_pilot_throttle_command_us(1450U);
    s_tick_ms = 460U;
    App_FlightTask100Hz();
    App_FlightTask50Hz();
    if (s_last_altitude_target_m != hold_target_m) { return 41; }

    set_pilot_throttle_command_us(1800U);
    s_tick_ms = 470U;
    App_FlightTask100Hz();
    App_FlightTask50Hz();
    if (s_last_altitude_target_m <= hold_target_m) { return 42; }
    climbed_target_m = s_last_altitude_target_m;

    set_pilot_throttle_command_us(1200U);
    s_tick_ms = 480U;
    App_FlightTask100Hz();
    App_FlightTask50Hz();
    if (s_last_altitude_target_m >= climbed_target_m) { return 43; }

    /* CH6拨低但油门未回捕获区时继续定高，不直接交权。 */
    s_fake_rc.mode_switch = false;
    set_pilot_throttle_command_us(1800U);
    s_tick_ms = 500U;
    App_FlightTask100Hz();
    if (App_FlightGetDebug(&debug) != FC_STATUS_OK) { return 44; }
    if ((App_FlightGetMode() != FC_MODE_ALT_HOLD) ||
        (debug.altitude_hold_phase != APP_ALT_HOLD_PHASE_EXIT_WAIT)) { return 45; }

    /* 回到捕获区后，差值较大时用400ms平滑交给手动油门。 */
    s_fake_altitude_correction_us = 100.0f;
    s_tick_ms = 505U;
    App_FlightTask50Hz();
    set_pilot_throttle_command_us(1400U);
    s_tick_ms = 510U;
    App_FlightTask100Hz();
    if (App_FlightGetDebug(&debug) != FC_STATUS_OK) { return 46; }
    if ((App_FlightGetMode() != FC_MODE_ALT_HOLD) ||
        (debug.altitude_hold_phase != APP_ALT_HOLD_PHASE_EXIT_BLEND)) { return 47; }

    s_tick_ms = 710U;
    App_FlightTask100Hz();
    App_FlightTask500Hz();
    if (App_FlightGetMotorOutput(&flight_output) != FC_STATUS_OK) { return 48; }
    if ((flight_output.motor_us[0] < 1490U) ||
        (flight_output.motor_us[0] > 1500U)) { return 49; }

    s_tick_ms = 910U;
    App_FlightTask100Hz();
    if (App_FlightGetMode() != FC_MODE_STABILIZE) { return 50; }
    App_FlightTask500Hz();
    if (App_FlightGetMotorOutput(&flight_output) != FC_STATUS_OK) { return 51; }
    if ((flight_output.motor_us[0] < 1395U) ||
        (flight_output.motor_us[0] > 1405U)) { return 52; }

    /* 再次进入定高，确认原有高度传感器故障降级仍有效。 */
    s_fake_altitude_correction_us = 0.0f;
    s_fake_rc.mode_switch = true;
    s_tick_ms = 920U;
    App_FlightTask100Hz();
    if (App_FlightGetMode() != FC_MODE_ALT_HOLD) { return 25; }

    s_altitude_status = FC_STATUS_ERROR;
    s_tick_ms = 930U;
    App_FlightTask50Hz();
    if ((App_FlightGetState() != FC_STATE_RUNNING) ||
        (App_FlightGetMode() != FC_MODE_STABILIZE)) { return 16; }

    /* Holding the switch high cannot re-enter ALT_HOLD after a sensor fault. */
    s_tick_ms = 940U;
    App_FlightTask100Hz();
    if ((App_FlightGetState() != FC_STATE_RUNNING) ||
        (App_FlightGetMode() != FC_MODE_STABILIZE)) { return 18; }

    /* 高油门加持续上升证据确认离地；确认后收至1400us以下仍必须锁存AIRBORNE。 */
    s_altitude_status = FC_STATUS_OK;
    set_pilot_throttle_command_us(1800U);
    s_fake_altitude.altitude_m = 0.20f;
    s_fake_altitude.vertical_velocity_mps = 0.20f;
    s_tick_ms = 945U;
    App_FlightTask250Hz();
    s_tick_ms = 950U;
    App_FlightTask100Hz();
    s_tick_ms = 1160U;
    App_FlightTask100Hz();
    if (App_FlightGetDebug(&debug) != FC_STATUS_OK || !debug.airborne ||
        (debug.takeoff_phase != APP_TAKEOFF_PHASE_AIRBORNE)) { return 53; }
    set_pilot_throttle_command_us(1300U);
    s_tick_ms = 1170U;
    App_FlightTask100Hz();
    if (App_FlightGetDebug(&debug) != FC_STATUS_OK || !debug.airborne ||
        (debug.takeoff_phase != APP_TAKEOFF_PHASE_AIRBORNE)) { return 54; }

    s_fake_battery.critical = true;
    s_fake_rc.mode_switch = false;
    s_tick_ms = 1180U;
    App_FlightTask100Hz();
    if (App_SafetyGetStatus(&safety_status) != FC_STATUS_OK) { return 20; }
    if ((safety_status.active_faults & FC_SAFETY_FAULT_BATTERY_CRITICAL) == 0U) { return 21; }
    if (App_FlightGetState() != FC_STATE_STOP) { return 22; }

    App_FlightTask10Hz();
    return 0;
}
