/* Host test for STOP/READY/RUNNING, PID resets, modes, and safety failures. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "app_flight.h"
#include "app_safety.h"
#include "app_scheduler.h"
#include "bsp_battery_adc.h"
#include "bsp_esc_pwm.h"
#include "ctl_altitude.h"
#include "ctl_attitude.h"
#include "ctl_mixer.h"
#include "ctl_rate.h"
#include "drv_bmp390.h"
#include "drv_ibus.h"
#include "drv_bmi088.h"
#include "est_altitude.h"
#include "est_attitude.h"
#include "fc_config.h"

static uint32_t s_tick_ms;
static FcRcInput_t s_fake_rc;
static FcImuData_t s_fake_imu;
static FcAttitude_t s_fake_attitude;
static FcBatteryStatus_t s_fake_battery;
static FcAltitude_t s_fake_altitude;
static FcStatus_t s_altitude_status;
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

FcStatus_t Drv_Bmp390_Read(FcBarometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *data = (FcBarometerData_t){0};
    data->pressure_pa = 101325.0f;
    data->timestamp_ms = timestamp_ms;
    data->valid = s_altitude_status == FC_STATUS_OK;
    return s_altitude_status;
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

FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcRangeData_t *range,
                              float dt_s,
                              FcAltitude_t *altitude)
{
    (void)range;
    (void)dt_s;
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
    (void)target_altitude_m;
    (void)dt_s;
    if ((altitude == NULL) || (throttle_correction_us == NULL) || !altitude->valid)
    {
        return FC_STATUS_INVALID_DATA;
    }
    *throttle_correction_us = 0.0f;
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

    prepare_safe_inputs();
    if (App_SafetyInit() != FC_STATUS_OK) { return 1; }
    App_SafetySetInitializationResult(true);
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

    s_fake_rc.throttle = 150U;
    s_fake_rc.throttle_low = false;
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

    s_fake_rc.mode_switch = true;
    s_tick_ms = 30U;
    App_FlightTask100Hz();
    if ((App_FlightGetMode() != FC_MODE_ALT_HOLD) ||
        (App_FlightGetState() != FC_STATE_RUNNING)) { return 10; }
    if ((s_rate_reset_count != 3U) || (s_attitude_reset_count != 3U) ||
        (s_altitude_reset_count != 3U)) { return 11; }

    s_fake_rc.failsafe = true;
    s_fake_rc.link_valid = false;
    s_tick_ms = 40U;
    App_FlightTask100Hz();
    if ((App_FlightGetState() != FC_STATE_STOP) || s_esc_enabled ||
        !motors_are_stopped(&s_esc_output)) { return 12; }
    if ((s_rate_reset_count != 4U) || (s_attitude_reset_count != 4U) ||
        (s_altitude_reset_count != 4U)) { return 13; }

    s_fake_rc.failsafe = false;
    s_fake_rc.link_valid = true;
    s_fake_rc.throttle = 0U;
    s_fake_rc.throttle_low = true;
    s_tick_ms = 50U;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_READY) { return 14; }

    s_fake_rc.throttle = 150U;
    s_fake_rc.throttle_low = false;
    s_tick_ms = 60U;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_RUNNING) { return 15; }

    s_altitude_status = FC_STATUS_ERROR;
    s_tick_ms = 80U;
    App_FlightTask50Hz();
    if ((App_FlightGetState() != FC_STATE_STOP) || s_esc_enabled ||
        !motors_are_stopped(&s_esc_output)) { return 16; }
    if ((s_rate_reset_count != 7U) || (s_attitude_reset_count != 7U) ||
        (s_altitude_reset_count != 7U)) { return 17; }

    /* ALT_HOLD remains selected: invalid altitude must prevent re-entry to READY. */
    s_fake_rc.throttle = 0U;
    s_fake_rc.throttle_low = true;
    s_tick_ms = 90U;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_STOP) { return 18; }
    if ((s_rate_reset_count != 7U) || (s_attitude_reset_count != 7U) ||
        (s_altitude_reset_count != 7U)) { return 19; }

    s_fake_battery.critical = true;
    s_fake_rc.mode_switch = false;
    s_tick_ms = 100U;
    App_FlightTask100Hz();
    if (App_SafetyGetStatus(&safety_status) != FC_STATUS_OK) { return 20; }
    if ((safety_status.active_faults & FC_SAFETY_FAULT_BATTERY_CRITICAL) == 0U) { return 21; }
    if (App_FlightGetState() != FC_STATE_STOP) { return 22; }

    App_FlightTask10Hz();
    return 0;
}
