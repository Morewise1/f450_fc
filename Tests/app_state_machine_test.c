/* Host test for state transitions, PID resets, modes, and fail-closed inputs. */

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
#include "drv_mmc5983ma.h"
#include "drv_ibus.h"
#include "drv_bmi088.h"
#include "est_altitude.h"
#include "est_attitude.h"
#include "fc_config.h"

static FcRcInput_t s_mock_rc;
static FcImuData_t s_mock_imu;
static FcBatteryStatus_t s_mock_battery;
static FcAttitude_t s_mock_attitude;
static FcBarometerData_t s_mock_barometer;
static FcAltitude_t s_mock_altitude;
static FcMotorOutput_t s_esc_output;
static uint32_t s_rate_reset_count;
static uint32_t s_attitude_reset_count;
static uint32_t s_altitude_reset_count;
static uint32_t s_rate_update_count;
static uint32_t s_attitude_update_count;
static uint32_t s_altitude_update_count;
static uint32_t s_esc_write_count;
static bool s_esc_enabled;

static void set_motor_stop(FcMotorOutput_t *output)
{
    uint32_t motor;
    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        output->motor_us[motor] = FC_ESC_STOP_US;
    }
    output->valid = true;
}

void Drv_Ibus_UpdateTimeout(uint32_t now_ms)
{
    (void)now_ms;
}

FcStatus_t Drv_Ibus_GetInput(FcRcInput_t *input)
{
    if (input == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *input = s_mock_rc;
    return FC_STATUS_OK;
}

FcStatus_t Drv_Bmi088_Read(FcImuData_t *imu)
{
    if (imu == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *imu = s_mock_imu;
    return s_mock_imu.valid ? FC_STATUS_OK : FC_STATUS_ERROR;
}

FcStatus_t BSP_BatteryAdc_Read(FcBatteryStatus_t *status, uint32_t timestamp_ms)
{
    if (status == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *status = s_mock_battery;
    status->timestamp_ms = timestamp_ms;
    return s_mock_battery.valid ? FC_STATUS_OK : FC_STATUS_NOT_READY;
}

FcStatus_t Drv_Bmp388_Read(FcBarometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *data = s_mock_barometer;
    data->timestamp_ms = timestamp_ms;
    return s_mock_barometer.valid ? FC_STATUS_OK : FC_STATUS_NOT_READY;
}

FcStatus_t Drv_Mmc5983ma_Read(FcMagnetometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *data = (FcMagnetometerData_t){0};
    data->timestamp_ms = timestamp_ms;
    return FC_STATUS_NOT_READY;
}

FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu,
                              float dt_s,
                              FcAttitude_t *attitude)
{
    if ((imu == NULL) || (attitude == NULL) || !imu->valid || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *attitude = s_mock_attitude;
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
                              FcAltitude_t *altitude)
{
    (void)imu;
    (void)attitude;
    if ((barometer == NULL) || (altitude == NULL) || !barometer->valid || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *altitude = s_mock_altitude;
    return altitude->valid ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

FcStatus_t Est_AltitudePredict(const FcImuData_t *imu,
                               const FcAttitude_t *attitude,
                               float dt_s,
                               uint32_t timestamp_ms,
                               FcAltitude_t *altitude)
{
    (void)imu;
    (void)attitude;
    (void)dt_s;
    if (altitude == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *altitude = s_mock_altitude;
    altitude->timestamp_ms = timestamp_ms;
    return altitude->valid ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

FcStatus_t Est_InertialNavUpdate(const FcImuData_t *imu,
                                 const FcAttitude_t *attitude,
                                 bool aircraft_stopped,
                                 float dt_s)
{
    (void)imu;
    (void)attitude;
    (void)aircraft_stopped;
    (void)dt_s;
    return FC_STATUS_OK;
}

FcStatus_t Ctl_AttitudeUpdate(const FcControlTarget_t *target,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              FcVector3f_t *target_rate_dps)
{
    if ((target == NULL) || (attitude == NULL) || (target_rate_dps == NULL) ||
        !attitude->valid || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    ++s_attitude_update_count;
    *target_rate_dps = (FcVector3f_t){0};
    return FC_STATUS_OK;
}

FcStatus_t Ctl_RateUpdate(const FcVector3f_t *target_rate_dps,
                          const FcImuData_t *imu,
                          float dt_s,
                          FcControlOutput_t *output)
{
    if ((target_rate_dps == NULL) || (imu == NULL) || (output == NULL) ||
        !imu->valid || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    ++s_rate_update_count;
    *output = (FcControlOutput_t){0};
    output->valid = true;
    return FC_STATUS_OK;
}

FcStatus_t Ctl_AltitudeUpdate(float target_altitude_m,
                              const FcAltitude_t *altitude,
                              float dt_s,
                              float *throttle_correction_us)
{
    (void)target_altitude_m;
    if ((altitude == NULL) || (throttle_correction_us == NULL) ||
        !altitude->valid || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    ++s_altitude_update_count;
    *throttle_correction_us = 25.0f;
    return FC_STATUS_OK;
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

void Ctl_MixerSetStop(FcMotorOutput_t *output)
{
    if (output != NULL) { set_motor_stop(output); }
}

void Ctl_RateReset(void) { ++s_rate_reset_count; }
void Ctl_AttitudeReset(void) { ++s_attitude_reset_count; }
void Ctl_AltitudeReset(void) { ++s_altitude_reset_count; }

FcStatus_t BSP_EscPwm_StopAll(void)
{
    s_esc_enabled = false;
    set_motor_stop(&s_esc_output);
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_SetOutputEnabled(bool enabled)
{
    s_esc_enabled = enabled;
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_WriteAll(const FcMotorOutput_t *out)
{
    if ((out == NULL) || !out->valid || !s_esc_enabled)
    {
        return FC_STATUS_NOT_READY;
    }
    s_esc_output = *out;
    ++s_esc_write_count;
    return FC_STATUS_OK;
}

static bool output_is_stopped(const FcMotorOutput_t *output)
{
    uint32_t motor;
    if ((output == NULL) || !output->valid) { return false; }
    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        if (output->motor_us[motor] != FC_ESC_STOP_US) { return false; }
    }
    return true;
}

static bool reset_counts_equal(uint32_t expected)
{
    return (s_rate_reset_count == expected) &&
           (s_attitude_reset_count == expected) &&
           (s_altitude_reset_count == expected);
}

static bool update_counts_equal(uint32_t rate,
                                uint32_t attitude,
                                uint32_t altitude)
{
    return (s_rate_update_count == rate) &&
           (s_attitude_update_count == attitude) &&
           (s_altitude_update_count == altitude);
}

static void prepare_safe_inputs(void)
{
    s_mock_rc = (FcRcInput_t){0};
    s_mock_rc.link_valid = true;
    s_mock_rc.arm_switch = true;
    s_mock_rc.safety_switch = true;
    s_mock_rc.throttle_low = true;

    s_mock_imu = (FcImuData_t){0};
    s_mock_imu.valid = true;
    s_mock_imu.calibrated = true;

    s_mock_battery = (FcBatteryStatus_t){0};
    s_mock_battery.valid = true;

    s_mock_attitude = (FcAttitude_t){0};
    s_mock_attitude.valid = true;

    s_mock_barometer = (FcBarometerData_t){0};
    s_mock_barometer.valid = true;

    s_mock_altitude = (FcAltitude_t){0};
    s_mock_altitude.altitude_m = 1.5f;
    s_mock_altitude.valid = true;
}

int main(void)
{
    AppFlightTaskStats_t task_stats;
    FcMotorOutput_t output;

    prepare_safe_inputs();
    if (App_SchedulerInit() != FC_STATUS_OK) { return 1; }
    if (App_SafetyInit() != FC_STATUS_OK) { return 2; }
    App_SafetySetInitializationResult(true);
    if (App_CommandMuxInit() != FC_STATUS_OK) { return 35; }
    if (App_FlightInit() != FC_STATUS_OK) { return 3; }
    if (App_FlightGetState() != FC_STATE_STOP) { return 4; }
    if (App_FlightGetMode() != FC_MODE_STABILIZE) { return 5; }
    if (!output_is_stopped(&s_esc_output) || !reset_counts_equal(0U)) { return 6; }

    App_FlightTask500Hz();
    App_FlightTask250Hz();
    App_FlightTask50Hz();
    if (!update_counts_equal(0U, 0U, 0U)) { return 7; }

    /* High throttle cannot skip STOP and enter RUNNING directly. */
    s_mock_rc.throttle = FC_RC_THROTTLE_TAKEOFF_MIN + 1U;
    s_mock_rc.throttle_low = false;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_STOP || !reset_counts_equal(0U)) { return 8; }

    s_mock_rc.throttle = 0U;
    s_mock_rc.throttle_low = true;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_READY || !reset_counts_equal(1U)) { return 9; }
    if (s_esc_enabled || !output_is_stopped(&s_esc_output)) { return 10; }

    App_FlightTask100Hz();
    if (!reset_counts_equal(1U)) { return 11; }

    s_mock_rc.mode_switch = true;
    App_FlightTask100Hz();
    if (App_FlightGetMode() != FC_MODE_ALT_HOLD || !reset_counts_equal(2U)) { return 12; }
    App_FlightTask50Hz();
    if (!update_counts_equal(0U, 0U, 0U)) { return 13; }

    /* A contradictory throttle_low flag cannot enter RUNNING. */
    s_mock_rc.throttle = FC_RC_THROTTLE_TAKEOFF_MIN + 1U;
    s_mock_rc.throttle_low = true;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_READY || !reset_counts_equal(2U)) { return 14; }

    s_mock_rc.throttle_low = false;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_RUNNING || !reset_counts_equal(3U)) { return 15; }

    App_FlightTask50Hz();
    App_FlightTask250Hz();
    App_FlightTask500Hz();
    if (!update_counts_equal(1U, 1U, 1U)) { return 16; }
    if (!s_esc_enabled || (s_esc_write_count != 1U)) { return 17; }
    if (s_esc_output.motor_us[0] <= FC_ESC_IDLE_US) { return 18; }

    s_mock_rc.mode_switch = false;
    App_FlightTask100Hz();
    if ((App_FlightGetMode() != FC_MODE_STABILIZE) ||
        (App_FlightGetState() != FC_STATE_RUNNING) || !reset_counts_equal(4U)) { return 19; }

    s_mock_rc.failsafe = true;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_STOP || !reset_counts_equal(5U)) { return 20; }
    if (s_esc_enabled || !output_is_stopped(&s_esc_output)) { return 21; }

    /* Re-arm, then an IMU read failure must stop at the 500 Hz task. */
    s_mock_rc.failsafe = false;
    s_mock_rc.throttle = 0U;
    s_mock_rc.throttle_low = true;
    s_mock_imu.valid = true;
    App_FlightTask500Hz();
    App_FlightTask250Hz();
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_READY || !reset_counts_equal(6U)) { return 22; }
    s_mock_rc.throttle = FC_RC_THROTTLE_TAKEOFF_MIN + 1U;
    s_mock_rc.throttle_low = false;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_RUNNING || !reset_counts_equal(7U)) { return 23; }
    s_mock_imu.valid = false;
    App_FlightTask500Hz();
    if (App_FlightGetState() != FC_STATE_STOP || !reset_counts_equal(8U)) { return 24; }
    if (s_esc_enabled || !output_is_stopped(&s_esc_output)) { return 25; }

    /* Critical battery and emergency stop are evaluated at 100 Hz. */
    s_mock_imu.valid = true;
    s_mock_rc.throttle = 0U;
    s_mock_rc.throttle_low = true;
    App_FlightTask500Hz();
    App_FlightTask250Hz();
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_READY || !reset_counts_equal(9U)) { return 26; }
    s_mock_battery.critical = true;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_STOP || !reset_counts_equal(10U)) { return 27; }

    s_mock_battery.critical = false;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_READY || !reset_counts_equal(11U)) { return 28; }
    s_mock_rc.emergency_stop = true;
    App_FlightTask100Hz();
    if (App_FlightGetState() != FC_STATE_STOP || !reset_counts_equal(12U)) { return 29; }

    /* ALT_HOLD request without altitude data must remain STOP. */
    s_mock_rc.emergency_stop = false;
    s_mock_barometer.valid = false;
    App_FlightTask50Hz();
    s_mock_rc.mode_switch = true;
    App_FlightTask100Hz();
    if ((App_FlightGetState() != FC_STATE_STOP) ||
        (App_FlightGetMode() != FC_MODE_STABILIZE) || !reset_counts_equal(12U)) { return 30; }

    App_FlightTask10Hz();
    if (App_FlightGetTaskStats(&task_stats) != FC_STATUS_OK) { return 31; }
    if (task_stats.task_10hz_count != 1U) { return 32; }
    if (App_FlightGetMotorOutput(&output) != FC_STATUS_OK || !output_is_stopped(&output)) { return 33; }
    if (!update_counts_equal(1U, 1U, 1U)) { return 34; }
    return 0;
}
