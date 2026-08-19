/* Host tests for startup leveling, relative yaw integration, and angle deadband. */

#include <math.h>
#include "ctl_attitude.h"
#include "est_attitude.h"
#include "fc_config.h"
#include "fc_params.h"

#define DEG_TO_RAD_TEST (0.01745329251994329577f)

static int nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    FcImuData_t imu = {0};
    FcAttitude_t attitude = {0};
    FcControlTarget_t target = {0};
    FcVector3f_t target_rate_dps = {0};
    uint32_t sample;

    imu.valid = true;
    imu.calibrated = true;
    imu.accel_g.z = -1.0f;

    if (Est_AttitudeUpdate(&imu, 0.004f, &attitude) != FC_STATUS_NOT_INITIALIZED) { return 1; }
    if (Est_AttitudeInit() != FC_STATUS_OK) { return 2; }

    for (sample = 0U; sample < FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT; ++sample)
    {
        imu.timestamp_ms = sample * 4U;
        FcStatus_t status = Est_AttitudeUpdate(&imu, 0.004f, &attitude);
        if ((sample + 1U) < FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT)
        {
            if ((status != FC_STATUS_BUSY) || attitude.valid) { return 3; }
        }
        else if ((status != FC_STATUS_OK) || !attitude.valid) { return 4; }
    }
    if (!attitude.valid || !nearly_equal(attitude.roll_deg, 0.0f, 0.05f) ||
        !nearly_equal(attitude.pitch_deg, 0.0f, 0.05f) ||
        !nearly_equal(attitude.yaw_deg, 0.0f, 0.05f) ||
        !g_est_attitude_debug.level_calibrated) { return 5; }

    imu.gyro_dps.z = 90.0f;
    for (sample = 0U; sample < 250U; ++sample)
    {
        if (Est_AttitudeUpdate(&imu, 0.004f, &attitude) != FC_STATUS_OK) { return 6; }
    }
    if (!nearly_equal(attitude.yaw_deg, 90.0f, 0.2f)) { return 7; }
    if (!nearly_equal(attitude.roll_deg, 0.0f, 0.1f) ||
        !nearly_equal(attitude.pitch_deg, 0.0f, 0.1f)) { return 8; }

    Est_AttitudeReset();
    imu.gyro_dps = (FcVector3f_t){0};
    for (sample = 0U; sample < FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT; ++sample)
    {
        float noise_g = ((sample & 1U) == 0U) ? 0.015f : -0.015f;
        FcStatus_t status;

        imu.accel_g.x = sinf(-2.0f * DEG_TO_RAD_TEST) + noise_g;
        imu.accel_g.y = (-sinf(3.0f * DEG_TO_RAD_TEST) * cosf(-2.0f * DEG_TO_RAD_TEST)) - noise_g;
        imu.accel_g.z = -cosf(3.0f * DEG_TO_RAD_TEST) * cosf(-2.0f * DEG_TO_RAD_TEST);
        status = Est_AttitudeUpdate(&imu, 0.004f, &attitude);
        if (((sample + 1U) < FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT) &&
            (status != FC_STATUS_BUSY)) { return 9; }
        if (((sample + 1U) == FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT) &&
            (status != FC_STATUS_OK)) { return 10; }
    }
    if (!nearly_equal(attitude.roll_deg, 0.0f, 0.05f) ||
        !nearly_equal(attitude.pitch_deg, 0.0f, 0.05f) ||
        !nearly_equal(g_est_attitude_debug.level_roll_trim_deg, 3.0f, 0.1f) ||
        !nearly_equal(g_est_attitude_debug.level_pitch_trim_deg, -2.0f, 0.1f)) { return 11; }

    if (Ctl_AttitudeInit() != FC_STATUS_OK) { return 12; }
    attitude.valid = true;
    attitude.roll_deg = 0.4f;
    attitude.pitch_deg = -0.4f;
    if (Ctl_AttitudeUpdate(&target, &attitude, 0.004f, &target_rate_dps) != FC_STATUS_OK ||
        !nearly_equal(target_rate_dps.x, 0.0f, 0.001f) ||
        !nearly_equal(target_rate_dps.y, 0.0f, 0.001f)) { return 13; }

    attitude.roll_deg = FC_ATTITUDE_ANGLE_DEADBAND_DEG + 0.25f;
    attitude.pitch_deg = -(FC_ATTITUDE_ANGLE_DEADBAND_DEG + 0.25f);
    if (Ctl_AttitudeUpdate(&target, &attitude, 0.004f, &target_rate_dps) != FC_STATUS_OK ||
        !nearly_equal(target_rate_dps.x, -0.25f * FC_ATTITUDE_ROLL_KP, 0.001f) ||
        !nearly_equal(target_rate_dps.y, 0.25f * FC_ATTITUDE_PITCH_KP, 0.001f)) { return 14; }

    Ctl_AttitudeReset();
    attitude.yaw_deg = 0.0f;
    target.yaw_rate_dps = FC_MAX_TARGET_YAW_RATE_DPS;
    for (sample = 0U; sample < 100U; ++sample)
    {
        if (Ctl_AttitudeUpdate(&target, &attitude, 0.004f, &target_rate_dps) != FC_STATUS_OK)
        {
            return 15;
        }
    }
#if FC_YAW_CONTROL_MODE == FC_YAW_CONTROL_MODE_HEADING_HOLD
    if (g_ctl_attitude_debug.heading_hold_enabled ||
        !g_ctl_attitude_debug.manual_rate_active ||
        !g_ctl_attitude_debug.yaw_target_initialized ||
        !nearly_equal(g_ctl_attitude_debug.yaw_target_deg,
                      attitude.yaw_deg,
                      0.01f) ||
        !nearly_equal(target_rate_dps.z, FC_MAX_TARGET_YAW_RATE_DPS, 0.001f)) { return 16; }

    /* 松杆不再追赶累计航向；先以0角速度制动，确认回中后锁住当前Yaw。 */
    target.yaw_rate_dps = 0.0f;
    for (sample = 0U;
         sample < ((FC_YAW_HOLD_CENTER_CONFIRM_MS + 3U) / 4U);
         ++sample)
    {
        if (Ctl_AttitudeUpdate(&target, &attitude, 0.004f,
                               &target_rate_dps) != FC_STATUS_OK ||
            !nearly_equal(target_rate_dps.z, 0.0f, 0.001f)) { return 17; }
    }
    if (!g_ctl_attitude_debug.heading_hold_enabled ||
        g_ctl_attitude_debug.manual_rate_active ||
        g_ctl_attitude_debug.center_wait_active) { return 18; }
    attitude.yaw_deg = 10.0f;
    if (Ctl_AttitudeUpdate(&target, &attitude, 0.004f,
                           &target_rate_dps) != FC_STATUS_OK ||
        !nearly_equal(target_rate_dps.z,
                      -(10.0f - FC_ATTITUDE_YAW_DEADBAND_DEG) *
                      FC_ATTITUDE_YAW_KP,
                      0.001f)) { return 19; }

    Ctl_AttitudeReset();
    attitude.yaw_deg = 179.0f;
    for (sample = 0U;
         sample < ((FC_YAW_HOLD_CENTER_CONFIRM_MS + 3U) / 4U);
         ++sample)
    {
        if (Ctl_AttitudeUpdate(&target, &attitude, 0.004f,
                               &target_rate_dps) != FC_STATUS_OK)
        {
            return 20;
        }
    }
    attitude.yaw_deg = -179.0f;
    if (Ctl_AttitudeUpdate(&target, &attitude, 0.004f, &target_rate_dps) != FC_STATUS_OK ||
        !nearly_equal(g_ctl_attitude_debug.yaw_error_deg, -2.0f, 0.001f) ||
        !nearly_equal(target_rate_dps.z, -FC_ATTITUDE_YAW_KP, 0.001f)) { return 21; }
#else
    if (g_ctl_attitude_debug.heading_hold_enabled ||
        !nearly_equal(target_rate_dps.z, FC_MAX_TARGET_YAW_RATE_DPS, 0.001f)) { return 16; }
#endif

    imu.valid = false;
    if (Est_AttitudeUpdate(&imu, 0.004f, &attitude) != FC_STATUS_INVALID_DATA ||
        attitude.valid) { return 22; }
    return 0;
}
