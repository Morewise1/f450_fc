/* Host tests for level stability and relative yaw integration. */

#include <math.h>
#include "est_attitude.h"

static int nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    FcImuData_t imu = {0};
    FcAttitude_t attitude = {0};
    uint32_t sample;

    imu.valid = true;
    imu.calibrated = true;
    imu.accel_g.z = -1.0f;

    if (Est_AttitudeUpdate(&imu, 0.004f, &attitude) != FC_STATUS_NOT_INITIALIZED) { return 1; }
    if (Est_AttitudeInit() != FC_STATUS_OK) { return 2; }

    for (sample = 0U; sample < 500U; ++sample)
    {
        imu.timestamp_ms = sample * 4U;
        if (Est_AttitudeUpdate(&imu, 0.004f, &attitude) != FC_STATUS_OK) { return 3; }
    }
    if (!attitude.valid || !nearly_equal(attitude.roll_deg, 0.0f, 0.05f) ||
        !nearly_equal(attitude.pitch_deg, 0.0f, 0.05f) ||
        !nearly_equal(attitude.yaw_deg, 0.0f, 0.05f)) { return 4; }

    Est_AttitudeReset();
    imu.gyro_dps.z = 90.0f;
    for (sample = 0U; sample < 250U; ++sample)
    {
        if (Est_AttitudeUpdate(&imu, 0.004f, &attitude) != FC_STATUS_OK) { return 5; }
    }
    if (!nearly_equal(attitude.yaw_deg, 90.0f, 0.2f)) { return 6; }
    if (!nearly_equal(attitude.roll_deg, 0.0f, 0.1f) ||
        !nearly_equal(attitude.pitch_deg, 0.0f, 0.1f)) { return 7; }

    imu.valid = false;
    if (Est_AttitudeUpdate(&imu, 0.004f, &attitude) != FC_STATUS_INVALID_DATA ||
        attitude.valid) { return 8; }
    return 0;
}
