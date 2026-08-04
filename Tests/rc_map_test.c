/* Host tests for center deadband, expo, throttle curve, and fail-closed input. */

#include <math.h>
#include "ctl_rc_map.h"
#include "fc_config.h"

static int nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    FcRcInput_t input = {0};
    FcPilotCommand_t command = {0};
    float previous;
    float normalized_mid;
    float expected_mid;
    uint16_t throttle;

    if (Ctl_RcMapUpdate(NULL, &command) != FC_STATUS_INVALID_ARGUMENT) { return 1; }
    if (Ctl_RcMapUpdate(&input, &command) != FC_STATUS_INVALID_DATA || command.valid) { return 2; }

    input.link_valid = true;
    input.arm_switch = true;
    input.safety_switch = true;
    input.roll = FC_RC_AXIS_DEADBAND;
    input.pitch = -FC_RC_AXIS_DEADBAND;
    input.yaw = 0;
    input.throttle = FC_RC_THROTTLE_DEADBAND;
    if (Ctl_RcMapUpdate(&input, &command) != FC_STATUS_OK) { return 3; }
    if (!command.valid || !command.motor_safe) { return 4; }
    if (!nearly_equal(command.roll, 0.0f, 0.0001f) ||
        !nearly_equal(command.pitch, 0.0f, 0.0001f) ||
        !nearly_equal(command.throttle, 0.0f, 0.0001f)) { return 5; }

    input.roll = FC_RC_AXIS_MAX;
    input.pitch = FC_RC_AXIS_MIN;
    input.yaw = FC_RC_AXIS_MAX;
    input.throttle = FC_RC_THROTTLE_MAX;
    if (Ctl_RcMapUpdate(&input, &command) != FC_STATUS_OK) { return 6; }
    if (!nearly_equal(command.roll, 1.0f, 0.0001f) ||
        !nearly_equal(command.pitch, -1.0f, 0.0001f) ||
        !nearly_equal(command.yaw, 1.0f, 0.0001f) ||
        !nearly_equal(command.throttle, 1.0f, 0.0001f)) { return 7; }

    previous = -1.0f;
    for (throttle = 0U; throttle <= FC_RC_THROTTLE_MAX; throttle += 10U)
    {
        input.throttle = throttle;
        if (Ctl_RcMapUpdate(&input, &command) != FC_STATUS_OK) { return 8; }
        if ((command.throttle < previous) || (command.throttle < 0.0f) ||
            (command.throttle > 1.0f)) { return 9; }
        previous = command.throttle;
    }

    input.throttle = 500U;
    if (Ctl_RcMapUpdate(&input, &command) != FC_STATUS_OK) { return 10; }
    normalized_mid = (500.0f - (float)FC_RC_THROTTLE_DEADBAND) /
                     ((float)FC_RC_THROTTLE_MAX - (float)FC_RC_THROTTLE_DEADBAND);
#if FC_THROTTLE_CURVE_MODE == FC_THROTTLE_CURVE_MODE_LINEAR
    expected_mid = normalized_mid;
#else
    expected_mid = ((1.0f - FC_RC_THROTTLE_EXPO) * normalized_mid) +
                   (FC_RC_THROTTLE_EXPO * normalized_mid * normalized_mid);
#endif
    if (!nearly_equal(command.throttle, expected_mid, 0.0001f)) { return 11; }

    input.failsafe = true;
    if (Ctl_RcMapUpdate(&input, &command) != FC_STATUS_INVALID_DATA) { return 12; }
    if (command.valid || command.motor_safe || command.throttle != 0.0f) { return 13; }
    return 0;
}
