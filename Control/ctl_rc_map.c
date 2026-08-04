/* Center deadband and expo mapping for manual stabilize-mode commands. */

#include <stddef.h>
#include "ctl_rc_map.h"
#include "fc_config.h"

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static float map_axis(int16_t value)
{
    float magnitude = (value < 0) ? -(float)value : (float)value;
    float normalized;
    float expo;

    if (magnitude <= (float)FC_RC_AXIS_DEADBAND)
    {
        return 0.0f;
    }

    normalized = (magnitude - (float)FC_RC_AXIS_DEADBAND) /
                 ((float)FC_RC_AXIS_MAX - (float)FC_RC_AXIS_DEADBAND);
    normalized = clamp_float(normalized, 0.0f, 1.0f);
    expo = clamp_float(FC_RC_AXIS_EXPO, 0.0f, 1.0f);
    normalized = ((1.0f - expo) * normalized) +
                 (expo * normalized * normalized * normalized);
    return (value < 0) ? -normalized : normalized;
}

static float map_throttle(uint16_t value)
{
    float normalized;
#if FC_THROTTLE_CURVE_MODE == FC_THROTTLE_CURVE_MODE_EXPO
    float expo;
#endif

    if (value <= FC_RC_THROTTLE_DEADBAND)
    {
        return 0.0f;
    }

    normalized = ((float)value - (float)FC_RC_THROTTLE_DEADBAND) /
                 ((float)FC_RC_THROTTLE_MAX - (float)FC_RC_THROTTLE_DEADBAND);
    normalized = clamp_float(normalized, 0.0f, 1.0f);

#if FC_THROTTLE_CURVE_MODE == FC_THROTTLE_CURVE_MODE_LINEAR
    return normalized;
#else
    expo = clamp_float(FC_RC_THROTTLE_EXPO, 0.0f, 1.0f);

    /* Quadratic blend keeps endpoints exact and softens low-throttle response. */
    return ((1.0f - expo) * normalized) + (expo * normalized * normalized);
#endif
}

FcStatus_t Ctl_RcMapUpdate(const FcRcInput_t *input, FcPilotCommand_t *command)
{
    if ((input == NULL) || (command == NULL))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    *command = (FcPilotCommand_t){0};
    if (!input->link_valid || input->failsafe)
    {
        return FC_STATUS_INVALID_DATA;
    }

    command->roll = map_axis(input->roll);
    command->pitch = map_axis(input->pitch);
    command->yaw = map_axis(input->yaw);
    command->throttle = map_throttle(input->throttle);
    command->climb_rate = 0.0f;
    command->motor_safe = input->arm_switch && input->safety_switch;
    command->valid = true;
    return FC_STATUS_OK;
}
