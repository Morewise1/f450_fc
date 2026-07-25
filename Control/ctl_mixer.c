/* Applies centralized Quad-X coefficients and final 1000-2000 us limiting. */

#include <stddef.h>
#include "ctl_mixer.h"
#include "fc_board.h"
#include "fc_config.h"

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

void Ctl_MixerSetStop(FcMotorOutput_t *output)
{
    uint32_t index;
    if (output == NULL)
    {
        return;
    }
    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        output->motor_us[index] = FC_ESC_STOP_US;
    }
    output->valid = true;
}

FcStatus_t Ctl_MixerQuadX(uint16_t throttle_us,
                         float roll_cmd_us,
                         float pitch_cmd_us,
                         float yaw_cmd_us,
                         FcMotorOutput_t *output)
{
    float motor[FC_MOTOR_COUNT];
    uint32_t index;

    if (output == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    Ctl_MixerSetStop(output);

    if ((throttle_us < FC_ESC_MIN_US) || (throttle_us > FC_ESC_COMMAND_MAX_US))
    {
        return FC_STATUS_INVALID_DATA;
    }

    roll_cmd_us = clamp_float(roll_cmd_us, -FC_MIXER_ROLL_LIMIT_US, FC_MIXER_ROLL_LIMIT_US);
    pitch_cmd_us = clamp_float(pitch_cmd_us, -FC_MIXER_PITCH_LIMIT_US, FC_MIXER_PITCH_LIMIT_US);
    yaw_cmd_us = clamp_float(yaw_cmd_us, -FC_MIXER_YAW_LIMIT_US, FC_MIXER_YAW_LIMIT_US);

    motor[FC_MOTOR_INDEX_M1] = (float)throttle_us +
        (FC_MIX_M1_ROLL * roll_cmd_us) + (FC_MIX_M1_PITCH * pitch_cmd_us) + (FC_MIX_M1_YAW * yaw_cmd_us);
    motor[FC_MOTOR_INDEX_M2] = (float)throttle_us +
        (FC_MIX_M2_ROLL * roll_cmd_us) + (FC_MIX_M2_PITCH * pitch_cmd_us) + (FC_MIX_M2_YAW * yaw_cmd_us);
    motor[FC_MOTOR_INDEX_M3] = (float)throttle_us +
        (FC_MIX_M3_ROLL * roll_cmd_us) + (FC_MIX_M3_PITCH * pitch_cmd_us) + (FC_MIX_M3_YAW * yaw_cmd_us);
    motor[FC_MOTOR_INDEX_M4] = (float)throttle_us +
        (FC_MIX_M4_ROLL * roll_cmd_us) + (FC_MIX_M4_PITCH * pitch_cmd_us) + (FC_MIX_M4_YAW * yaw_cmd_us);

    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        motor[index] = clamp_float(motor[index],
                                   (float)FC_ESC_MIN_US,
                                   (float)FC_ESC_COMMAND_MAX_US);
        output->motor_us[index] = (uint16_t)(motor[index] + 0.5f);
    }
    output->valid = true;
    return FC_STATUS_OK;
}
