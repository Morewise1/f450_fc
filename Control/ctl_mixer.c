/* Quad-X mixer that preserves collective throttle and prioritizes roll/pitch. */

#include <stddef.h>
#include "ctl_mixer.h"
#include "fc_board.h"
#include "fc_config.h"

volatile CtlMixerDebug_t g_ctl_mixer_debug;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static float calculate_available_scale(const float base[FC_MOTOR_COUNT],
                                       const float contribution[FC_MOTOR_COUNT])
{
    float scale = 1.0f;
    uint32_t index;

    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        float candidate;

        if (contribution[index] > 0.0f)
        {
            candidate = ((float)FC_ESC_COMMAND_MAX_US - base[index]) /
                        contribution[index];
        }
        else if (contribution[index] < 0.0f)
        {
            candidate = (base[index] - (float)FC_ESC_IDLE_US) /
                        (-contribution[index]);
        }
        else
        {
            continue;
        }

        if (candidate < scale) { scale = candidate; }
    }

    return clamp_float(scale, 0.0f, 1.0f);
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
    g_ctl_mixer_debug = (CtlMixerDebug_t){0};
}

FcStatus_t Ctl_MixerQuadX(uint16_t throttle_us,
                         float roll_cmd_us,
                         float pitch_cmd_us,
                         float yaw_cmd_us,
                         FcMotorOutput_t *output)
{
    float base[FC_MOTOR_COUNT];
    float roll_pitch[FC_MOTOR_COUNT];
    float yaw[FC_MOTOR_COUNT];
    float motor[FC_MOTOR_COUNT];
    float roll_pitch_scale;
    float yaw_scale;
    float requested_yaw_us;
    uint32_t index;

    if (output == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    Ctl_MixerSetStop(output);

    if ((throttle_us < FC_ESC_IDLE_US) || (throttle_us > FC_ESC_COMMAND_MAX_US))
    {
        return FC_STATUS_INVALID_DATA;
    }

    roll_cmd_us = clamp_float(roll_cmd_us, -FC_MIXER_ROLL_LIMIT_US, FC_MIXER_ROLL_LIMIT_US);
    pitch_cmd_us = clamp_float(pitch_cmd_us, -FC_MIXER_PITCH_LIMIT_US, FC_MIXER_PITCH_LIMIT_US);
    requested_yaw_us = yaw_cmd_us;
    yaw_cmd_us = clamp_float(yaw_cmd_us, -FC_MIXER_YAW_LIMIT_US, FC_MIXER_YAW_LIMIT_US);

    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        base[index] = (float)throttle_us;
    }

    roll_pitch[FC_MOTOR_INDEX_M1] = (FC_MIX_M1_ROLL * roll_cmd_us) +
                                    (FC_MIX_M1_PITCH * pitch_cmd_us);
    roll_pitch[FC_MOTOR_INDEX_M2] = (FC_MIX_M2_ROLL * roll_cmd_us) +
                                    (FC_MIX_M2_PITCH * pitch_cmd_us);
    roll_pitch[FC_MOTOR_INDEX_M3] = (FC_MIX_M3_ROLL * roll_cmd_us) +
                                    (FC_MIX_M3_PITCH * pitch_cmd_us);
    roll_pitch[FC_MOTOR_INDEX_M4] = (FC_MIX_M4_ROLL * roll_cmd_us) +
                                    (FC_MIX_M4_PITCH * pitch_cmd_us);
    roll_pitch_scale = calculate_available_scale(base, roll_pitch);

    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        roll_pitch[index] *= roll_pitch_scale;
        base[index] += roll_pitch[index];
    }

    yaw[FC_MOTOR_INDEX_M1] = FC_MIX_M1_YAW * yaw_cmd_us;
    yaw[FC_MOTOR_INDEX_M2] = FC_MIX_M2_YAW * yaw_cmd_us;
    yaw[FC_MOTOR_INDEX_M3] = FC_MIX_M3_YAW * yaw_cmd_us;
    yaw[FC_MOTOR_INDEX_M4] = FC_MIX_M4_YAW * yaw_cmd_us;
    yaw_scale = calculate_available_scale(base, yaw);

    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        motor[index] = base[index] + (yaw[index] * yaw_scale);
        motor[index] = clamp_float(motor[index],
                                   (float)FC_ESC_IDLE_US,
                                   (float)FC_ESC_COMMAND_MAX_US);
        output->motor_us[index] = (uint16_t)(motor[index] + 0.5f);
    }
    g_ctl_mixer_debug.roll_pitch_scale = roll_pitch_scale;
    g_ctl_mixer_debug.yaw_scale = yaw_scale;
    g_ctl_mixer_debug.requested_yaw_us = requested_yaw_us;
    g_ctl_mixer_debug.applied_yaw_us = yaw_cmd_us * yaw_scale;
    g_ctl_mixer_debug.saturated = (roll_pitch_scale < 0.9999f) ||
                                  (yaw_scale < 0.9999f) ||
                                  (requested_yaw_us != yaw_cmd_us);
    output->valid = true;
    return FC_STATUS_OK;
}
