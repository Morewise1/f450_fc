/* Minimal bounded proportional outer loop; gains remain zero until tuning. */

#include <stddef.h>
#include "ctl_attitude.h"
#include "fc_config.h"
#include "fc_params.h"

static bool s_initialized;

static float clamp_float(float value, float limit)
{
    if (value < -limit) { return -limit; }
    if (value > limit) { return limit; }
    return value;
}

FcStatus_t Ctl_AttitudeInit(void)
{
    s_initialized = true;
    return FC_STATUS_OK;
}

FcStatus_t Ctl_AttitudeUpdate(const FcControlTarget_t *target,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              FcVector3f_t *target_rate_dps)
{
    if ((target == NULL) || (attitude == NULL) || (target_rate_dps == NULL) || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *target_rate_dps = (FcVector3f_t){0};
    if (!s_initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (!attitude->valid) { return FC_STATUS_INVALID_DATA; }

    target_rate_dps->x = clamp_float((target->roll_deg - attitude->roll_deg) * FC_ATTITUDE_ROLL_KP,
                                     FC_MAX_TARGET_RATE_DPS);
    target_rate_dps->y = clamp_float((target->pitch_deg - attitude->pitch_deg) * FC_ATTITUDE_PITCH_KP,
                                     FC_MAX_TARGET_RATE_DPS);
    target_rate_dps->z = clamp_float(target->yaw_rate_dps, FC_MAX_TARGET_YAW_RATE_DPS);
    return FC_STATUS_OK;
}

void Ctl_AttitudeReset(void)
{
}

