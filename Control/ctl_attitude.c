/* Bounded proportional attitude outer loop feeding the rate PID. */

#include <stddef.h>
#include "ctl_attitude.h"
#include "fc_config.h"
#include "fc_params.h"

static bool s_initialized;
static float s_yaw_target_deg;
static bool s_yaw_target_initialized;
volatile CtlAttitudeDebug_t g_ctl_attitude_debug;

static float clamp_float(float value, float limit)
{
    if (value < -limit) { return -limit; }
    if (value > limit) { return limit; }
    return value;
}

static float apply_continuous_deadband(float error, float deadband)
{
    if (error > deadband) { return error - deadband; }
    if (error < -deadband) { return error + deadband; }
    return 0.0f;
}

#if FC_YAW_CONTROL_MODE == FC_YAW_CONTROL_MODE_HEADING_HOLD
static float wrap_degrees(float angle_deg)
{
    if (angle_deg > 180.0f) { angle_deg -= 360.0f; }
    if (angle_deg < -180.0f) { angle_deg += 360.0f; }
    return angle_deg;
}
#endif

static float update_yaw_target_rate(const FcControlTarget_t *target,
                                    const FcAttitude_t *attitude,
                                    float dt_s)
{
#if FC_YAW_CONTROL_MODE == FC_YAW_CONTROL_MODE_HEADING_HOLD
    float yaw_error;
    float effective_error;
    float target_rate;

    if (!s_yaw_target_initialized)
    {
        s_yaw_target_deg = attitude->yaw_deg;
        s_yaw_target_initialized = true;
    }

    s_yaw_target_deg = wrap_degrees(s_yaw_target_deg +
                                    (target->yaw_rate_dps * dt_s));
    yaw_error = wrap_degrees(s_yaw_target_deg - attitude->yaw_deg);

    /* Do not allow an unavailable yaw actuator to accumulate a large heading demand. */
    if (yaw_error > FC_ATTITUDE_YAW_ERROR_LIMIT_DEG)
    {
        yaw_error = FC_ATTITUDE_YAW_ERROR_LIMIT_DEG;
        s_yaw_target_deg = wrap_degrees(attitude->yaw_deg + yaw_error);
    }
    else if (yaw_error < -FC_ATTITUDE_YAW_ERROR_LIMIT_DEG)
    {
        yaw_error = -FC_ATTITUDE_YAW_ERROR_LIMIT_DEG;
        s_yaw_target_deg = wrap_degrees(attitude->yaw_deg + yaw_error);
    }

    effective_error = apply_continuous_deadband(yaw_error,
                                                 FC_ATTITUDE_YAW_DEADBAND_DEG);
    target_rate = target->yaw_rate_dps + (effective_error * FC_ATTITUDE_YAW_KP);
    g_ctl_attitude_debug.yaw_target_deg = s_yaw_target_deg;
    g_ctl_attitude_debug.yaw_error_deg = yaw_error;
    g_ctl_attitude_debug.yaw_target_initialized = true;
    g_ctl_attitude_debug.heading_hold_enabled = true;
    return clamp_float(target_rate, FC_MAX_TARGET_YAW_RATE_DPS);
#else
    (void)attitude;
    (void)dt_s;
    g_ctl_attitude_debug.yaw_target_deg = 0.0f;
    g_ctl_attitude_debug.yaw_error_deg = 0.0f;
    g_ctl_attitude_debug.yaw_target_initialized = false;
    g_ctl_attitude_debug.heading_hold_enabled = false;
    return clamp_float(target->yaw_rate_dps, FC_MAX_TARGET_YAW_RATE_DPS);
#endif
}

FcStatus_t Ctl_AttitudeInit(void)
{
    s_yaw_target_deg = 0.0f;
    s_yaw_target_initialized = false;
    g_ctl_attitude_debug = (CtlAttitudeDebug_t){0};
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

    target_rate_dps->x = clamp_float(apply_continuous_deadband(target->roll_deg - attitude->roll_deg,
                                                               FC_ATTITUDE_ANGLE_DEADBAND_DEG) *
                                     FC_ATTITUDE_ROLL_KP,
                                     FC_MAX_TARGET_RATE_DPS);
    target_rate_dps->y = clamp_float(apply_continuous_deadband(target->pitch_deg - attitude->pitch_deg,
                                                               FC_ATTITUDE_ANGLE_DEADBAND_DEG) *
                                     FC_ATTITUDE_PITCH_KP,
                                     FC_MAX_TARGET_RATE_DPS);
    target_rate_dps->z = update_yaw_target_rate(target, attitude, dt_s);
    return FC_STATUS_OK;
}

void Ctl_AttitudeReset(void)
{
    s_yaw_target_deg = 0.0f;
    s_yaw_target_initialized = false;
    g_ctl_attitude_debug = (CtlAttitudeDebug_t){0};
}
