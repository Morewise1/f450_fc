/* Runs bounded PID controllers only when IMU data is valid. */

#include <stddef.h>
#include "ctl_rate.h"
#include "ctl_pid.h"
#include "fc_config.h"
#include "fc_params.h"

static CtlPid_t s_roll_pid;
static CtlPid_t s_pitch_pid;
static CtlPid_t s_yaw_pid;
static bool s_initialized;

static CtlPidConfig_t make_config(float kp, float ki, float kd, float limit)
{
    CtlPidConfig_t config;
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
    config.integrator_min = -limit;
    config.integrator_max = limit;
    config.output_min = -limit;
    config.output_max = limit;
    return config;
}

FcStatus_t Ctl_RateInit(void)
{
    CtlPidConfig_t roll = make_config(FC_RATE_ROLL_KP, FC_RATE_ROLL_KI, FC_RATE_ROLL_KD, FC_MIXER_ROLL_LIMIT_US);
    CtlPidConfig_t pitch = make_config(FC_RATE_PITCH_KP, FC_RATE_PITCH_KI, FC_RATE_PITCH_KD, FC_MIXER_PITCH_LIMIT_US);
    CtlPidConfig_t yaw = make_config(FC_RATE_YAW_KP, FC_RATE_YAW_KI, FC_RATE_YAW_KD, FC_MIXER_YAW_LIMIT_US);

    if ((Ctl_PidInit(&s_roll_pid, &roll) != FC_STATUS_OK) ||
        (Ctl_PidInit(&s_pitch_pid, &pitch) != FC_STATUS_OK) ||
        (Ctl_PidInit(&s_yaw_pid, &yaw) != FC_STATUS_OK))
    {
        s_initialized = false;
        return FC_STATUS_ERROR;
    }
    s_initialized = true;
    return FC_STATUS_OK;
}

FcStatus_t Ctl_RateUpdate(const FcVector3f_t *target_rate_dps,
                          const FcImuData_t *imu,
                          float dt_s,
                          FcControlOutput_t *output)
{
    if ((target_rate_dps == NULL) || (imu == NULL) || (output == NULL) || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *output = (FcControlOutput_t){0};
    if (!s_initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (!imu->valid) { return FC_STATUS_INVALID_DATA; }

    output->roll_cmd_us = Ctl_PidUpdate(&s_roll_pid, target_rate_dps->x, imu->gyro_dps.x, dt_s);
    output->pitch_cmd_us = Ctl_PidUpdate(&s_pitch_pid, target_rate_dps->y, imu->gyro_dps.y, dt_s);
    output->yaw_cmd_us = Ctl_PidUpdate(&s_yaw_pid, target_rate_dps->z, imu->gyro_dps.z, dt_s);
    output->valid = true;
    return FC_STATUS_OK;
}

void Ctl_RateReset(void)
{
    Ctl_PidReset(&s_roll_pid);
    Ctl_PidReset(&s_pitch_pid);
    Ctl_PidReset(&s_yaw_pid);
}

