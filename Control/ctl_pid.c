/* Implements bounded PID without dynamic allocation. */

#include <stddef.h>
#include "ctl_pid.h"

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static bool config_valid(const CtlPidConfig_t *config)
{
    return (config != NULL) &&
           (config->integrator_min <= config->integrator_max) &&
           (config->output_min <= config->output_max);
}

FcStatus_t Ctl_PidInit(CtlPid_t *pid, const CtlPidConfig_t *config)
{
    if ((pid == NULL) || !config_valid(config))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    pid->config = *config;
    pid->initialized = true;
    Ctl_PidReset(pid);
    return FC_STATUS_OK;
}

void Ctl_PidReset(CtlPid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->integrator = 0.0f;
    pid->previous_error = 0.0f;
    pid->last_output = 0.0f;
    pid->has_previous_error = false;
}

float Ctl_PidUpdate(CtlPid_t *pid, float setpoint, float measurement, float dt_s)
{
    float error;
    float derivative = 0.0f;
    float output;

    if ((pid == NULL) || !pid->initialized || (dt_s <= 0.0f) || (dt_s > 1.0f))
    {
        return 0.0f;
    }

    error = setpoint - measurement;
    pid->integrator += error * pid->config.ki * dt_s;
    pid->integrator = clamp_float(pid->integrator,
                                  pid->config.integrator_min,
                                  pid->config.integrator_max);
    if (pid->has_previous_error)
    {
        derivative = (error - pid->previous_error) / dt_s;
    }

    output = (pid->config.kp * error) + pid->integrator + (pid->config.kd * derivative);
    output = clamp_float(output, pid->config.output_min, pid->config.output_max);
    pid->previous_error = error;
    pid->has_previous_error = true;
    pid->last_output = output;
    return output;
}

FcStatus_t Ctl_PidSetConfig(CtlPid_t *pid, const CtlPidConfig_t *config)
{
    if ((pid == NULL) || !config_valid(config))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    pid->config = *config;
    pid->initialized = true;
    Ctl_PidReset(pid);
    return FC_STATUS_OK;
}

