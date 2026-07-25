/* Advanced bounded positional PID. No allocation and no hidden time source. */

#include <stddef.h>
#include "ctl_pid.h"

#define TWO_PI_F 6.2831853071795864769f

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool config_valid(const CtlPidConfig_t *config)
{
    return (config != NULL) &&
           (config->integral_min <= config->integral_max) &&
           (config->output_min <= config->output_max) &&
           (config->input_deadband >= 0.0f) &&
           (config->integral_separation_threshold >= 0.0f) &&
           (config->variable_integral_full_error >= 0.0f) &&
           (config->variable_integral_zero_error >= config->variable_integral_full_error) &&
           (config->derivative_lpf_hz >= 0.0f);
}

static float apply_deadband(float error, float deadband)
{
    if (error > deadband) { return error - deadband; }
    if (error < -deadband) { return error + deadband; }
    return 0.0f;
}

static float calculate_integral_scale(const CtlPidConfig_t *config,
                                      float effective_error)
{
    float magnitude = absolute_float(effective_error);
    float scale = 1.0f;

    if (config->enable_integral_separation &&
        (magnitude > config->integral_separation_threshold))
    {
        return 0.0f;
    }

    if (config->enable_variable_integral)
    {
        if (magnitude >= config->variable_integral_zero_error)
        {
            scale = 0.0f;
        }
        else if (magnitude > config->variable_integral_full_error)
        {
            float span = config->variable_integral_zero_error -
                         config->variable_integral_full_error;
            scale = (span > 0.0f) ?
                (config->variable_integral_zero_error - magnitude) / span : 0.0f;
        }
    }
    return clamp_float(scale, 0.0f, 1.0f);
}

static float calculate_derivative(CtlPid_t *pid,
                                  float effective_error,
                                  float measurement,
                                  float dt_s,
                                  float *raw_derivative)
{
    float derivative = 0.0f;

    *raw_derivative = 0.0f;
    if (pid->has_previous_sample)
    {
        if (pid->config.derivative_on_measurement)
        {
            *raw_derivative = -(measurement - pid->previous_measurement) / dt_s;
        }
        else
        {
            *raw_derivative = (effective_error - pid->previous_error) / dt_s;
        }

        if (pid->config.enable_derivative_lpf &&
            (pid->config.derivative_lpf_hz > 0.0f))
        {
            float omega_dt = TWO_PI_F * pid->config.derivative_lpf_hz * dt_s;
            float alpha = omega_dt / (1.0f + omega_dt);
            pid->derivative_state += alpha * (*raw_derivative - pid->derivative_state);
            derivative = pid->derivative_state;
        }
        else
        {
            pid->derivative_state = *raw_derivative;
            derivative = *raw_derivative;
        }
    }
    return derivative;
}

FcStatus_t Ctl_PidInit(CtlPid_t *pid, const CtlPidConfig_t *config)
{
    if ((pid == NULL) || !config_valid(config))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *pid = (CtlPid_t){0};
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
    pid->terms = (CtlPidTerms_t){0};
    pid->integrator = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->derivative_state = 0.0f;
    pid->last_output = pid->config.output_offset;
    pid->has_previous_sample = false;
}

float Ctl_PidUpdate(CtlPid_t *pid, float setpoint, float measurement, float dt_s)
{
    float error;
    float effective_error;
    float integral_scale;
    float candidate_integrator;
    float derivative_rate;
    float raw_derivative;
    float base_output;
    float candidate_output;
    float output;
    bool reject_integral = false;

    if ((pid == NULL) || !pid->initialized || (dt_s <= 0.0f) || (dt_s > 1.0f))
    {
        return 0.0f;
    }

    error = setpoint - measurement;
    effective_error = apply_deadband(error, pid->config.input_deadband);
    integral_scale = calculate_integral_scale(&pid->config, effective_error);
    derivative_rate = calculate_derivative(pid,
                                           effective_error,
                                           measurement,
                                           dt_s,
                                           &raw_derivative);

    candidate_integrator = pid->integrator +
        (effective_error * pid->config.ki * integral_scale * dt_s);
    candidate_integrator = clamp_float(candidate_integrator,
                                       pid->config.integral_min,
                                       pid->config.integral_max);

    base_output = pid->config.output_offset +
                  (pid->config.kp * effective_error) +
                  (pid->config.kd * derivative_rate);
    candidate_output = base_output + candidate_integrator;

    if (pid->config.enable_anti_windup)
    {
        reject_integral = ((candidate_output > pid->config.output_max) &&
                           (effective_error > 0.0f)) ||
                          ((candidate_output < pid->config.output_min) &&
                           (effective_error < 0.0f));
    }
    if (!reject_integral)
    {
        pid->integrator = candidate_integrator;
    }

    pid->terms.error = error;
    pid->terms.effective_error = effective_error;
    pid->terms.proportional = pid->config.kp * effective_error;
    pid->terms.integral = pid->integrator;
    pid->terms.derivative_raw = raw_derivative;
    pid->terms.derivative = pid->config.kd * derivative_rate;
    pid->terms.integral_scale = integral_scale;
    pid->terms.output_before_limit = pid->config.output_offset +
                                     pid->terms.proportional +
                                     pid->terms.integral +
                                     pid->terms.derivative;
    output = clamp_float(pid->terms.output_before_limit,
                         pid->config.output_min,
                         pid->config.output_max);
    pid->terms.output_saturated = output != pid->terms.output_before_limit;
    pid->terms.output = output;

    pid->previous_error = effective_error;
    pid->previous_measurement = measurement;
    pid->has_previous_sample = true;
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

FcStatus_t Ctl_PidGetConfig(const CtlPid_t *pid, CtlPidConfig_t *config)
{
    if ((pid == NULL) || (config == NULL))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *config = pid->config;
    return pid->initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}

FcStatus_t Ctl_PidGetTerms(const CtlPid_t *pid, CtlPidTerms_t *terms)
{
    if ((pid == NULL) || (terms == NULL))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *terms = pid->terms;
    return pid->initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}
