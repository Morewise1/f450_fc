#ifndef CTL_PID_H
#define CTL_PID_H

/* Configurable positional PID object with bounded state and debug terms. */

#include <stdbool.h>
#include "fc_types.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float output_offset;
    float input_deadband;
    float integral_min;
    float integral_max;
    float output_min;
    float output_max;
    float integral_separation_threshold;
    float variable_integral_full_error;
    float variable_integral_zero_error;
    float derivative_lpf_hz;
    bool enable_integral_separation;
    bool enable_variable_integral;
    bool enable_anti_windup;
    bool derivative_on_measurement;
    bool enable_derivative_lpf;
} CtlPidConfig_t;

typedef struct
{
    float error;
    float effective_error;
    float proportional;
    float integral;
    float derivative;
    float derivative_raw;
    float integral_scale;
    float output_before_limit;
    float output;
    bool output_saturated;
} CtlPidTerms_t;

typedef struct
{
    CtlPidConfig_t config;
    CtlPidTerms_t terms;
    float integrator;
    float previous_error;
    float previous_measurement;
    float derivative_state;
    float last_output;
    bool has_previous_sample;
    bool initialized;
} CtlPid_t;

FcStatus_t Ctl_PidInit(CtlPid_t *pid, const CtlPidConfig_t *config);
void Ctl_PidReset(CtlPid_t *pid);
float Ctl_PidUpdate(CtlPid_t *pid, float setpoint, float measurement, float dt_s);
FcStatus_t Ctl_PidSetConfig(CtlPid_t *pid, const CtlPidConfig_t *config);
FcStatus_t Ctl_PidGetConfig(const CtlPid_t *pid, CtlPidConfig_t *config);
FcStatus_t Ctl_PidGetTerms(const CtlPid_t *pid, CtlPidTerms_t *terms);

#endif /* CTL_PID_H */
