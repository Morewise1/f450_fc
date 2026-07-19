#ifndef CTL_PID_H
#define CTL_PID_H

/* Generic PID with explicit seconds-based dt and hard integral/output limits. */

#include <stdbool.h>
#include "fc_types.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integrator_min;
    float integrator_max;
    float output_min;
    float output_max;
} CtlPidConfig_t;

typedef struct
{
    CtlPidConfig_t config;
    float integrator;
    float previous_error;
    float last_output;
    bool has_previous_error;
    bool initialized;
} CtlPid_t;

FcStatus_t Ctl_PidInit(CtlPid_t *pid, const CtlPidConfig_t *config);
void Ctl_PidReset(CtlPid_t *pid);
float Ctl_PidUpdate(CtlPid_t *pid, float setpoint, float measurement, float dt_s);
FcStatus_t Ctl_PidSetConfig(CtlPid_t *pid, const CtlPidConfig_t *config);

#endif /* CTL_PID_H */

