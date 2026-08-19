#ifndef CTL_ATTITUDE_H
#define CTL_ATTITUDE_H

/* Attitude-angle outer loop producing target angular rates in deg/s. */

#include "fc_types.h"

typedef struct
{
    float yaw_target_deg;
    float yaw_error_deg;
    float yaw_center_time_ms;
    bool yaw_target_initialized;
    bool heading_hold_enabled;
    bool manual_rate_active;
    bool center_wait_active;
} CtlAttitudeDebug_t;

extern volatile CtlAttitudeDebug_t g_ctl_attitude_debug;

FcStatus_t Ctl_AttitudeInit(void);
FcStatus_t Ctl_AttitudeUpdate(const FcControlTarget_t *target,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              FcVector3f_t *target_rate_dps);
void Ctl_AttitudeReset(void);

#endif /* CTL_ATTITUDE_H */
