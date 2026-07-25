#ifndef CTL_RATE_H
#define CTL_RATE_H

/* Three-axis angular-rate inner loop. */

#include "fc_types.h"
#include "ctl_pid.h"

typedef struct
{
    CtlPidTerms_t roll;
    CtlPidTerms_t pitch;
    CtlPidTerms_t yaw;
    bool valid;
} CtlRateDebug_t;

extern volatile CtlRateDebug_t g_ctl_rate_debug;

FcStatus_t Ctl_RateInit(void);
FcStatus_t Ctl_RateUpdate(const FcVector3f_t *target_rate_dps,
                          const FcImuData_t *imu,
                          float dt_s,
                          FcControlOutput_t *output);
void Ctl_RateReset(void);
FcStatus_t Ctl_RateGetDebug(CtlRateDebug_t *debug);

#endif /* CTL_RATE_H */
