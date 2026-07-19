#ifndef CTL_ATTITUDE_H
#define CTL_ATTITUDE_H

/* Attitude-angle outer loop producing target angular rates in deg/s. */

#include "fc_types.h"

FcStatus_t Ctl_AttitudeInit(void);
FcStatus_t Ctl_AttitudeUpdate(const FcControlTarget_t *target,
                              const FcAttitude_t *attitude,
                              float dt_s,
                              FcVector3f_t *target_rate_dps);
void Ctl_AttitudeReset(void);

#endif /* CTL_ATTITUDE_H */

