#ifndef CTL_RATE_H
#define CTL_RATE_H

/* Three-axis angular-rate inner loop. */

#include "fc_types.h"

FcStatus_t Ctl_RateInit(void);
FcStatus_t Ctl_RateUpdate(const FcVector3f_t *target_rate_dps,
                          const FcImuData_t *imu,
                          float dt_s,
                          FcControlOutput_t *output);
void Ctl_RateReset(void);

#endif /* CTL_RATE_H */

