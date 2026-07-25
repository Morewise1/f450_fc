#ifndef EST_ATTITUDE_H
#define EST_ATTITUDE_H

/* Six-axis Mahony attitude estimator; yaw is relative without magnetometer. */

#include "fc_types.h"

FcStatus_t Est_AttitudeInit(void);
FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu, float dt_s, FcAttitude_t *attitude);
void Est_AttitudeReset(void);

#endif /* EST_ATTITUDE_H */
