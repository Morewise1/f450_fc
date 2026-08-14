#ifndef EST_INERTIAL_NAV_H
#define EST_INERTIAL_NAV_H

/*
 * Short-term inertial XY diagnostic only.  With no GPS/optical-flow position
 * observation, velocity and position necessarily drift and MUST NOT feed PID.
 */

#include "fc_types.h"

typedef struct
{
    FcVector3f_t earth_acceleration_mps2;
    FcVector3f_t estimated_velocity_mps;
    FcVector3f_t estimated_position_m;
    uint32_t update_count;
    uint32_t stationary_reset_count;
    bool stationary;
    bool valid;
} EstInertialNavDebug_t;

extern volatile EstInertialNavDebug_t g_est_inertial_nav_debug;

FcStatus_t Est_InertialNavInit(void);
FcStatus_t Est_InertialNavUpdate(const FcImuData_t *imu,
                                 const FcAttitude_t *attitude,
                                 bool aircraft_stopped,
                                 float dt_s);
void Est_InertialNavReset(void);

#endif /* EST_INERTIAL_NAV_H */
