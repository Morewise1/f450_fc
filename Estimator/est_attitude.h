#ifndef EST_ATTITUDE_H
#define EST_ATTITUDE_H

/* Mahony attitude estimator with optional, rejectable magnetic yaw aiding. */

#include "fc_types.h"

typedef struct
{
    float level_roll_trim_deg;
    float level_pitch_trim_deg;
    uint32_t level_sample_count;
    float magnetic_heading_deg;
    float magnetic_yaw_error_deg;
    uint32_t magnetic_reject_count;
    bool aligned;
    bool level_calibrated;
    bool magnetic_heading_initialized;
    bool magnetic_aiding_active;
} EstAttitudeDebug_t;

/* Kept global so the startup calibration can be inspected directly in Keil Watch. */
extern volatile EstAttitudeDebug_t g_est_attitude_debug;

FcStatus_t Est_AttitudeInit(void);
FcStatus_t Est_AttitudeSetMagnetometer(const FcMagnetometerData_t *magnetometer);
FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu, float dt_s, FcAttitude_t *attitude);
void Est_AttitudeReset(void);

#endif /* EST_ATTITUDE_H */
