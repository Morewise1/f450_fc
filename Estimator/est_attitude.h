#ifndef EST_ATTITUDE_H
#define EST_ATTITUDE_H

/* Six-axis Mahony attitude estimator; yaw is relative without magnetometer. */

#include "fc_types.h"

typedef struct
{
    float level_roll_trim_deg;
    float level_pitch_trim_deg;
    uint32_t level_sample_count;
    bool aligned;
    bool level_calibrated;
} EstAttitudeDebug_t;

/* Kept global so the startup calibration can be inspected directly in Keil Watch. */
extern volatile EstAttitudeDebug_t g_est_attitude_debug;

FcStatus_t Est_AttitudeInit(void);
FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu, float dt_s, FcAttitude_t *attitude);
void Est_AttitudeReset(void);

#endif /* EST_ATTITUDE_H */
