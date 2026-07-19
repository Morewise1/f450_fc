/* Safe stub: never invents attitude from missing or unimplemented sensor data. */

#include <stddef.h>
#include "est_attitude.h"

FcStatus_t Est_AttitudeInit(void)
{
    return FC_STATUS_OK;
}

FcStatus_t Est_AttitudeUpdate(const FcImuData_t *imu, float dt_s, FcAttitude_t *attitude)
{
    if ((imu == NULL) || (attitude == NULL) || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *attitude = (FcAttitude_t){0};
    attitude->timestamp_ms = imu->timestamp_ms;
    attitude->valid = false;
    return imu->valid ? FC_STATUS_NOT_IMPLEMENTED : FC_STATUS_INVALID_DATA;
}

void Est_AttitudeReset(void)
{
}

