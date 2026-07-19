/* Safe stub: never produces a throttle correction before altitude phase work. */

#include <stddef.h>
#include "ctl_altitude.h"

FcStatus_t Ctl_AltitudeInit(void)
{
    return FC_STATUS_OK;
}

FcStatus_t Ctl_AltitudeUpdate(float target_altitude_m,
                              const FcAltitude_t *altitude,
                              float dt_s,
                              float *throttle_correction_us)
{
    (void)target_altitude_m;
    if ((altitude == NULL) || (throttle_correction_us == NULL) || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *throttle_correction_us = 0.0f;
    return FC_STATUS_NOT_IMPLEMENTED;
}

void Ctl_AltitudeReset(void)
{
}
