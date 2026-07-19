/* Safe stub: altitude remains invalid until the second project phase. */

#include <stddef.h>
#include "est_altitude.h"

FcStatus_t Est_AltitudeInit(void)
{
    return FC_STATUS_OK;
}

FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcRangeData_t *range,
                              float dt_s,
                              FcAltitude_t *altitude)
{
    (void)range;
    if ((altitude == NULL) || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *altitude = (FcAltitude_t){0};
    if (barometer != NULL)
    {
        altitude->timestamp_ms = barometer->timestamp_ms;
    }
    altitude->valid = false;
    return FC_STATUS_NOT_IMPLEMENTED;
}

void Est_AltitudeReset(void)
{
}
