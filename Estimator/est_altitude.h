#ifndef EST_ALTITUDE_H
#define EST_ALTITUDE_H

/* Barometer/ToF altitude-estimator contract for the later altitude phase. */

#include "fc_types.h"

FcStatus_t Est_AltitudeInit(void);
FcStatus_t Est_AltitudeUpdate(const FcBarometerData_t *barometer,
                              const FcRangeData_t *range,
                              float dt_s,
                              FcAltitude_t *altitude);
void Est_AltitudeReset(void);

#endif /* EST_ALTITUDE_H */

