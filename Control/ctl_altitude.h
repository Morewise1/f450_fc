#ifndef CTL_ALTITUDE_H
#define CTL_ALTITUDE_H

/* Altitude-control contract reserved for the second project phase. */

#include "fc_types.h"

FcStatus_t Ctl_AltitudeInit(void);
FcStatus_t Ctl_AltitudeUpdate(float target_altitude_m,
                              const FcAltitude_t *altitude,
                              float dt_s,
                              float *throttle_correction_us);
void Ctl_AltitudeReset(void);

#endif /* CTL_ALTITUDE_H */

