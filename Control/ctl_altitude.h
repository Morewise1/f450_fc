#ifndef CTL_ALTITUDE_H
#define CTL_ALTITUDE_H

/* Cascaded altitude/vertical-speed controller producing collective correction. */

#include "fc_types.h"

typedef struct
{
    float altitude_error_m;
    float target_vertical_velocity_mps;
    float velocity_error_mps;
    float integral_us;
    float output_us;
    bool saturated;
} CtlAltitudeDebug_t;

extern volatile CtlAltitudeDebug_t g_ctl_altitude_debug;

FcStatus_t Ctl_AltitudeInit(void);
FcStatus_t Ctl_AltitudeUpdate(float target_altitude_m,
                              const FcAltitude_t *altitude,
                              float dt_s,
                              float *throttle_correction_us);
void Ctl_AltitudeReset(void);

#endif /* CTL_ALTITUDE_H */
