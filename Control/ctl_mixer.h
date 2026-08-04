#ifndef CTL_MIXER_H
#define CTL_MIXER_H

/* Configurable Quad-X mixer; arming decisions remain in App safety. */

#include <stdint.h>
#include "fc_types.h"

typedef struct
{
    float roll_pitch_scale;
    float yaw_scale;
    float requested_yaw_us;
    float applied_yaw_us;
    bool saturated;
} CtlMixerDebug_t;

/* Read-only flight diagnostic for Keil Watch. */
extern volatile CtlMixerDebug_t g_ctl_mixer_debug;

FcStatus_t Ctl_MixerQuadX(uint16_t throttle_us,
                         float roll_cmd_us,
                         float pitch_cmd_us,
                         float yaw_cmd_us,
                         FcMotorOutput_t *output);
void Ctl_MixerSetStop(FcMotorOutput_t *output);

#endif /* CTL_MIXER_H */
