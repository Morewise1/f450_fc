#ifndef CTL_MIXER_H
#define CTL_MIXER_H

/* Configurable Quad-X mixer; arming decisions remain in App safety. */

#include <stdint.h>
#include "fc_types.h"

FcStatus_t Ctl_MixerQuadX(uint16_t throttle_us,
                         float roll_cmd_us,
                         float pitch_cmd_us,
                         float yaw_cmd_us,
                         FcMotorOutput_t *output);
void Ctl_MixerSetStop(FcMotorOutput_t *output);

#endif /* CTL_MIXER_H */

