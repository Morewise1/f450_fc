#ifndef CTL_RC_MAP_H
#define CTL_RC_MAP_H

/* Maps normalized receiver channels into nonlinear pilot intent. */

#include "fc_types.h"

FcStatus_t Ctl_RcMapUpdate(const FcRcInput_t *input, FcPilotCommand_t *command);

#endif /* CTL_RC_MAP_H */
