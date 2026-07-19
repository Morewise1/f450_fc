#ifndef APP_MAIN_H
#define APP_MAIN_H

/* Top-level initialization and non-blocking main-loop integration API. */

#include "fc_types.h"

FcStatus_t App_MainInit(void);
void App_MainLoop(void);
FcStatus_t App_MainGetStatus(void);

/* Short aliases for CubeMX main.c integration. */
FcStatus_t App_Init(void);
void App_Loop(void);

#endif /* APP_MAIN_H */

