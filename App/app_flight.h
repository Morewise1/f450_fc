#ifndef APP_FLIGHT_H
#define APP_FLIGHT_H

/* Flight state machine and periodic task orchestration. */

#include <stdint.h>
#include "fc_types.h"

typedef struct
{
    uint32_t task_500hz_count;
    uint32_t task_250hz_count;
    uint32_t task_100hz_count;
    uint32_t task_50hz_count;
    uint32_t task_10hz_count;
} AppFlightTaskStats_t;

FcStatus_t App_FlightInit(void);
void App_FlightTask500Hz(void);
void App_FlightTask250Hz(void);
void App_FlightTask100Hz(void);
void App_FlightTask50Hz(void);
void App_FlightTask10Hz(void);
FcFlightState_t App_FlightGetState(void);
FcFlightMode_t App_FlightGetMode(void);
FcStatus_t App_FlightGetMotorOutput(FcMotorOutput_t *output);
FcStatus_t App_FlightGetTaskStats(AppFlightTaskStats_t *stats);

#endif /* APP_FLIGHT_H */
