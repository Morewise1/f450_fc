#ifndef APP_COMMAND_MUX_H
#define APP_COMMAND_MUX_H

#include <stdint.h>
#include "fc_types.h"

typedef enum
{
    APP_CONTROL_SOURCE_IBUS = 0,
    APP_CONTROL_SOURCE_WIFI = 1
} AppControlSource_t;

typedef struct
{
    uint32_t session_id;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint16_t throttle;
    uint8_t requested_mode;
    uint8_t flags;
    uint16_t sequence;
} AppRemoteControl_t;

#define APP_REMOTE_FLAG_DEADMAN (1U << 0)

FcStatus_t App_CommandMuxInit(void);
void App_CommandMuxUpdateIbus(const FcRcInput_t *input);
FcStatus_t App_CommandMuxSetSource(AppControlSource_t source,
                                  uint32_t session_id,
                                  FcFlightState_t flight_state);
FcStatus_t App_CommandMuxApplyRemote(const AppRemoteControl_t *command,
                                    uint32_t now_ms);
FcStatus_t App_CommandMuxArmRemote(uint32_t session_id,
                                  uint32_t now_ms,
                                  FcFlightState_t flight_state);
FcStatus_t App_CommandMuxDisarmRemote(uint32_t session_id);
FcStatus_t App_CommandMuxEmergencyStop(uint32_t session_id);
FcStatus_t App_CommandMuxGetInput(uint32_t now_ms, FcRcInput_t *output);
AppControlSource_t App_CommandMuxGetSource(void);
uint16_t App_CommandMuxGetLastRemoteSequence(void);
uint32_t App_CommandMuxGetRemoteAgeMs(uint32_t now_ms);

#endif /* APP_COMMAND_MUX_H */
