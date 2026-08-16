#include <stddef.h>
#include "app_command_mux.h"
#include "fc_config.h"

static FcRcInput_t s_ibus;
static FcRcInput_t s_remote;
static AppControlSource_t s_source;
static uint32_t s_session_id;
static uint32_t s_last_remote_ms;
static uint16_t s_last_remote_sequence;
static bool s_remote_seen;
static bool s_remote_armed;
static bool s_emergency_latched;
static bool s_initialized;

FcStatus_t App_CommandMuxInit(void)
{
    s_ibus = (FcRcInput_t){0};
    s_remote = (FcRcInput_t){0};
    s_source = APP_CONTROL_SOURCE_IBUS;
    s_session_id = 0U;
    s_last_remote_ms = 0U;
    s_last_remote_sequence = 0U;
    s_remote_seen = false;
    s_remote_armed = false;
    s_emergency_latched = false;
    s_initialized = true;
    return FC_STATUS_OK;
}

void App_CommandMuxUpdateIbus(const FcRcInput_t *input)
{
    if (s_initialized && (input != NULL)) { s_ibus = *input; }
}

FcStatus_t App_CommandMuxSetSource(AppControlSource_t source,
                                  uint32_t session_id,
                                  FcFlightState_t flight_state)
{
    if (!s_initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (flight_state != FC_STATE_STOP) { return FC_STATUS_NOT_READY; }
    if (source == APP_CONTROL_SOURCE_IBUS)
    {
        s_source = source;
        s_session_id = 0U;
        s_remote_armed = false;
        s_remote_seen = false;
        return FC_STATUS_OK;
    }
#if FC_ENABLE_APP_CONTROL
    if ((source != APP_CONTROL_SOURCE_WIFI) || (session_id == 0U))
    {
        return FC_STATUS_INVALID_DATA;
    }
    s_source = source;
    s_session_id = session_id;
    s_remote = (FcRcInput_t){0};
    s_remote_armed = false;
    s_remote_seen = false;
    return FC_STATUS_OK;
#else
    (void)session_id;
    return FC_STATUS_NOT_IMPLEMENTED;
#endif
}

FcStatus_t App_CommandMuxApplyRemote(const AppRemoteControl_t *command,
                                    uint32_t now_ms)
{
#if FC_ENABLE_APP_CONTROL
    if (command == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    if ((s_source != APP_CONTROL_SOURCE_WIFI) ||
        (command->session_id != s_session_id))
    {
        return FC_STATUS_INVALID_DATA;
    }
    if ((command->roll < FC_RC_AXIS_MIN) ||
        (command->roll > FC_RC_AXIS_MAX) ||
        (command->pitch < FC_RC_AXIS_MIN) ||
        (command->pitch > FC_RC_AXIS_MAX) ||
        (command->yaw < FC_RC_AXIS_MIN) ||
        (command->yaw > FC_RC_AXIS_MAX) ||
        (command->throttle > FC_RC_THROTTLE_MAX) ||
        (command->requested_mode > (uint8_t)FC_MODE_ALT_HOLD))
    {
        return FC_STATUS_INVALID_DATA;
    }
    s_remote.roll = command->roll;
    s_remote.pitch = command->pitch;
    s_remote.yaw = command->yaw;
    s_remote.throttle = command->throttle;
    s_remote.throttle_low = command->throttle <= FC_RC_THROTTLE_ARM_MAX;
    s_remote.arm_switch = s_remote_armed;
    s_remote.mode_switch =
        command->requested_mode == (uint8_t)FC_MODE_ALT_HOLD;
    s_remote.safety_switch =
        (command->flags & APP_REMOTE_FLAG_DEADMAN) != 0U;
    s_remote.emergency_stop = s_emergency_latched;
    s_remote.link_valid = true;
    s_remote.failsafe = false;
    s_remote.last_frame_ms = now_ms;
    s_last_remote_ms = now_ms;
    s_last_remote_sequence = command->sequence;
    s_remote_seen = true;
    return FC_STATUS_OK;
#else
    (void)command;
    (void)now_ms;
    return FC_STATUS_NOT_IMPLEMENTED;
#endif
}

FcStatus_t App_CommandMuxArmRemote(uint32_t session_id,
                                  uint32_t now_ms,
                                  FcFlightState_t flight_state)
{
#if FC_ENABLE_APP_CONTROL
    if ((s_source != APP_CONTROL_SOURCE_WIFI) ||
        (session_id != s_session_id))
    {
        return FC_STATUS_INVALID_DATA;
    }
    if ((flight_state != FC_STATE_STOP) || !s_remote_seen ||
        ((now_ms - s_last_remote_ms) > FC_APP_CONTROL_TIMEOUT_MS) ||
        !s_remote.throttle_low || !s_remote.safety_switch ||
        s_emergency_latched)
    {
        return FC_STATUS_NOT_READY;
    }
    s_remote_armed = true;
    s_remote.arm_switch = true;
    return FC_STATUS_OK;
#else
    (void)session_id;
    (void)now_ms;
    (void)flight_state;
    return FC_STATUS_NOT_IMPLEMENTED;
#endif
}

FcStatus_t App_CommandMuxDisarmRemote(uint32_t session_id)
{
    if ((s_source != APP_CONTROL_SOURCE_WIFI) ||
        (session_id != s_session_id))
    {
        return FC_STATUS_INVALID_DATA;
    }
    s_remote_armed = false;
    s_remote.arm_switch = false;
    return FC_STATUS_OK;
}

FcStatus_t App_CommandMuxEmergencyStop(uint32_t session_id)
{
    if ((s_source != APP_CONTROL_SOURCE_WIFI) ||
        (session_id != s_session_id))
    {
        return FC_STATUS_INVALID_DATA;
    }
    s_emergency_latched = true;
    s_remote_armed = false;
    s_remote.arm_switch = false;
    s_remote.emergency_stop = true;
    return FC_STATUS_OK;
}

FcStatus_t App_CommandMuxGetInput(uint32_t now_ms, FcRcInput_t *output)
{
    if (output == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    if (!s_initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (s_source == APP_CONTROL_SOURCE_IBUS)
    {
        *output = s_ibus;
        return output->link_valid && !output->failsafe ?
               FC_STATUS_OK : FC_STATUS_INVALID_DATA;
    }
    *output = s_remote;
    output->arm_switch = s_remote_armed;
    output->emergency_stop = s_emergency_latched;
    if (!s_remote_seen ||
        ((now_ms - s_last_remote_ms) > FC_APP_CONTROL_TIMEOUT_MS))
    {
        output->link_valid = false;
        output->failsafe = true;
        output->arm_switch = false;
        output->safety_switch = false;
        return FC_STATUS_TIMEOUT;
    }
    return output->link_valid && !output->failsafe ?
           FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

AppControlSource_t App_CommandMuxGetSource(void) { return s_source; }
uint16_t App_CommandMuxGetLastRemoteSequence(void)
{
    return s_last_remote_sequence;
}
uint32_t App_CommandMuxGetRemoteAgeMs(uint32_t now_ms)
{
    return s_remote_seen ? (now_ms - s_last_remote_ms) : UINT32_MAX;
}
