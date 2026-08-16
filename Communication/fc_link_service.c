#include <stddef.h>
#include <string.h>
#include "fc_link_service.h"
#include "fc_link_protocol.h"
#include "app_command_mux.h"
#include "app_flight.h"
#include "app_scheduler.h"
#include "bsp_esp_uart.h"

#define FAST_TELEMETRY_PERIOD_MS 40U
#define STATUS_PERIOD_MS         100U
#define RX_BYTES_PER_SERVICE     128U

#define VALID_IMU        (1U << 0)
#define VALID_ATTITUDE   (1U << 1)
#define VALID_BAROMETER  (1U << 2)
#define VALID_ALTITUDE   (1U << 3)
#define VALID_MAGNETIC   (1U << 4)
#define VALID_MOTOR      (1U << 5)
#define VALID_RC         (1U << 6)
#define VALID_SAFETY     (1U << 7)

static FcLinkParser_t s_parser;
static FcLinkStats_t s_stats;
static uint16_t s_tx_sequence;
static uint32_t s_next_fast_ms;
static uint32_t s_next_status_ms;
static uint8_t s_tx_frame[FC_LINK_MAX_FRAME];
static uint8_t s_pending_frame[FC_LINK_MAX_FRAME];
static uint16_t s_pending_length;
static bool s_initialized;

static bool time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static int16_t scaled_i16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f) { return INT16_MAX; }
    if (scaled < -32768.0f) { return INT16_MIN; }
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t scaled_i32(float value, float scale)
{
    double scaled = (double)value * (double)scale;
    if (scaled > 2147483647.0) { return INT32_MAX; }
    if (scaled < -2147483648.0) { return INT32_MIN; }
    return (int32_t)(scaled + ((scaled >= 0.0) ? 0.5 : -0.5));
}

static uint32_t positive_u32(float value)
{
    if (value <= 0.0f) { return 0U; }
    if (value >= 4294967295.0f) { return UINT32_MAX; }
    return (uint32_t)(value + 0.5f);
}

static uint8_t result_from_status(FcStatus_t status)
{
    switch (status)
    {
        case FC_STATUS_OK: return FC_LINK_RESULT_OK;
        case FC_STATUS_NOT_IMPLEMENTED: return FC_LINK_RESULT_DISABLED;
        case FC_STATUS_NOT_READY: return FC_LINK_RESULT_NOT_READY;
        case FC_STATUS_INVALID_ARGUMENT:
        case FC_STATUS_INVALID_DATA: return FC_LINK_RESULT_BAD_VALUE;
        default: return FC_LINK_RESULT_WRONG_STATE;
    }
}

static void queue_response(uint8_t original_type,
                           uint16_t original_sequence,
                           FcStatus_t status,
                           uint32_t now_ms)
{
    uint8_t payload[4];
    uint16_t length = 0U;

    payload[0] = original_type;
    payload[1] = result_from_status(status);
    FcLink_WriteU16Le(&payload[2], original_sequence);
    if (FcLink_Encode((status == FC_STATUS_OK) ?
                      FC_LINK_MSG_ACK : FC_LINK_MSG_ERROR,
                      s_tx_sequence++,
                      now_ms,
                      payload,
                      sizeof(payload),
                      s_pending_frame,
                      sizeof(s_pending_frame),
                      &length) == FC_STATUS_OK)
    {
        s_pending_length = length;
    }
}

static void handle_control(const FcLinkFrame_t *frame, uint32_t now_ms)
{
    AppRemoteControl_t command;

    if (frame->payload_length != 16U)
    {
        ++s_stats.rx_errors;
        return;
    }
    command.session_id = FcLink_ReadU32Le(&frame->payload[0]);
    command.roll = FcLink_ReadI16Le(&frame->payload[4]);
    command.pitch = FcLink_ReadI16Le(&frame->payload[6]);
    command.yaw = FcLink_ReadI16Le(&frame->payload[8]);
    command.throttle = FcLink_ReadU16Le(&frame->payload[10]);
    command.requested_mode = frame->payload[12];
    command.flags = frame->payload[13];
    command.sequence = frame->sequence;
    if (App_CommandMuxApplyRemote(&command, now_ms) != FC_STATUS_OK)
    {
        ++s_stats.rx_errors;
    }
}

static void handle_command(const FcLinkFrame_t *frame, uint32_t now_ms)
{
    FcStatus_t status = FC_STATUS_INVALID_DATA;
    uint32_t session_id;

    s_stats.last_rx_sequence = frame->sequence;
    if (frame->type == FC_LINK_MSG_CONTROL)
    {
        handle_control(frame, now_ms);
        return;
    }
    if ((frame->type == FC_LINK_MSG_SET_CONTROL_SOURCE) &&
        (frame->payload_length == 5U))
    {
        session_id = FcLink_ReadU32Le(&frame->payload[0]);
        status = App_CommandMuxSetSource(
            (AppControlSource_t)frame->payload[4],
            session_id,
            App_FlightGetState());
    }
    else if ((frame->type == FC_LINK_MSG_ARM_REQUEST) &&
             (frame->payload_length == 4U))
    {
        session_id = FcLink_ReadU32Le(&frame->payload[0]);
        status = App_CommandMuxArmRemote(session_id,
                                         now_ms,
                                         App_FlightGetState());
    }
    else if ((frame->type == FC_LINK_MSG_DISARM) &&
             (frame->payload_length == 4U))
    {
        session_id = FcLink_ReadU32Le(&frame->payload[0]);
        status = App_CommandMuxDisarmRemote(session_id);
    }
    else if ((frame->type == FC_LINK_MSG_EMERGENCY_STOP) &&
             (frame->payload_length == 4U))
    {
        session_id = FcLink_ReadU32Le(&frame->payload[0]);
        status = App_CommandMuxEmergencyStop(session_id);
    }
    else
    {
        status = (frame->payload_length > FC_LINK_MAX_PAYLOAD) ?
                 FC_STATUS_INVALID_DATA : FC_STATUS_NOT_IMPLEMENTED;
    }
    queue_response(frame->type, frame->sequence, status, now_ms);
}

static uint16_t build_fast_payload(uint8_t *payload)
{
    AppFlightDebug_t debug;
    uint16_t validity = 0U;
    uint16_t offset = 0U;

    if (App_FlightGetDebug(&debug) != FC_STATUS_OK) { return 0U; }
    if (debug.imu.valid) { validity |= VALID_IMU; }
    if (debug.attitude.valid) { validity |= VALID_ATTITUDE; }
    if (debug.barometer.valid) { validity |= VALID_BAROMETER; }
    if (debug.altitude.valid) { validity |= VALID_ALTITUDE; }
    if (debug.magnetometer.valid) { validity |= VALID_MAGNETIC; }
    if (debug.motors.valid) { validity |= VALID_MOTOR; }
    if (debug.receiver.link_valid && !debug.receiver.failsafe)
    {
        validity |= VALID_RC;
    }
    validity |= VALID_SAFETY;

    FcLink_WriteU16Le(&payload[offset], validity); offset += 2U;
    payload[offset++] = (uint8_t)debug.state;
    payload[offset++] = (uint8_t)debug.mode;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.attitude.roll_deg, 100.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.attitude.pitch_deg, 100.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.attitude.yaw_deg, 100.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.imu.gyro_dps.x, 10.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.imu.gyro_dps.y, 10.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.imu.gyro_dps.z, 10.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.imu.accel_g.x, 1000.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.imu.accel_g.y, 1000.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.imu.accel_g.z, 1000.0f));
    offset += 2U;
    FcLink_WriteI32Le(&payload[offset],
                      scaled_i32(debug.altitude.altitude_m, 100.0f));
    offset += 4U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.altitude.vertical_velocity_mps, 100.0f));
    offset += 2U;
    FcLink_WriteU16Le(&payload[offset], debug.motors.motor_us[0]); offset += 2U;
    FcLink_WriteU16Le(&payload[offset], debug.motors.motor_us[1]); offset += 2U;
    FcLink_WriteU16Le(&payload[offset], debug.motors.motor_us[2]); offset += 2U;
    FcLink_WriteU16Le(&payload[offset], debug.motors.motor_us[3]); offset += 2U;
    FcLink_WriteU32Le(&payload[offset], debug.safety.active_faults); offset += 4U;
    FcLink_WriteU16Le(&payload[offset],
                      App_CommandMuxGetLastRemoteSequence());
    offset += 2U;
    return offset;
}

static uint16_t build_status_payload(uint8_t *payload, uint32_t now_ms)
{
    AppFlightDebug_t debug;
    AppSchedulerStats_t scheduler = {0};
    uint32_t remote_age_ms;
    uint16_t flags = 0U;
    uint16_t age;
    uint16_t offset = 0U;

    if (App_FlightGetDebug(&debug) != FC_STATUS_OK) { return 0U; }
    (void)App_SchedulerGetStats(&scheduler);
    if (debug.imu.calibrated) { flags |= 1U << 0; }
    if (debug.safety.rc_online) { flags |= 1U << 1; }
    if (debug.safety.scheduler_ok) { flags |= 1U << 2; }
    if (debug.safety.motor_output_allowed) { flags |= 1U << 3; }
    if (debug.safety.initialization_ok) { flags |= 1U << 4; }
    remote_age_ms = App_CommandMuxGetRemoteAgeMs(now_ms);
    age = remote_age_ms > UINT16_MAX ? UINT16_MAX : (uint16_t)remote_age_ms;

    FcLink_WriteU32Le(&payload[offset],
                      positive_u32(debug.barometer.pressure_pa));
    offset += 4U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.imu.temperature_c, 100.0f));
    offset += 2U;
    FcLink_WriteI16Le(&payload[offset],
                      scaled_i16(debug.barometer.temperature_c, 100.0f));
    offset += 2U;
    FcLink_WriteU32Le(&payload[offset], debug.safety.active_faults); offset += 4U;
    FcLink_WriteU16Le(&payload[offset], flags); offset += 2U;
    payload[offset++] = (uint8_t)App_CommandMuxGetSource();
    payload[offset++] = 0U;
    FcLink_WriteU16Le(&payload[offset], age); offset += 2U;
    FcLink_WriteU32Le(&payload[offset], scheduler.missed_deadline_count);
    offset += 4U;
    return offset;
}

static bool try_send(uint8_t type,
                     uint8_t *payload,
                     uint16_t payload_length,
                     uint32_t now_ms)
{
    uint16_t length = 0U;

    if (!BSP_EspUart_TxReady()) { return false; }
    if ((FcLink_Encode(type,
                       s_tx_sequence++,
                       now_ms,
                       payload,
                       payload_length,
                       s_tx_frame,
                       sizeof(s_tx_frame),
                       &length) == FC_STATUS_OK) &&
        (BSP_EspUart_WriteAsync(s_tx_frame, length) == FC_STATUS_OK))
    {
        ++s_stats.tx_frames;
        return true;
    }
    ++s_stats.tx_drops;
    return false;
}

FcStatus_t FcLink_Init(void)
{
    FcLink_ParserInit(&s_parser);
    (void)memset(&s_stats, 0, sizeof(s_stats));
    s_tx_sequence = 0U;
    s_next_fast_ms = 0U;
    s_next_status_ms = 0U;
    s_pending_length = 0U;
    s_initialized = true;
    return FC_STATUS_OK;
}

void FcLink_Service(void)
{
    FcLinkFrame_t frame;
    bool complete;
    uint8_t value;
    uint8_t payload[48];
    uint16_t processed = 0U;
    uint16_t length;
    uint32_t now_ms;

    if (!s_initialized) { return; }
    now_ms = App_SchedulerGetTickMs();
    while ((processed < RX_BYTES_PER_SERVICE) &&
           BSP_EspUart_ReadByte(&value))
    {
        FcStatus_t status = FcLink_ParserInput(&s_parser,
                                               value,
                                               &frame,
                                               &complete);
        if (status != FC_STATUS_OK) { ++s_stats.rx_errors; }
        if (complete)
        {
            ++s_stats.rx_frames;
            handle_command(&frame, now_ms);
        }
        ++processed;
    }

    if (s_pending_length != 0U)
    {
        if (BSP_EspUart_TxReady() &&
            (BSP_EspUart_WriteAsync(s_pending_frame,
                                    s_pending_length) == FC_STATUS_OK))
        {
            ++s_stats.tx_frames;
            s_pending_length = 0U;
        }
        return;
    }
    if (time_reached(now_ms, s_next_status_ms))
    {
        length = build_status_payload(payload, now_ms);
        if ((length != 0U) &&
            try_send(FC_LINK_MSG_STATUS, payload, length, now_ms))
        {
            s_next_status_ms = now_ms + STATUS_PERIOD_MS;
        }
        return;
    }
    if (time_reached(now_ms, s_next_fast_ms))
    {
        length = build_fast_payload(payload);
        if ((length != 0U) &&
            try_send(FC_LINK_MSG_FAST_TELEMETRY, payload, length, now_ms))
        {
            s_next_fast_ms = now_ms + FAST_TELEMETRY_PERIOD_MS;
        }
    }
}

FcStatus_t FcLink_GetStats(FcLinkStats_t *stats)
{
    if (stats == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    *stats = s_stats;
    return s_initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}
