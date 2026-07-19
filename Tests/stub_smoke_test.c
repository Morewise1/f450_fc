/* Host-side safety, scheduler, and i-BUS parser smoke test. */

#include <stdint.h>
#include "app_flight.h"
#include "app_main.h"
#include "app_scheduler.h"
#include "bsp_esc_pwm.h"
#include "drv_ibus.h"
#include "fc_config.h"
#include "fc_params.h"

static int motors_are_stopped(const FcMotorOutput_t *output)
{
    uint32_t index;
    if ((output == 0) || !output->valid)
    {
        return 0;
    }
    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        if (output->motor_us[index] != FC_ESC_STOP_US)
        {
            return 0;
        }
    }
    return 1;
}

static void set_ibus_channel(uint8_t frame[FC_IBUS_FRAME_LENGTH], uint32_t channel, uint16_t value)
{
    uint32_t offset = 2U + (channel * 2U);
    frame[offset] = (uint8_t)(value & 0xFFU);
    frame[offset + 1U] = (uint8_t)(value >> 8U);
}

static void update_ibus_checksum(uint8_t frame[FC_IBUS_FRAME_LENGTH])
{
    uint32_t index;
    uint16_t checksum = 0xFFFFU;

    for (index = 0U; index < (FC_IBUS_FRAME_LENGTH - 2U); ++index)
    {
        checksum = (uint16_t)(checksum - frame[index]);
    }
    frame[FC_IBUS_FRAME_LENGTH - 2U] = (uint8_t)(checksum & 0xFFU);
    frame[FC_IBUS_FRAME_LENGTH - 1U] = (uint8_t)(checksum >> 8U);
}

static void build_ibus_frame(uint8_t frame[FC_IBUS_FRAME_LENGTH])
{
    uint32_t channel;
    uint32_t index;

    for (index = 0U; index < FC_IBUS_FRAME_LENGTH; ++index)
    {
        frame[index] = 0U;
    }
    frame[0] = FC_IBUS_LENGTH_BYTE;
    frame[1] = FC_IBUS_COMMAND_BYTE;
    for (channel = 0U; channel < FC_IBUS_CHANNEL_COUNT; ++channel)
    {
        set_ibus_channel(frame, channel, FC_IBUS_RAW_CENTER);
    }
    set_ibus_channel(frame, FC_IBUS_CHANNEL_ROLL, FC_IBUS_RAW_MAX);
    set_ibus_channel(frame, FC_IBUS_CHANNEL_PITCH, FC_IBUS_RAW_MIN);
    set_ibus_channel(frame, FC_IBUS_CHANNEL_THROTTLE, FC_IBUS_RAW_MIN);
    set_ibus_channel(frame, FC_IBUS_CHANNEL_YAW, FC_IBUS_RAW_MAX);
    set_ibus_channel(frame, FC_IBUS_CHANNEL_ARM, FC_IBUS_RAW_MIN);
    set_ibus_channel(frame, FC_IBUS_CHANNEL_MODE, FC_IBUS_RAW_MAX);
    update_ibus_checksum(frame);
}

int main(void)
{
    FcStatus_t init_status;
    FcMotorOutput_t flight_output;
    FcMotorOutput_t esc_output;
    AppFlightTaskStats_t task_stats;
    FcRcInput_t rc_input;
    DrvIbusStats_t ibus_stats;
    uint16_t raw_channels[FC_IBUS_CHANNEL_COUNT];
    uint8_t ibus_frame[FC_IBUS_FRAME_LENGTH];
    uint32_t millisecond;

    init_status = App_MainInit();
    if ((init_status == FC_STATUS_OK) || (init_status == FC_STATUS_NOT_INITIALIZED)) { return 1; }

    for (millisecond = 0U; millisecond < 100U; ++millisecond)
    {
        App_Scheduler1msTick();
        App_MainLoop();
    }

    if (App_FlightGetState() != FC_STATE_STOP) { return 2; }
    if (App_FlightGetMotorOutput(&flight_output) != FC_STATUS_OK) { return 3; }
    if (!motors_are_stopped(&flight_output)) { return 4; }
    if (BSP_EscPwm_GetLastCommand(&esc_output) != FC_STATUS_OK) { return 5; }
    if (!motors_are_stopped(&esc_output)) { return 6; }
    if (App_FlightGetTaskStats(&task_stats) != FC_STATUS_OK) { return 7; }
    if (task_stats.task_500hz_count != 50U) { return 8; }
    if (task_stats.task_250hz_count != 25U) { return 9; }
    if (task_stats.task_100hz_count != 10U) { return 10; }
    if (task_stats.task_50hz_count != 5U) { return 11; }
    if (task_stats.task_10hz_count != 1U) { return 12; }

    build_ibus_frame(ibus_frame);
    if (Drv_Ibus_ProcessBuffer(ibus_frame, FC_IBUS_FRAME_LENGTH, 100U) != 1U) { return 13; }
    if (Drv_Ibus_GetInput(&rc_input) != FC_STATUS_OK) { return 14; }
    if ((rc_input.roll != 500) || (rc_input.pitch != -500) ||
        (rc_input.yaw != 500) || (rc_input.throttle != 0U)) { return 15; }
    if (!rc_input.throttle_low || rc_input.arm_switch || !rc_input.mode_switch) { return 16; }
    if (rc_input.safety_switch) { return 17; }
    if (Drv_Ibus_GetRawChannels(raw_channels) != FC_STATUS_OK) { return 18; }
    if (raw_channels[FC_IBUS_CHANNEL_ROLL] != FC_IBUS_RAW_MAX) { return 19; }

    ibus_frame[2] ^= 0x01U;
    if (Drv_Ibus_ProcessBuffer(ibus_frame, FC_IBUS_FRAME_LENGTH, 110U) != 0U) { return 20; }
    if (Drv_Ibus_GetInput(&rc_input) != FC_STATUS_OK) { return 21; }
    if ((rc_input.last_frame_ms != 100U) || (rc_input.roll != 500)) { return 22; }

    build_ibus_frame(ibus_frame);
    set_ibus_channel(ibus_frame, FC_IBUS_CHANNEL_ROLL, 2500U);
    update_ibus_checksum(ibus_frame);
    if (Drv_Ibus_ProcessBuffer(ibus_frame, FC_IBUS_FRAME_LENGTH, 120U) != 0U) { return 23; }

    build_ibus_frame(ibus_frame);
    for (millisecond = 0U; millisecond < 5U; ++millisecond)
    {
        if (Drv_Ibus_ProcessByte(ibus_frame[millisecond], 130U)) { return 24; }
    }
    if (Drv_Ibus_ProcessBuffer(ibus_frame, FC_IBUS_FRAME_LENGTH, 140U) != 1U) { return 25; }

    Drv_Ibus_UpdateTimeout(241U);
    if (Drv_Ibus_GetInput(&rc_input) != FC_STATUS_TIMEOUT) { return 26; }
    if (!rc_input.failsafe || rc_input.link_valid || rc_input.throttle_low ||
        (rc_input.roll != 0) || (rc_input.pitch != 0) ||
        (rc_input.yaw != 0) || (rc_input.throttle != 0U) || rc_input.arm_switch) { return 27; }

    if (Drv_Ibus_GetStats(&ibus_stats) != FC_STATUS_OK) { return 28; }
    if ((ibus_stats.valid_frame_count != 2U) ||
        (ibus_stats.checksum_error_count != 1U) ||
        (ibus_stats.range_error_count != 1U) ||
        (ibus_stats.sync_reset_count != 1U) ||
        (ibus_stats.timeout_count != 1U)) { return 29; }
    return 0;
}
