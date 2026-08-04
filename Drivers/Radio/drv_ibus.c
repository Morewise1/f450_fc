/* Robust 32-byte i-BUS parser with fail-closed timeout and no HAL ownership. */

#include <stddef.h>
#include "drv_ibus.h"
#include "fc_config.h"
#include "fc_params.h"

#if FC_USE_STM32_HAL
#include "main.h"
#endif

static uint8_t s_frame[FC_IBUS_FRAME_LENGTH];
static uint16_t s_raw_channels[FC_IBUS_CHANNEL_COUNT];
static uint8_t s_index;
static uint32_t s_last_byte_ms;
static bool s_have_last_byte;
static FcRcInput_t s_input;
static DrvIbusStats_t s_stats;
static bool s_initialized;

static uint32_t enter_critical(void)
{
#if FC_USE_STM32_HAL
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    return 0U;
#endif
}

static void exit_critical(uint32_t state)
{
#if FC_USE_STM32_HAL
    if (state == 0U)
    {
        __enable_irq();
    }
#else
    (void)state;
#endif
}

static uint16_t clamp_u16(uint16_t value, uint16_t minimum, uint16_t maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

static int16_t clamp_axis(int32_t value)
{
    if (value < FC_RC_AXIS_MIN) { return FC_RC_AXIS_MIN; }
    if (value > FC_RC_AXIS_MAX) { return FC_RC_AXIS_MAX; }
    return (int16_t)value;
}

static uint16_t clamp_throttle(int32_t value)
{
    if (value < (int32_t)FC_RC_THROTTLE_MIN) { return FC_RC_THROTTLE_MIN; }
    if (value > (int32_t)FC_RC_THROTTLE_MAX) { return FC_RC_THROTTLE_MAX; }
    return (uint16_t)value;
}

static bool decode_switch(uint16_t value, uint32_t active_high)
{
    bool above_threshold = value >= FC_IBUS_SWITCH_THRESHOLD_RAW;
    return (active_high != 0U) ? above_threshold : !above_threshold;
}

static void make_failsafe_input(FcRcInput_t *input, uint32_t last_frame_ms)
{
    *input = (FcRcInput_t){0};
    input->throttle_low = false;
    input->link_valid = false;
    input->failsafe = true;
    input->last_frame_ms = last_frame_ms;
}

static uint16_t read_frame_channel(uint8_t channel)
{
    uint8_t offset = (uint8_t)(2U + (channel * 2U));
    return (uint16_t)s_frame[offset] | ((uint16_t)s_frame[offset + 1U] << 8U);
}

static bool frame_checksum_valid(void)
{
    uint32_t index;
    uint16_t checksum = 0xFFFFU;
    uint16_t received;

    for (index = 0U; index < (FC_IBUS_FRAME_LENGTH - 2U); ++index)
    {
        checksum = (uint16_t)(checksum - s_frame[index]);
    }
    received = (uint16_t)s_frame[FC_IBUS_FRAME_LENGTH - 2U] |
               ((uint16_t)s_frame[FC_IBUS_FRAME_LENGTH - 1U] << 8U);
    return checksum == received;
}

static bool core_channels_in_plausible_range(const uint16_t *channels)
{
    uint32_t channel;
    for (channel = 0U; channel <= FC_IBUS_CHANNEL_MODE; ++channel)
    {
        if ((channels[channel] < FC_IBUS_RAW_VALID_MIN) ||
            (channels[channel] > FC_IBUS_RAW_VALID_MAX))
        {
            return false;
        }
    }
    return true;
}

static bool decode_and_commit(uint32_t timestamp_ms)
{
    uint16_t channels[FC_IBUS_CHANNEL_COUNT];
    FcRcInput_t decoded;
    uint32_t channel;
    uint16_t roll_raw;
    uint16_t pitch_raw;
    uint16_t throttle_raw;
    uint16_t yaw_raw;
    uint16_t arm_raw;
    uint16_t mode_raw;

    for (channel = 0U; channel < FC_IBUS_CHANNEL_COUNT; ++channel)
    {
        channels[channel] = read_frame_channel((uint8_t)channel);
    }
    if (!core_channels_in_plausible_range(channels))
    {
        ++s_stats.range_error_count;
        return false;
    }

    roll_raw = clamp_u16(channels[FC_IBUS_CHANNEL_ROLL], FC_IBUS_RAW_MIN, FC_IBUS_RAW_MAX);
    pitch_raw = clamp_u16(channels[FC_IBUS_CHANNEL_PITCH], FC_IBUS_RAW_MIN, FC_IBUS_RAW_MAX);
    throttle_raw = clamp_u16(channels[FC_IBUS_CHANNEL_THROTTLE], FC_IBUS_RAW_MIN, FC_IBUS_RAW_MAX);
    yaw_raw = clamp_u16(channels[FC_IBUS_CHANNEL_YAW], FC_IBUS_RAW_MIN, FC_IBUS_RAW_MAX);
    arm_raw = clamp_u16(channels[FC_IBUS_CHANNEL_ARM], FC_IBUS_RAW_MIN, FC_IBUS_RAW_MAX);
    mode_raw = clamp_u16(channels[FC_IBUS_CHANNEL_MODE], FC_IBUS_RAW_MIN, FC_IBUS_RAW_MAX);

    decoded = (FcRcInput_t){0};
    decoded.roll = clamp_axis(((int32_t)roll_raw - (int32_t)FC_IBUS_RAW_CENTER) *
                              (int32_t)FC_RC_ROLL_SIGN);
    decoded.pitch = clamp_axis(((int32_t)pitch_raw - (int32_t)FC_IBUS_RAW_CENTER) *
                               (int32_t)FC_RC_PITCH_SIGN);
    decoded.yaw = clamp_axis(((int32_t)yaw_raw - (int32_t)FC_IBUS_RAW_CENTER) *
                             (int32_t)FC_RC_YAW_SIGN);
    decoded.throttle = clamp_throttle((int32_t)throttle_raw - FC_IBUS_RAW_MIN);
    decoded.throttle_low = decoded.throttle <= FC_RC_THROTTLE_ARM_MAX;
    decoded.arm_switch = decode_switch(arm_raw, FC_IBUS_ARM_ACTIVE_HIGH);
    decoded.mode_switch = decode_switch(mode_raw, FC_IBUS_MODE_ACTIVE_HIGH);

    /* FS-i6 auxiliary setup: CH5=SwD, used as both arm and safety permission. */
    decoded.safety_switch = decoded.arm_switch;
    decoded.emergency_stop = false;
    decoded.link_valid = true;
    decoded.failsafe = false;
    decoded.last_frame_ms = timestamp_ms;

    for (channel = 0U; channel < FC_IBUS_CHANNEL_COUNT; ++channel)
    {
        s_raw_channels[channel] = channels[channel];
    }
    s_input = decoded;
    ++s_stats.valid_frame_count;
    s_stats.last_valid_frame_ms = timestamp_ms;
    return true;
}

FcStatus_t Drv_Ibus_Init(void)
{
    uint32_t channel;

    s_index = 0U;
    s_last_byte_ms = 0U;
    s_have_last_byte = false;
    for (channel = 0U; channel < FC_IBUS_CHANNEL_COUNT; ++channel)
    {
        s_raw_channels[channel] = 0U;
    }
    make_failsafe_input(&s_input, 0U);
    s_stats = (DrvIbusStats_t){0};
    s_initialized = true;
    return FC_STATUS_OK;
}

void Drv_Ibus_ResetParser(void)
{
    s_index = 0U;
    s_have_last_byte = false;
}

bool Drv_Ibus_ProcessByte(uint8_t byte, uint32_t timestamp_ms)
{
    bool accepted = false;

    if (!s_initialized)
    {
        return false;
    }

    if ((s_index > 0U) && s_have_last_byte &&
        ((uint32_t)(timestamp_ms - s_last_byte_ms) > FC_IBUS_INTERBYTE_TIMEOUT_MS))
    {
        s_index = 0U;
        ++s_stats.sync_reset_count;
    }
    s_last_byte_ms = timestamp_ms;
    s_have_last_byte = true;

    if (s_index == 0U)
    {
        if (byte == FC_IBUS_LENGTH_BYTE)
        {
            s_frame[0] = byte;
            s_index = 1U;
        }
        return false;
    }

    if (s_index == 1U)
    {
        if (byte != FC_IBUS_COMMAND_BYTE)
        {
            ++s_stats.format_error_count;
            if (byte == FC_IBUS_LENGTH_BYTE)
            {
                s_frame[0] = byte;
                s_index = 1U;
            }
            else
            {
                s_index = 0U;
            }
            return false;
        }
        s_frame[s_index++] = byte;
        return false;
    }

    s_frame[s_index++] = byte;
    if (s_index >= FC_IBUS_FRAME_LENGTH)
    {
        if (!frame_checksum_valid())
        {
            ++s_stats.checksum_error_count;
        }
        else
        {
            accepted = decode_and_commit(timestamp_ms);
        }
        s_index = 0U;
    }
    return accepted;
}

uint32_t Drv_Ibus_ProcessBuffer(const uint8_t *data, size_t length, uint32_t timestamp_ms)
{
    size_t index;
    uint32_t valid_frames = 0U;

    if ((data == NULL) || !s_initialized)
    {
        return 0U;
    }
    for (index = 0U; index < length; ++index)
    {
        if (Drv_Ibus_ProcessByte(data[index], timestamp_ms))
        {
            ++valid_frames;
        }
    }
    return valid_frames;
}

void Drv_Ibus_UpdateTimeout(uint32_t now_ms)
{
    uint32_t critical_state;
    uint32_t last_frame_ms;

    if (!s_initialized)
    {
        return;
    }

    critical_state = enter_critical();
    if (s_input.link_valid &&
        ((uint32_t)(now_ms - s_input.last_frame_ms) > FC_RC_TIMEOUT_MS))
    {
        last_frame_ms = s_input.last_frame_ms;
        make_failsafe_input(&s_input, last_frame_ms);
        ++s_stats.timeout_count;
    }
    exit_critical(critical_state);
}

FcStatus_t Drv_Ibus_GetInput(FcRcInput_t *input)
{
    uint32_t critical_state;

    if (input == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (!s_initialized)
    {
        make_failsafe_input(input, 0U);
        return FC_STATUS_NOT_INITIALIZED;
    }

    critical_state = enter_critical();
    *input = s_input;
    exit_critical(critical_state);
    return (input->link_valid && !input->failsafe) ? FC_STATUS_OK : FC_STATUS_TIMEOUT;
}

FcStatus_t Drv_Ibus_GetRawChannels(uint16_t channels[FC_IBUS_CHANNEL_COUNT])
{
    uint32_t critical_state;
    uint32_t channel;
    bool has_valid_frame;

    if (channels == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (!s_initialized)
    {
        return FC_STATUS_NOT_INITIALIZED;
    }

    critical_state = enter_critical();
    for (channel = 0U; channel < FC_IBUS_CHANNEL_COUNT; ++channel)
    {
        channels[channel] = s_raw_channels[channel];
    }
    has_valid_frame = s_stats.valid_frame_count > 0U;
    exit_critical(critical_state);
    return has_valid_frame ? FC_STATUS_OK : FC_STATUS_NOT_READY;
}

FcStatus_t Drv_Ibus_GetStats(DrvIbusStats_t *stats)
{
    uint32_t critical_state;

    if (stats == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (!s_initialized)
    {
        *stats = (DrvIbusStats_t){0};
        return FC_STATUS_NOT_INITIALIZED;
    }
    critical_state = enter_critical();
    *stats = s_stats;
    exit_critical(critical_state);
    return FC_STATUS_OK;
}

bool Drv_Ibus_IsOnline(void)
{
    FcRcInput_t input;
    return Drv_Ibus_GetInput(&input) == FC_STATUS_OK;
}
