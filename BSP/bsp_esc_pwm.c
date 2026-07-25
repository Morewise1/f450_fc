/* STM32 HAL ESC PWM output with per-channel us-to-CCR conversion. */

#include <stddef.h>
#include "bsp_esc_pwm.h"
#include "fc_board.h"
#include "fc_config.h"

#if FC_USE_STM32_HAL
typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint32_t counter_hz;
} EscPwmChannel_t;

static const EscPwmChannel_t s_channels[FC_MOTOR_COUNT] = {
    {&FC_ESC_M1_TIM_HANDLE, FC_ESC_M1_TIM_CHANNEL, FC_ESC_M1_COUNTER_HZ},
    {&FC_ESC_M2_TIM_HANDLE, FC_ESC_M2_TIM_CHANNEL, FC_ESC_M2_COUNTER_HZ},
    {&FC_ESC_M3_TIM_HANDLE, FC_ESC_M3_TIM_CHANNEL, FC_ESC_M3_COUNTER_HZ},
    {&FC_ESC_M4_TIM_HANDLE, FC_ESC_M4_TIM_CHANNEL, FC_ESC_M4_COUNTER_HZ}
};
#endif

static FcMotorOutput_t s_last_command = {
    {FC_ESC_STOP_US, FC_ESC_STOP_US, FC_ESC_STOP_US, FC_ESC_STOP_US},
    true
};
static bool s_initialized;
static bool s_output_enabled;

uint16_t BSP_EscPwm_ClampUs(uint16_t pulse_us)
{
    if (pulse_us < FC_ESC_MIN_US) { return FC_ESC_MIN_US; }
    if (pulse_us > FC_ESC_COMMAND_MAX_US) { return FC_ESC_COMMAND_MAX_US; }
    return pulse_us;
}

static void set_shadow_stop(void)
{
    uint32_t motor;

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        s_last_command.motor_us[motor] = FC_ESC_STOP_US;
    }
    s_last_command.valid = true;
}

#if FC_USE_STM32_HAL
/* CCR = round(pulse_us * timer_counter_hz / 1,000,000). */
static uint32_t pulse_us_to_ccr(uint16_t pulse_us, uint32_t counter_hz)
{
    uint64_t numerator = ((uint64_t)pulse_us * (uint64_t)counter_hz) + 500000ULL;
    return (uint32_t)(numerator / 1000000ULL);
}

static void write_compare(uint8_t motor_id, uint16_t pulse_us)
{
    const EscPwmChannel_t *output = &s_channels[motor_id];

    __HAL_TIM_SET_COMPARE(output->timer,
                          output->channel,
                          pulse_us_to_ccr(pulse_us, output->counter_hz));
}

static void write_stop_to_all_channels(void)
{
    uint32_t motor;

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        write_compare((uint8_t)motor, FC_ESC_STOP_US);
    }
}

static bool channel_period_can_fit_pulse(const EscPwmChannel_t *output)
{
    uint64_t period_ticks;
    uint64_t maximum_ccr;

    if ((output->counter_hz == 0U) || (FC_ESC_PWM_FRAME_HZ == 0U))
    {
        return false;
    }

    period_ticks = (uint64_t)output->counter_hz / (uint64_t)FC_ESC_PWM_FRAME_HZ;
    maximum_ccr = ((uint64_t)FC_ESC_COMMAND_MAX_US * (uint64_t)output->counter_hz + 999999ULL) /
                  1000000ULL;
    return (maximum_ccr > 0U) && (period_ticks > maximum_ccr);
}

static bool channel_mapping_is_valid(void)
{
    uint32_t motor;
    uint32_t other;

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        if ((s_channels[motor].timer == NULL) || !channel_period_can_fit_pulse(&s_channels[motor]))
        {
            return false;
        }
        for (other = motor + 1U; other < FC_MOTOR_COUNT; ++other)
        {
            if ((s_channels[motor].timer == s_channels[other].timer) &&
                (s_channels[motor].channel == s_channels[other].channel))
            {
                return false;
            }
        }
    }
    return true;
}

static void stop_started_channels(uint32_t started_count)
{
    while (started_count > 0U)
    {
        --started_count;
        (void)HAL_TIM_PWM_Stop(s_channels[started_count].timer,
                               s_channels[started_count].channel);
    }
}
#endif

FcStatus_t BSP_EscPwm_Init(void)
{
    s_initialized = false;
    s_output_enabled = false;
    set_shadow_stop();

#if !FC_USE_STM32_HAL
    return FC_STATUS_NOT_READY;
#else
    uint32_t motor;

    if (!channel_mapping_is_valid())
    {
        return FC_STATUS_INVALID_DATA;
    }

    /* Program 1000 us before enabling any PWM output pin. */
    write_stop_to_all_channels();

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        if (HAL_TIM_PWM_Start(s_channels[motor].timer, s_channels[motor].channel) != HAL_OK)
        {
            write_stop_to_all_channels();
            stop_started_channels(motor);
            return FC_STATUS_ERROR;
        }
    }

    s_initialized = true;
    return FC_STATUS_OK;
#endif
}

FcStatus_t BSP_EscPwm_WriteUs(uint8_t motor_id, uint16_t pulse_us)
{
    uint16_t limited_us;

    if (motor_id >= FC_MOTOR_COUNT)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    limited_us = BSP_EscPwm_ClampUs(pulse_us);
    if (!s_initialized)
    {
        set_shadow_stop();
        return FC_STATUS_NOT_READY;
    }
    if (!s_output_enabled && (limited_us != FC_ESC_STOP_US))
    {
#if FC_USE_STM32_HAL
        write_compare(motor_id, FC_ESC_STOP_US);
#endif
        s_last_command.motor_us[motor_id] = FC_ESC_STOP_US;
        return FC_STATUS_NOT_READY;
    }

#if FC_USE_STM32_HAL
    write_compare(motor_id, limited_us);
#endif
    s_last_command.motor_us[motor_id] = limited_us;
    s_last_command.valid = true;
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_WriteAll(const FcMotorOutput_t *out)
{
    uint16_t limited[FC_MOTOR_COUNT];
    uint32_t motor;

    if ((out == NULL) || !out->valid)
    {
        (void)BSP_EscPwm_StopAll();
        return FC_STATUS_INVALID_ARGUMENT;
    }

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        limited[motor] = BSP_EscPwm_ClampUs(out->motor_us[motor]);
    }

    if (!s_initialized)
    {
        set_shadow_stop();
        return FC_STATUS_NOT_READY;
    }
    if (!s_output_enabled)
    {
        for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
        {
            if (limited[motor] != FC_ESC_STOP_US)
            {
                (void)BSP_EscPwm_StopAll();
                return FC_STATUS_NOT_READY;
            }
        }
    }

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
#if FC_USE_STM32_HAL
        write_compare((uint8_t)motor, limited[motor]);
#endif
        s_last_command.motor_us[motor] = limited[motor];
    }
    s_last_command.valid = true;
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_WriteTestUs(uint8_t motor_id, uint16_t pulse_us)
{
#if !FC_ENABLE_MOTOR_TEST
    (void)motor_id;
    (void)pulse_us;
    return FC_STATUS_NOT_READY;
#else
    if (motor_id >= FC_MOTOR_COUNT)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (pulse_us > FC_MOTOR_TEST_MAX_US)
    {
        pulse_us = FC_MOTOR_TEST_MAX_US;
    }
    return BSP_EscPwm_WriteUs(motor_id, pulse_us);
#endif
}

FcStatus_t BSP_EscPwm_StopAll(void)
{
    s_output_enabled = false;
    set_shadow_stop();

    if (!s_initialized)
    {
        return FC_STATUS_NOT_READY;
    }

#if FC_USE_STM32_HAL
    write_stop_to_all_channels();
#endif
    return FC_STATUS_OK;
}

FcStatus_t BSP_EscPwm_SetOutputEnabled(bool enabled)
{
    if (!enabled)
    {
        return BSP_EscPwm_StopAll();
    }
    if (!s_initialized)
    {
        s_output_enabled = false;
        return FC_STATUS_NOT_READY;
    }

    s_output_enabled = true;
    return FC_STATUS_OK;
}

bool BSP_EscPwm_IsOutputEnabled(void)
{
    return s_initialized && s_output_enabled;
}

bool BSP_EscPwm_IsInitialized(void)
{
    return s_initialized;
}

FcStatus_t BSP_EscPwm_GetLastCommand(FcMotorOutput_t *out)
{
    if (out == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    *out = s_last_command;
    return FC_STATUS_OK;
}
