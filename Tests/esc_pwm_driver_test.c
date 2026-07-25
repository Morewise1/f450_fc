/* Host test for four ESC channels using two timers and four counter rates. */

#include <stdbool.h>
#include <stdint.h>
#include "bsp_esc_pwm.h"
#include "fc_board.h"
#include "fc_config.h"
#include "main.h"

TIM_HandleTypeDef htim3 = {3U};
TIM_HandleTypeDef htim4 = {4U};

static uint32_t s_compare[FC_MOTOR_COUNT];
static bool s_started[FC_MOTOR_COUNT];
static uint32_t s_compare_write_count;
static uint32_t s_start_attempt_count;
static uint32_t s_stop_count;
static int32_t s_fail_start_motor = -1;

static int32_t motor_from_channel(TIM_HandleTypeDef *handle, uint32_t channel)
{
    if ((handle == &htim3) && (channel == TIM_CHANNEL_1)) { return FC_MOTOR_INDEX_M1; }
    if ((handle == &htim3) && (channel == TIM_CHANNEL_2)) { return FC_MOTOR_INDEX_M2; }
    if ((handle == &htim4) && (channel == TIM_CHANNEL_1)) { return FC_MOTOR_INDEX_M3; }
    if ((handle == &htim4) && (channel == TIM_CHANNEL_2)) { return FC_MOTOR_INDEX_M4; }
    return -1;
}

static uint32_t pulse_to_ticks(uint16_t pulse_us, uint32_t counter_hz)
{
    return (uint32_t)((((uint64_t)pulse_us * counter_hz) + 500000ULL) / 1000000ULL);
}

static uint32_t motor_counter_hz(uint32_t motor)
{
    const uint32_t counters[FC_MOTOR_COUNT] = {
        FC_ESC_M1_COUNTER_HZ,
        FC_ESC_M2_COUNTER_HZ,
        FC_ESC_M3_COUNTER_HZ,
        FC_ESC_M4_COUNTER_HZ
    };
    return counters[motor];
}

static bool all_channels_stopped(void)
{
    uint32_t motor;

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        if (s_compare[motor] != pulse_to_ticks(FC_ESC_STOP_US, motor_counter_hz(motor)))
        {
            return false;
        }
    }
    return true;
}

static bool no_channels_started(void)
{
    uint32_t motor;

    for (motor = 0U; motor < FC_MOTOR_COUNT; ++motor)
    {
        if (s_started[motor]) { return false; }
    }
    return true;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel)
{
    int32_t motor = motor_from_channel(handle, channel);

    ++s_start_attempt_count;
    if (motor < 0) { return HAL_ERROR; }
    if (motor == s_fail_start_motor) { return HAL_ERROR; }
    s_started[motor] = true;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *handle, uint32_t channel)
{
    int32_t motor = motor_from_channel(handle, channel);

    if (motor < 0) { return HAL_ERROR; }
    s_started[motor] = false;
    ++s_stop_count;
    return HAL_OK;
}

void FakeHalTimSetCompare(TIM_HandleTypeDef *handle, uint32_t channel, uint32_t compare)
{
    int32_t motor = motor_from_channel(handle, channel);

    if (motor >= 0)
    {
        s_compare[motor] = compare;
        ++s_compare_write_count;
    }
}

int main(void)
{
    FcMotorOutput_t output = {{999U, FC_ESC_IDLE_US, 1500U, 2500U}, true};
    FcMotorOutput_t invalid_output = {{1500U, 1500U, 1500U, 1500U}, false};
    FcMotorOutput_t last;
    uint32_t writes_before_invalid;

    if (BSP_EscPwm_ClampUs(500U) != 1000U) { return 1; }
    if (BSP_EscPwm_ClampUs(1500U) != 1500U) { return 2; }
    if (BSP_EscPwm_ClampUs(2500U) != FC_ESC_COMMAND_MAX_US) { return 3; }
    if (BSP_EscPwm_WriteTestUs(FC_MOTOR_INDEX_M1, 1100U) != FC_STATUS_NOT_READY) { return 35; }

    /* A partial start failure must stop every channel that already started. */
    s_fail_start_motor = FC_MOTOR_INDEX_M3;
    if (BSP_EscPwm_Init() != FC_STATUS_ERROR) { return 4; }
    if (BSP_EscPwm_IsInitialized() || BSP_EscPwm_IsOutputEnabled()) { return 5; }
    if (!all_channels_stopped() || !no_channels_started()) { return 6; }
    if ((s_start_attempt_count != 3U) || (s_stop_count != 2U)) { return 7; }

    s_fail_start_motor = -1;
    s_start_attempt_count = 0U;
    s_stop_count = 0U;
    if (BSP_EscPwm_Init() != FC_STATUS_OK) { return 8; }
    if (!BSP_EscPwm_IsInitialized() || BSP_EscPwm_IsOutputEnabled()) { return 9; }
    if ((s_start_attempt_count != FC_MOTOR_COUNT) || !all_channels_stopped()) { return 10; }

    if (BSP_EscPwm_WriteUs(FC_MOTOR_INDEX_M1, 1500U) != FC_STATUS_NOT_READY) { return 11; }
    if (s_compare[FC_MOTOR_INDEX_M1] != pulse_to_ticks(1000U, FC_ESC_M1_COUNTER_HZ)) { return 12; }

    writes_before_invalid = s_compare_write_count;
    if (BSP_EscPwm_WriteUs(FC_MOTOR_COUNT, 1500U) != FC_STATUS_INVALID_ARGUMENT) { return 13; }
    if (s_compare_write_count != writes_before_invalid) { return 14; }

    if (BSP_EscPwm_WriteAll(&output) != FC_STATUS_NOT_READY) { return 15; }
    if (!all_channels_stopped()) { return 16; }

    if (BSP_EscPwm_SetOutputEnabled(true) != FC_STATUS_OK) { return 17; }
    if (!BSP_EscPwm_IsOutputEnabled()) { return 18; }
    if (BSP_EscPwm_WriteAll(&output) != FC_STATUS_OK) { return 19; }
    if (s_compare[0] != pulse_to_ticks(1000U, FC_ESC_M1_COUNTER_HZ)) { return 20; }
    if (s_compare[1] != pulse_to_ticks(FC_ESC_IDLE_US, FC_ESC_M2_COUNTER_HZ)) { return 21; }
    if (s_compare[2] != pulse_to_ticks(1500U, FC_ESC_M3_COUNTER_HZ)) { return 22; }
    if (s_compare[3] != pulse_to_ticks(FC_ESC_COMMAND_MAX_US, FC_ESC_M4_COUNTER_HZ)) { return 23; }

    if (BSP_EscPwm_GetLastCommand(&last) != FC_STATUS_OK) { return 24; }
    if ((last.motor_us[0] != 1000U) ||
        (last.motor_us[1] != FC_ESC_IDLE_US) ||
        (last.motor_us[2] != 1500U) ||
        (last.motor_us[3] != FC_ESC_COMMAND_MAX_US) || !last.valid) { return 25; }

    if (BSP_EscPwm_WriteAll(&invalid_output) != FC_STATUS_INVALID_ARGUMENT) { return 26; }
    if (BSP_EscPwm_IsOutputEnabled() || !all_channels_stopped()) { return 27; }

    if (BSP_EscPwm_SetOutputEnabled(true) != FC_STATUS_OK) { return 28; }
    if (BSP_EscPwm_WriteAll(NULL) != FC_STATUS_INVALID_ARGUMENT) { return 29; }
    if (BSP_EscPwm_IsOutputEnabled() || !all_channels_stopped()) { return 30; }

    if (BSP_EscPwm_SetOutputEnabled(true) != FC_STATUS_OK) { return 31; }
    if (BSP_EscPwm_WriteUs(FC_MOTOR_INDEX_M2, 2000U) != FC_STATUS_OK) { return 32; }
    if (BSP_EscPwm_StopAll() != FC_STATUS_OK) { return 33; }
    if (BSP_EscPwm_IsOutputEnabled() || !all_channels_stopped()) { return 34; }
    return 0;
}
