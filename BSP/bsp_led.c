/* Keeps a requested LED mode but reports unavailable hardware honestly. */

#include "bsp_led.h"

static BspLedMode_t s_mode = BSP_LED_OFF;

FcStatus_t BSP_Led_Init(void)
{
    s_mode = BSP_LED_OFF;
    /* TODO(Phase 2, BSP): bind the board LED GPIO selected in CubeMX. */
    return FC_STATUS_NOT_READY;
}

FcStatus_t BSP_Led_SetMode(BspLedMode_t mode)
{
    if (mode > BSP_LED_BLINK_FAST)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    s_mode = mode;
    return FC_STATUS_NOT_READY;
}

void BSP_Led_1msTick(void)
{
    (void)s_mode;
}

bool BSP_Led_IsReady(void)
{
    return false;
}

