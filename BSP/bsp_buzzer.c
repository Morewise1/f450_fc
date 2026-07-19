/* Stores buzzer intent without claiming an unbound GPIO/timer is operational. */

#include "bsp_buzzer.h"

static BspBuzzerPattern_t s_pattern = BSP_BUZZER_SILENT;

FcStatus_t BSP_Buzzer_Init(void)
{
    s_pattern = BSP_BUZZER_SILENT;
    /* TODO(Phase 2, BSP): bind buzzer GPIO or timer output. */
    return FC_STATUS_NOT_READY;
}

FcStatus_t BSP_Buzzer_SetPattern(BspBuzzerPattern_t pattern)
{
    if (pattern > BSP_BUZZER_CRITICAL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    s_pattern = pattern;
    return FC_STATUS_NOT_READY;
}

void BSP_Buzzer_1msTick(void)
{
    (void)s_pattern;
}

bool BSP_Buzzer_IsReady(void)
{
    return false;
}

