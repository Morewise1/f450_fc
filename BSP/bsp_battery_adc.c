/* Returns an explicitly invalid battery status until ADC/divider binding exists. */

#include <stddef.h>
#include "bsp_battery_adc.h"

FcStatus_t BSP_BatteryAdc_Init(void)
{
    /* TODO(Phase 2, BSP): bind ADC channel and measured divider calibration. */
    return FC_STATUS_NOT_READY;
}

FcStatus_t BSP_BatteryAdc_Read(FcBatteryStatus_t *status, uint32_t timestamp_ms)
{
    if (status == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    *status = (FcBatteryStatus_t){0};
    status->timestamp_ms = timestamp_ms;
    status->valid = false;
    status->critical = true;
    return FC_STATUS_NOT_READY;
}

