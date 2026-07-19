/* Safe first-version stub: no pressure data is marked valid. */

#include <stddef.h>
#include "drv_bmp390.h"

FcStatus_t Drv_Bmp390_Init(void)
{
    /* TODO(Phase 6, altitude): bind extern I2C handle and Bosch compensation. */
    return FC_STATUS_NOT_IMPLEMENTED;
}

FcStatus_t Drv_Bmp390_Read(FcBarometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *data = (FcBarometerData_t){0};
    data->timestamp_ms = timestamp_ms;
    data->valid = false;
    return FC_STATUS_NOT_IMPLEMENTED;
}

