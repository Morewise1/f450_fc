/* Safe placeholder for a later low-altitude hold phase. */

#include <stddef.h>
#include "drv_vl53l1x.h"

FcStatus_t Drv_Vl53l1x_Init(void)
{
    return FC_STATUS_NOT_IMPLEMENTED;
}

FcStatus_t Drv_Vl53l1x_Read(FcRangeData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *data = (FcRangeData_t){0};
    data->timestamp_ms = timestamp_ms;
    data->valid = false;
    return FC_STATUS_NOT_IMPLEMENTED;
}

