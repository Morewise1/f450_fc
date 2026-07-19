#ifndef DRV_VL53L1X_H
#define DRV_VL53L1X_H

/* VL53L1X low-altitude range contract; intentionally inactive in version 1. */

#include <stdint.h>
#include "fc_types.h"

FcStatus_t Drv_Vl53l1x_Init(void);
FcStatus_t Drv_Vl53l1x_Read(FcRangeData_t *data, uint32_t timestamp_ms);

#endif /* DRV_VL53L1X_H */

