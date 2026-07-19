#ifndef DRV_BMP390_H
#define DRV_BMP390_H

/* BMP390 I2C data-provider contract for the later altitude phase. */

#include <stdint.h>
#include "fc_types.h"

FcStatus_t Drv_Bmp390_Init(void);
FcStatus_t Drv_Bmp390_Read(FcBarometerData_t *data, uint32_t timestamp_ms);

#endif /* DRV_BMP390_H */

