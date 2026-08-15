#ifndef DRV_BMP388_H
#define DRV_BMP388_H

/* BMP388 I2C driver. Output units are pascals and degrees Celsius. */

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

typedef struct
{
    uint8_t address_7bit;
    uint8_t chip_id;
    uint8_t status_register;
    uint8_t error_register;
    FcStatus_t init_status;
    FcStatus_t last_read_status;
    uint32_t valid_read_count;
    uint32_t failed_read_count;
    bool ready;
} DrvBmp388Debug_t;

extern volatile DrvBmp388Debug_t g_bmp388_debug;

/* Initialization contains short blocking reset delays; Read() does not. */
FcStatus_t Drv_Bmp388_Init(void);
FcStatus_t Drv_Bmp388_Read(FcBarometerData_t *data, uint32_t timestamp_ms);
bool Drv_Bmp388_IsReady(void);
FcStatus_t Drv_Bmp388_GetDebug(DrvBmp388Debug_t *debug);

#endif /* DRV_BMP388_H */
