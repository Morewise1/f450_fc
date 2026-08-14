#ifndef BSP_SOFT_I2C_H
#define BSP_SOFT_I2C_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

typedef enum
{
    BSP_SOFT_I2C_BUS_BMP388 = 0,
    BSP_SOFT_I2C_BUS_MMC5983MA,
    BSP_SOFT_I2C_BUS_COUNT
} BspSoftI2cBus_t;

typedef struct
{
    uint32_t transaction_count;
    uint32_t recovery_count;
    uint32_t nack_count;
    uint32_t timeout_count;
    FcStatus_t last_status;
    bool initialized;
} BspSoftI2cDebug_t;

extern volatile BspSoftI2cDebug_t
    g_soft_i2c_debug[BSP_SOFT_I2C_BUS_COUNT];

FcStatus_t BSP_SoftI2c_Init(BspSoftI2cBus_t bus);
FcStatus_t BSP_SoftI2c_MemRead(BspSoftI2cBus_t bus,
                               uint8_t address_7bit,
                               uint8_t reg,
                               uint8_t *data,
                               uint16_t length);
FcStatus_t BSP_SoftI2c_MemWrite(BspSoftI2cBus_t bus,
                                uint8_t address_7bit,
                                uint8_t reg,
                                const uint8_t *data,
                                uint16_t length);

#endif /* BSP_SOFT_I2C_H */
