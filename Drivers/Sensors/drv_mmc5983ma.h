#ifndef DRV_MMC5983MA_H
#define DRV_MMC5983MA_H

/* MMC5983MA I2C driver. Output is uncalibrated sensor-frame magnetic field. */

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

typedef struct
{
    uint8_t product_id;
    FcStatus_t init_status;
    FcStatus_t last_read_status;
    uint32_t valid_read_count;
    uint32_t failed_read_count;
    uint32_t saturated_read_count;
    FcVector3f_t calibration_offset_ut;
    FcVector3f_t calibration_scale;
    uint32_t calibration_sample_count;
    bool calibration_active;
    bool calibration_valid;
    bool ready;
} DrvMmc5983maDebug_t;

extern volatile DrvMmc5983maDebug_t g_mmc5983ma_debug;

FcStatus_t Drv_Mmc5983ma_Init(void);
FcStatus_t Drv_Mmc5983ma_Read(FcMagnetometerData_t *data, uint32_t timestamp_ms);
FcStatus_t Drv_Mmc5983ma_PerformSet(void);
FcStatus_t Drv_Mmc5983ma_StartCalibration(void);
FcStatus_t Drv_Mmc5983ma_FinishCalibration(void);
bool Drv_Mmc5983ma_IsReady(void);
FcStatus_t Drv_Mmc5983ma_GetDebug(DrvMmc5983maDebug_t *debug);

#endif /* DRV_MMC5983MA_H */
