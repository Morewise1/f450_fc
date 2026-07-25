#ifndef DRV_BMI088_H
#define DRV_BMI088_H

/* BMI088 SPI driver. Output units are g, deg/s, and degrees Celsius. */

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

typedef struct
{
    uint8_t accel;
    uint8_t gyro;
} Bmi088ChipIds_t;

/* Initialization contains blocking delays; the 500 Hz read path does not. */
FcStatus_t Drv_Bmi088_Init(void);

/* A failed or in-progress read always returns imu->valid == false. */
FcStatus_t Drv_Bmi088_Read(FcImuData_t *imu);

/* Starts non-blocking RAM-only gyro calibration; Read() collects samples. */
FcStatus_t Drv_Bmi088_CalibrateGyro(void);
FcStatus_t Drv_Bmi088_SetBiasTrackingEnabled(bool enabled);
bool Drv_Bmi088_IsReady(void);

FcStatus_t Drv_Bmi088_ReadChipIds(Bmi088ChipIds_t *ids);
FcStatus_t Drv_Bmi088_GetChipIds(Bmi088ChipIds_t *ids);
bool Drv_Bmi088_IsCalibrationComplete(void);
bool Drv_Bmi088_IsDataValid(uint32_t now_ms);
FcStatus_t Drv_Bmi088_GetGyroBias(FcVector3f_t *bias_dps);

#endif /* DRV_BMI088_H */
