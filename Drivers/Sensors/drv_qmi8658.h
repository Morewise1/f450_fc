#ifndef DRV_QMI8658_H
#define DRV_QMI8658_H

/* QMI8658C SPI driver. Output units are g, deg/s, and degrees Celsius. */

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

/* Initialization performs the only blocking reset delay used by this driver. */
FcStatus_t Drv_Qmi8658_Init(void);

/* A failed or in-progress read always returns imu->valid == false. */
FcStatus_t Drv_Qmi8658_Read(FcImuData_t *imu);

/* Starts non-blocking RAM-only gyro calibration; Read() collects samples. */
FcStatus_t Drv_Qmi8658_CalibrateGyro(void);
bool Drv_Qmi8658_IsReady(void);

FcStatus_t Drv_Qmi8658_ReadWhoAmI(uint8_t *who_am_i);
uint8_t Drv_Qmi8658_GetWhoAmI(void);
bool Drv_Qmi8658_IsCalibrationComplete(void);
bool Drv_Qmi8658_IsDataValid(uint32_t now_ms);
FcStatus_t Drv_Qmi8658_GetGyroBias(FcVector3f_t *bias_dps);

#endif /* DRV_QMI8658_H */
