#ifndef BSP_BATTERY_ADC_H
#define BSP_BATTERY_ADC_H

/* 3S battery ADC acquisition and voltage-status interface. */

#include "fc_types.h"

FcStatus_t BSP_BatteryAdc_Init(void);
FcStatus_t BSP_BatteryAdc_Read(FcBatteryStatus_t *status, uint32_t timestamp_ms);

#endif /* BSP_BATTERY_ADC_H */

