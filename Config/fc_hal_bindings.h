#ifndef FC_HAL_BINDINGS_H
#define FC_HAL_BINDINGS_H

/* Optional CubeMX/HAL bindings. Modules never create HAL handles themselves. */

#include "fc_config.h"

#if FC_USE_STM32_HAL
#include "main.h"

/* ESC timer handles/channels are mapped per motor in fc_board.h. */
#ifndef FC_HAL_IBUS_UART_HANDLE
#error "Define FC_HAL_IBUS_UART_HANDLE as a CubeMX UART handle"
#endif
#ifndef FC_HAL_DEBUG_UART_HANDLE
#error "Define FC_HAL_DEBUG_UART_HANDLE as a CubeMX UART handle"
#endif
#ifndef FC_HAL_SENSOR_I2C_HANDLE
#error "Define FC_HAL_SENSOR_I2C_HANDLE as a CubeMX I2C handle"
#endif
#ifndef FC_HAL_BATTERY_ADC_HANDLE
#error "Define FC_HAL_BATTERY_ADC_HANDLE as a CubeMX ADC handle"
#endif

extern UART_HandleTypeDef FC_HAL_IBUS_UART_HANDLE;
extern UART_HandleTypeDef FC_HAL_DEBUG_UART_HANDLE;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef FC_HAL_SENSOR_I2C_HANDLE;
extern ADC_HandleTypeDef FC_HAL_BATTERY_ADC_HANDLE;

#define FC_HAL_BMI088_SPI_HANDLE hspi1
#endif

#endif /* FC_HAL_BINDINGS_H */

