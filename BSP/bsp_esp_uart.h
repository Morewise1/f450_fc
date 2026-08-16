#ifndef BSP_ESP_UART_H
#define BSP_ESP_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

FcStatus_t BSP_EspUart_Init(void *uart_handle);
FcStatus_t BSP_EspUart_WriteAsync(const uint8_t *data, uint16_t length);
bool BSP_EspUart_ReadByte(uint8_t *value);
bool BSP_EspUart_TxReady(void);
void BSP_EspUart_OnRxComplete(void *uart_handle);
void BSP_EspUart_OnTxComplete(void *uart_handle);
void BSP_EspUart_OnError(void *uart_handle);
uint32_t BSP_EspUart_GetRxOverflowCount(void);
uint32_t BSP_EspUart_GetErrorCount(void);

#endif /* BSP_ESP_UART_H */
