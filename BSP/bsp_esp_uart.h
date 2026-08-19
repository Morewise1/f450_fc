#ifndef BSP_ESP_UART_H
#define BSP_ESP_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

#define BSP_ESP_UART_DEBUG_TX_SIZE        160U
#define BSP_ESP_UART_DEBUG_RX_RECENT_SIZE 128U

/* Keil Watch专用快照：RX是ESP8266发给STM32，TX是STM32发给ESP8266。 */
typedef struct
{
    uint8_t last_tx[BSP_ESP_UART_DEBUG_TX_SIZE];
    uint8_t recent_rx[BSP_ESP_UART_DEBUG_RX_RECENT_SIZE];
    uint16_t last_tx_length;
    uint16_t recent_rx_write_index;
    uint16_t recent_rx_valid_length;
    uint8_t last_rx_byte;
    uint32_t rx_byte_count;
    uint32_t tx_start_count;
    uint32_t tx_complete_count;
    uint32_t tx_busy_reject_count;
    uint32_t rx_overflow_count;
    uint32_t uart_error_count;
    FcStatus_t last_write_status;
    bool tx_busy;
} BspEspUartDebug_t;

extern volatile BspEspUartDebug_t g_esp_uart_debug;

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
