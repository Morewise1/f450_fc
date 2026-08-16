#include <stddef.h>
#include <string.h>
#include "bsp_esp_uart.h"
#include "fc_config.h"

#define ESP_UART_RX_RING_SIZE 256U
#define ESP_UART_TX_SIZE      160U

static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile bool s_tx_busy;
static volatile uint32_t s_rx_overflows;
static volatile uint32_t s_errors;
static uint8_t s_rx_ring[ESP_UART_RX_RING_SIZE];
#if FC_USE_STM32_HAL
static uint8_t s_rx_byte;
#endif
static uint8_t s_tx_buffer[ESP_UART_TX_SIZE];
static void *s_uart;

#if FC_USE_STM32_HAL
#include "main.h"
#endif

FcStatus_t BSP_EspUart_Init(void *uart_handle)
{
    if (uart_handle == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    s_uart = uart_handle;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_tx_busy = false;
    s_rx_overflows = 0U;
    s_errors = 0U;
#if FC_USE_STM32_HAL
    return (HAL_UART_Receive_IT((UART_HandleTypeDef *)s_uart,
                                &s_rx_byte,
                                1U) == HAL_OK) ?
           FC_STATUS_OK : FC_STATUS_ERROR;
#else
    return FC_STATUS_NOT_IMPLEMENTED;
#endif
}

FcStatus_t BSP_EspUart_WriteAsync(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if ((length > ESP_UART_TX_SIZE) || (s_uart == NULL))
    {
        return FC_STATUS_INVALID_DATA;
    }
    if (s_tx_busy) { return FC_STATUS_BUSY; }
    (void)memcpy(s_tx_buffer, data, length);
    s_tx_busy = true;
#if FC_USE_STM32_HAL
    if (HAL_UART_Transmit_IT((UART_HandleTypeDef *)s_uart,
                             s_tx_buffer,
                             length) != HAL_OK)
    {
        s_tx_busy = false;
        return FC_STATUS_ERROR;
    }
    return FC_STATUS_OK;
#else
    s_tx_busy = false;
    return FC_STATUS_NOT_IMPLEMENTED;
#endif
}

bool BSP_EspUart_ReadByte(uint8_t *value)
{
    if ((value == NULL) || (s_rx_tail == s_rx_head)) { return false; }
    *value = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % ESP_UART_RX_RING_SIZE);
    return true;
}

bool BSP_EspUart_TxReady(void)
{
    return (s_uart != NULL) && !s_tx_busy;
}

void BSP_EspUart_OnRxComplete(void *uart_handle)
{
#if FC_USE_STM32_HAL
    uint16_t next;
    if ((uart_handle == NULL) || (uart_handle != s_uart)) { return; }
    next = (uint16_t)((s_rx_head + 1U) % ESP_UART_RX_RING_SIZE);
    if (next != s_rx_tail)
    {
        s_rx_ring[s_rx_head] = s_rx_byte;
        s_rx_head = next;
    }
    else if (s_rx_overflows != UINT32_MAX)
    {
        ++s_rx_overflows;
    }
    if (HAL_UART_Receive_IT((UART_HandleTypeDef *)s_uart,
                            &s_rx_byte,
                            1U) != HAL_OK)
    {
        if (s_errors != UINT32_MAX) { ++s_errors; }
    }
#else
    (void)uart_handle;
#endif
}

void BSP_EspUart_OnTxComplete(void *uart_handle)
{
    if (uart_handle == s_uart) { s_tx_busy = false; }
}

void BSP_EspUart_OnError(void *uart_handle)
{
    if (uart_handle != s_uart) { return; }
    if (s_errors != UINT32_MAX) { ++s_errors; }
#if FC_USE_STM32_HAL
    /* Do not release a still-active TX buffer for an unrelated RX error. */
    if (((UART_HandleTypeDef *)s_uart)->gState == HAL_UART_STATE_READY)
    {
        s_tx_busy = false;
    }
    if (((UART_HandleTypeDef *)s_uart)->RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_IT((UART_HandleTypeDef *)s_uart,
                                 &s_rx_byte,
                                 1U);
    }
#else
    s_tx_busy = false;
#endif
}

uint32_t BSP_EspUart_GetRxOverflowCount(void) { return s_rx_overflows; }
uint32_t BSP_EspUart_GetErrorCount(void) { return s_errors; }
