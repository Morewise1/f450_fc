#ifndef BSP_DEBUG_UART_H
#define BSP_DEBUG_UART_H

/* Future non-blocking debug UART interface; never called from an ISR. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fc_types.h"

FcStatus_t BSP_DebugUart_Init(void);
FcStatus_t BSP_DebugUart_Write(const uint8_t *data, size_t length);
bool BSP_DebugUart_IsReady(void);

#endif /* BSP_DEBUG_UART_H */

