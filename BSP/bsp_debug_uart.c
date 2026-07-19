/* Fails clearly until an externally owned CubeMX UART handle is bound. */

#include "bsp_debug_uart.h"

FcStatus_t BSP_DebugUart_Init(void)
{
    /* TODO(Phase 2, BSP): bind a CubeMX UART and add a non-blocking TX queue. */
    return FC_STATUS_NOT_READY;
}

FcStatus_t BSP_DebugUart_Write(const uint8_t *data, size_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    return FC_STATUS_NOT_READY;
}

bool BSP_DebugUart_IsReady(void)
{
    return false;
}
