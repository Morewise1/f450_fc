#ifndef BSP_LED_H
#define BSP_LED_H

/* Board status LED stub. */

#include <stdbool.h>
#include "fc_types.h"

typedef enum
{
    BSP_LED_OFF = 0,
    BSP_LED_ON,
    BSP_LED_BLINK_SLOW,
    BSP_LED_BLINK_FAST
} BspLedMode_t;

FcStatus_t BSP_Led_Init(void);
FcStatus_t BSP_Led_SetMode(BspLedMode_t mode);
void BSP_Led_1msTick(void);
bool BSP_Led_IsReady(void);

#endif /* BSP_LED_H */

