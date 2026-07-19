#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

/* Non-blocking buzzer pattern stub. */

#include <stdbool.h>
#include "fc_types.h"

typedef enum
{
    BSP_BUZZER_SILENT = 0,
    BSP_BUZZER_ARMED,
    BSP_BUZZER_WARNING,
    BSP_BUZZER_CRITICAL
} BspBuzzerPattern_t;

FcStatus_t BSP_Buzzer_Init(void);
FcStatus_t BSP_Buzzer_SetPattern(BspBuzzerPattern_t pattern);
void BSP_Buzzer_1msTick(void);
bool BSP_Buzzer_IsReady(void);

#endif /* BSP_BUZZER_H */

