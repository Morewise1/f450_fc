#ifndef BSP_ESC_PWM_H
#define BSP_ESC_PWM_H

/* Four-channel ESC PWM output. Every public pulse value is in microseconds. */

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

FcStatus_t BSP_EscPwm_Init(void);
FcStatus_t BSP_EscPwm_WriteUs(uint8_t motor_id, uint16_t pulse_us);
FcStatus_t BSP_EscPwm_WriteAll(const FcMotorOutput_t *out);
FcStatus_t BSP_EscPwm_WriteTestUs(uint8_t motor_id, uint16_t pulse_us);
FcStatus_t BSP_EscPwm_StopAll(void);
uint16_t BSP_EscPwm_ClampUs(uint16_t pulse_us);

/* App safety grants permission; this BSP never reads flight-state enums. */
FcStatus_t BSP_EscPwm_SetOutputEnabled(bool enabled);
bool BSP_EscPwm_IsOutputEnabled(void);
bool BSP_EscPwm_IsInitialized(void);
FcStatus_t BSP_EscPwm_GetLastCommand(FcMotorOutput_t *out);

#endif /* BSP_ESC_PWM_H */
