#ifndef APP_SAFETY_H
#define APP_SAFETY_H

/* Conservative arming, failsafe, and motor-output permission decisions. */

#include <stdbool.h>
#include "fc_types.h"

FcStatus_t App_SafetyInit(void);
void App_SafetySetInitializationResult(bool initialization_ok);
void App_SafetyEvaluate(const FcRcInput_t *rc,
                        const FcImuData_t *imu,
                        const FcAttitude_t *attitude,
                        const FcBatteryStatus_t *battery,
                        bool scheduler_ok);
bool App_SafetyArmConditionsMet(void);
bool App_SafetyMotorOutputAllowed(void);
FcStatus_t App_SafetyGetStatus(FcSafetyStatus_t *status);

#endif /* APP_SAFETY_H */
