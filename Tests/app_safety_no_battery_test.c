#include "app_safety.h"
#include "fc_config.h"

int main(void)
{
    FcRcInput_t rc = {0};
    FcImuData_t imu = {0};
    FcAttitude_t attitude = {0};
    FcBatteryStatus_t unavailable_battery = {0};
    FcSafetyStatus_t status = {0};

#if FC_ENABLE_BATTERY_MONITOR
    return 1;
#endif

    rc.link_valid = true;
    rc.failsafe = false;
    rc.throttle = 0U;
    rc.throttle_low = true;
    rc.arm_switch = true;
    rc.safety_switch = true;

    imu.valid = true;
    imu.calibrated = true;
    attitude.valid = true;

    unavailable_battery.valid = false;
    unavailable_battery.critical = true;

    if (App_SafetyInit() != FC_STATUS_OK) { return 2; }
    App_SafetySetInitializationResult(true);
    App_SafetyEvaluate(&rc, &imu, &attitude, &unavailable_battery, true);
    if (App_SafetyGetStatus(&status) != FC_STATUS_OK) { return 3; }
    if (!status.battery_ok) { return 4; }
    if ((status.active_faults & (FC_SAFETY_FAULT_BATTERY_UNKNOWN |
                                 FC_SAFETY_FAULT_BATTERY_CRITICAL)) != 0U) { return 5; }
    if ((status.active_faults != FC_SAFETY_FAULT_NONE) ||
        !status.arm_conditions_met || !status.motor_output_allowed) { return 6; }
    return 0;
}
