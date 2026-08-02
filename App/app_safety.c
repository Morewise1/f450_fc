/* Unknown, stale, inconsistent, or out-of-range data is always unsafe. */

#include <stddef.h>
#include "app_safety.h"
#include "fc_config.h"

static FcSafetyStatus_t s_status;
static bool s_initialized;

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool rc_input_is_sane(const FcRcInput_t *rc)
{
    return (rc != NULL) &&
           rc->link_valid && !rc->failsafe &&
           (rc->roll >= FC_RC_AXIS_MIN) && (rc->roll <= FC_RC_AXIS_MAX) &&
           (rc->pitch >= FC_RC_AXIS_MIN) && (rc->pitch <= FC_RC_AXIS_MAX) &&
           (rc->yaw >= FC_RC_AXIS_MIN) && (rc->yaw <= FC_RC_AXIS_MAX) &&
           (rc->throttle <= FC_RC_THROTTLE_MAX);
}

static void set_fail_closed_defaults(void)
{
    s_status = (FcSafetyStatus_t){0};
    s_status.active_faults = FC_SAFETY_FAULT_RC_LOST |
                             FC_SAFETY_FAULT_IMU_INVALID |
                             FC_SAFETY_FAULT_INITIALIZATION;
#if FC_ENABLE_BATTERY_MONITOR
    s_status.active_faults |= FC_SAFETY_FAULT_BATTERY_UNKNOWN;
#else
    s_status.battery_ok = true;
#endif
    s_status.arm_conditions_met = false;
    s_status.motor_output_allowed = false;
}

FcStatus_t App_SafetyInit(void)
{
    s_initialized = false;
    set_fail_closed_defaults();
    s_initialized = true;
    return FC_STATUS_OK;
}

void App_SafetySetInitializationResult(bool initialization_ok)
{
    if (!s_initialized)
    {
        return;
    }

    s_status.initialization_ok = initialization_ok;
    if (initialization_ok)
    {
        s_status.active_faults &= ~((uint32_t)FC_SAFETY_FAULT_INITIALIZATION);
    }
    else
    {
        s_status.active_faults |= FC_SAFETY_FAULT_INITIALIZATION;
        s_status.arm_conditions_met = false;
        s_status.motor_output_allowed = false;
    }
}

void App_SafetyEvaluate(const FcRcInput_t *rc,
                        const FcImuData_t *imu,
                        const FcAttitude_t *attitude,
                        const FcBatteryStatus_t *battery,
                        bool scheduler_ok)
{
    uint32_t faults = FC_SAFETY_FAULT_NONE;

    if (!s_initialized)
    {
        return;
    }

    s_status.rc_online = rc_input_is_sane(rc);
    s_status.imu_ready = (imu != NULL) && imu->valid;
    s_status.imu_calibrated = (imu != NULL) && imu->calibrated;
#if FC_ENABLE_BATTERY_MONITOR
    s_status.battery_ok = (battery != NULL) && battery->valid && !battery->critical;
#else
    (void)battery;
    s_status.battery_ok = true;
#endif
    s_status.tilt_ok = (attitude != NULL) && attitude->valid &&
                       (absolute_float(attitude->roll_deg) <= FC_SAFETY_MAX_TILT_DEG) &&
                       (absolute_float(attitude->pitch_deg) <= FC_SAFETY_MAX_TILT_DEG);
    s_status.scheduler_ok = scheduler_ok;

    if (!s_status.rc_online) { faults |= FC_SAFETY_FAULT_RC_LOST; }
    if (!s_status.imu_ready || !s_status.imu_calibrated) { faults |= FC_SAFETY_FAULT_IMU_INVALID; }
#if FC_ENABLE_BATTERY_MONITOR
    if ((battery == NULL) || !battery->valid) { faults |= FC_SAFETY_FAULT_BATTERY_UNKNOWN; }
    else if (battery->critical) { faults |= FC_SAFETY_FAULT_BATTERY_CRITICAL; }
#endif
    if (!s_status.tilt_ok) { faults |= FC_SAFETY_FAULT_EXCESSIVE_TILT; }
    if (!scheduler_ok) { faults |= FC_SAFETY_FAULT_SCHEDULER; }
    if ((rc == NULL) || !rc->arm_switch || !rc->safety_switch) { faults |= FC_SAFETY_FAULT_ARM_SWITCH; }
    if ((rc != NULL) && rc->emergency_stop) { faults |= FC_SAFETY_FAULT_EMERGENCY_STOP; }
    if (!s_status.initialization_ok) { faults |= FC_SAFETY_FAULT_INITIALIZATION; }

    s_status.active_faults = faults;
    s_status.motor_output_allowed = (faults == FC_SAFETY_FAULT_NONE);
    s_status.arm_conditions_met = s_status.motor_output_allowed &&
                                  (rc != NULL) && rc->throttle_low &&
                                  (rc->throttle <= FC_RC_THROTTLE_ARM_MAX);
}

bool App_SafetyArmConditionsMet(void)
{
    return s_initialized && s_status.arm_conditions_met;
}

bool App_SafetyMotorOutputAllowed(void)
{
    return s_initialized && s_status.motor_output_allowed;
}

FcStatus_t App_SafetyGetStatus(FcSafetyStatus_t *status)
{
    if (status == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    *status = s_status;
    return s_initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}
