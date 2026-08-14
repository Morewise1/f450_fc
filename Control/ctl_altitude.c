/* Conservative cascaded altitude controller. */

#include <math.h>
#include <stddef.h>
#include "ctl_altitude.h"
#include "fc_config.h"

static float s_velocity_integral_us;
static bool s_initialized;
volatile CtlAltitudeDebug_t g_ctl_altitude_debug;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

FcStatus_t Ctl_AltitudeInit(void)
{
    s_velocity_integral_us = 0.0f;
    s_initialized = true;
    g_ctl_altitude_debug = (CtlAltitudeDebug_t){0};
    return FC_STATUS_OK;
}

FcStatus_t Ctl_AltitudeUpdate(float target_altitude_m,
                              const FcAltitude_t *altitude,
                              float dt_s,
                              float *throttle_correction_us)
{
    float altitude_error_m;
    float effective_altitude_error_m;
    float target_velocity_mps;
    float velocity_error_mps;
    float proportional_us;
    float unclamped_output_us;
    float output_us;

    if ((altitude == NULL) || (throttle_correction_us == NULL) ||
        (dt_s <= 0.0f) || (dt_s > 0.2f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *throttle_correction_us = 0.0f;
    if (!s_initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (!altitude->valid) { return FC_STATUS_INVALID_DATA; }

    altitude_error_m = target_altitude_m - altitude->altitude_m;
    if (fabsf(altitude_error_m) <= FC_ALTITUDE_ERROR_DEADBAND_M)
    {
        effective_altitude_error_m = 0.0f;
    }
    else
    {
        effective_altitude_error_m = altitude_error_m -
            ((altitude_error_m > 0.0f) ? FC_ALTITUDE_ERROR_DEADBAND_M :
                                        -FC_ALTITUDE_ERROR_DEADBAND_M);
    }
    target_velocity_mps = clamp_float(effective_altitude_error_m * FC_ALTITUDE_POSITION_KP,
                                      -FC_ALTITUDE_MAX_VERTICAL_SPEED_MPS,
                                      FC_ALTITUDE_MAX_VERTICAL_SPEED_MPS);
    velocity_error_mps = target_velocity_mps - altitude->vertical_velocity_mps;
    proportional_us = FC_ALTITUDE_VELOCITY_KP * velocity_error_mps;
    s_velocity_integral_us += FC_ALTITUDE_VELOCITY_KI * velocity_error_mps * dt_s;
    s_velocity_integral_us = clamp_float(s_velocity_integral_us,
                                         -FC_ALTITUDE_VELOCITY_I_LIMIT_US,
                                         FC_ALTITUDE_VELOCITY_I_LIMIT_US);
    unclamped_output_us = proportional_us + s_velocity_integral_us;
    output_us = clamp_float(unclamped_output_us,
                            -FC_ALTITUDE_CORRECTION_LIMIT_US,
                            FC_ALTITUDE_CORRECTION_LIMIT_US);
    if (output_us != unclamped_output_us)
    {
        /* Back-calculate the integrator so a saturated command cannot wind up. */
        s_velocity_integral_us = clamp_float(output_us - proportional_us,
                                             -FC_ALTITUDE_VELOCITY_I_LIMIT_US,
                                             FC_ALTITUDE_VELOCITY_I_LIMIT_US);
    }

    *throttle_correction_us = output_us;
    g_ctl_altitude_debug.altitude_error_m = altitude_error_m;
    g_ctl_altitude_debug.target_vertical_velocity_mps = target_velocity_mps;
    g_ctl_altitude_debug.velocity_error_mps = velocity_error_mps;
    g_ctl_altitude_debug.integral_us = s_velocity_integral_us;
    g_ctl_altitude_debug.output_us = output_us;
    g_ctl_altitude_debug.saturated = output_us != unclamped_output_us;
    return FC_STATUS_OK;
}

void Ctl_AltitudeReset(void)
{
    s_velocity_integral_us = 0.0f;
    g_ctl_altitude_debug = (CtlAltitudeDebug_t){0};
}
