/* Runs bounded PID controllers only when IMU data is valid. */

#include <stddef.h>
#include "ctl_rate.h"
#include "ctl_pid.h"
#include "fc_config.h"
#include "fc_params.h"

static CtlPid_t s_roll_pid;
static CtlPid_t s_pitch_pid;
static CtlPid_t s_yaw_pid;
static bool s_initialized;
volatile CtlRateDebug_t g_ctl_rate_debug;

static CtlPidConfig_t make_config(float kp,
                                  float ki,
                                  float kd,
                                  float output_limit,
                                  float integral_limit)
{
    CtlPidConfig_t config;
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
    config.output_offset = FC_PID_OUTPUT_OFFSET;
    config.input_deadband = FC_PID_INPUT_DEADBAND;
    config.integral_min = -integral_limit;
    config.integral_max = integral_limit;
    config.output_min = -output_limit;
    config.output_max = output_limit;
    config.integral_separation_threshold = FC_PID_INTEGRAL_SEPARATION;
    config.variable_integral_full_error = FC_PID_VARIABLE_I_FULL_ERROR;
    config.variable_integral_zero_error = FC_PID_VARIABLE_I_ZERO_ERROR;
    config.derivative_lpf_hz = FC_PID_DERIVATIVE_LPF_HZ;
    config.enable_integral_separation = FC_PID_ENABLE_INTEGRAL_SEPARATION != 0U;
    config.enable_variable_integral = FC_PID_ENABLE_VARIABLE_INTEGRAL != 0U;
    config.enable_anti_windup = FC_PID_ENABLE_ANTI_WINDUP != 0U;
    config.derivative_on_measurement = FC_PID_DERIVATIVE_ON_MEASUREMENT != 0U;
    config.enable_derivative_lpf = FC_PID_ENABLE_DERIVATIVE_LPF != 0U;
    return config;
}

FcStatus_t Ctl_RateInit(void)
{
    CtlPidConfig_t roll = make_config(FC_RATE_ROLL_KP,
                                      FC_RATE_ROLL_KI,
                                      FC_RATE_ROLL_KD,
                                      FC_MIXER_ROLL_LIMIT_US,
                                      FC_RATE_ROLL_I_LIMIT_US);
    CtlPidConfig_t pitch = make_config(FC_RATE_PITCH_KP,
                                       FC_RATE_PITCH_KI,
                                       FC_RATE_PITCH_KD,
                                       FC_MIXER_PITCH_LIMIT_US,
                                       FC_RATE_PITCH_I_LIMIT_US);
    CtlPidConfig_t yaw = make_config(FC_RATE_YAW_KP,
                                     FC_RATE_YAW_KI,
                                     FC_RATE_YAW_KD,
                                     FC_MIXER_YAW_LIMIT_US,
                                     FC_RATE_YAW_I_LIMIT_US);

    if ((Ctl_PidInit(&s_roll_pid, &roll) != FC_STATUS_OK) ||
        (Ctl_PidInit(&s_pitch_pid, &pitch) != FC_STATUS_OK) ||
        (Ctl_PidInit(&s_yaw_pid, &yaw) != FC_STATUS_OK))
    {
        s_initialized = false;
        return FC_STATUS_ERROR;
    }
    g_ctl_rate_debug = (CtlRateDebug_t){0};
    s_initialized = true;
    return FC_STATUS_OK;
}

FcStatus_t Ctl_RateUpdate(const FcVector3f_t *target_rate_dps,
                          const FcImuData_t *imu,
                          float dt_s,
                          FcControlOutput_t *output)
{
    CtlPidTerms_t terms;

    if ((target_rate_dps == NULL) || (imu == NULL) || (output == NULL) || (dt_s <= 0.0f))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *output = (FcControlOutput_t){0};
    if (!s_initialized) { return FC_STATUS_NOT_INITIALIZED; }
    if (!imu->valid) { return FC_STATUS_INVALID_DATA; }

    output->roll_cmd_us = Ctl_PidUpdate(&s_roll_pid, target_rate_dps->x, imu->gyro_dps.x, dt_s);
    output->pitch_cmd_us = Ctl_PidUpdate(&s_pitch_pid, target_rate_dps->y, imu->gyro_dps.y, dt_s);
    output->yaw_cmd_us = Ctl_PidUpdate(&s_yaw_pid, target_rate_dps->z, imu->gyro_dps.z, dt_s);
    (void)Ctl_PidGetTerms(&s_roll_pid, &terms);
    g_ctl_rate_debug.roll = terms;
    (void)Ctl_PidGetTerms(&s_pitch_pid, &terms);
    g_ctl_rate_debug.pitch = terms;
    (void)Ctl_PidGetTerms(&s_yaw_pid, &terms);
    g_ctl_rate_debug.yaw = terms;
    g_ctl_rate_debug.valid = true;
    output->valid = true;
    return FC_STATUS_OK;
}

void Ctl_RateReset(void)
{
    Ctl_PidReset(&s_roll_pid);
    Ctl_PidReset(&s_pitch_pid);
    Ctl_PidReset(&s_yaw_pid);
    g_ctl_rate_debug = (CtlRateDebug_t){0};
}

FcStatus_t Ctl_RateGetDebug(CtlRateDebug_t *debug)
{
    if (debug == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *debug = (CtlRateDebug_t){
        g_ctl_rate_debug.roll,
        g_ctl_rate_debug.pitch,
        g_ctl_rate_debug.yaw,
        g_ctl_rate_debug.valid
    };
    return s_initialized ? FC_STATUS_OK : FC_STATUS_NOT_INITIALIZED;
}
