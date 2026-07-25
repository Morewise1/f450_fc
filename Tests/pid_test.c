/* Host tests for optional advanced PID behavior and bounded state. */

#include <math.h>
#include "ctl_pid.h"

static int nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static CtlPidConfig_t base_config(void)
{
    CtlPidConfig_t config = {0};
    config.kp = 1.0f;
    config.ki = 1.0f;
    config.kd = 1.0f;
    config.integral_min = -10.0f;
    config.integral_max = 10.0f;
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    config.integral_separation_threshold = 5.0f;
    config.variable_integral_full_error = 1.0f;
    config.variable_integral_zero_error = 5.0f;
    config.derivative_lpf_hz = 10.0f;
    return config;
}

int main(void)
{
    CtlPid_t pid;
    CtlPidConfig_t config = base_config();
    CtlPidTerms_t terms;
    float output;

    config.output_offset = 2.0f;
    config.input_deadband = 0.5f;
    if (Ctl_PidInit(&pid, &config) != FC_STATUS_OK) { return 1; }
    output = Ctl_PidUpdate(&pid, 0.4f, 0.0f, 0.01f);
    if (!nearly_equal(output, 2.0f, 0.0001f)) { return 2; }

    config = base_config();
    config.enable_integral_separation = true;
    if (Ctl_PidSetConfig(&pid, &config) != FC_STATUS_OK) { return 3; }
    (void)Ctl_PidUpdate(&pid, 10.0f, 0.0f, 0.1f);
    if (Ctl_PidGetTerms(&pid, &terms) != FC_STATUS_OK) { return 4; }
    if ((terms.integral_scale != 0.0f) || (terms.integral != 0.0f)) { return 5; }

    config = base_config();
    config.enable_variable_integral = true;
    config.kp = 0.0f;
    config.kd = 0.0f;
    if (Ctl_PidSetConfig(&pid, &config) != FC_STATUS_OK) { return 6; }
    (void)Ctl_PidUpdate(&pid, 3.0f, 0.0f, 1.0f);
    if (Ctl_PidGetTerms(&pid, &terms) != FC_STATUS_OK) { return 7; }
    if (!nearly_equal(terms.integral_scale, 0.5f, 0.0001f) ||
        !nearly_equal(terms.integral, 1.5f, 0.0001f)) { return 8; }

    config = base_config();
    config.output_min = -5.0f;
    config.output_max = 5.0f;
    config.enable_anti_windup = true;
    if (Ctl_PidSetConfig(&pid, &config) != FC_STATUS_OK) { return 9; }
    output = Ctl_PidUpdate(&pid, 20.0f, 0.0f, 0.1f);
    if (!nearly_equal(output, 5.0f, 0.0001f)) { return 10; }
    if (Ctl_PidGetTerms(&pid, &terms) != FC_STATUS_OK ||
        !nearly_equal(terms.integral, 0.0f, 0.0001f) || !terms.output_saturated) { return 11; }

    config = base_config();
    config.kp = 0.0f;
    config.ki = 0.0f;
    config.derivative_on_measurement = true;
    config.enable_derivative_lpf = false;
    if (Ctl_PidSetConfig(&pid, &config) != FC_STATUS_OK) { return 12; }
    (void)Ctl_PidUpdate(&pid, 0.0f, 0.0f, 0.1f);
    output = Ctl_PidUpdate(&pid, 10.0f, 0.0f, 0.1f);
    if (!nearly_equal(output, 0.0f, 0.0001f)) { return 13; }
    output = Ctl_PidUpdate(&pid, 10.0f, 1.0f, 0.1f);
    if (!nearly_equal(output, -10.0f, 0.0001f)) { return 14; }

    config.enable_derivative_lpf = true;
    config.derivative_lpf_hz = 1.0f;
    if (Ctl_PidSetConfig(&pid, &config) != FC_STATUS_OK) { return 15; }
    (void)Ctl_PidUpdate(&pid, 0.0f, 0.0f, 0.1f);
    output = Ctl_PidUpdate(&pid, 0.0f, 1.0f, 0.1f);
    if (!(output < 0.0f && output > -10.0f)) { return 16; }

    config.output_min = 10.0f;
    config.output_max = -10.0f;
    if (Ctl_PidSetConfig(&pid, &config) != FC_STATUS_INVALID_ARGUMENT) { return 17; }
    return 0;
}
