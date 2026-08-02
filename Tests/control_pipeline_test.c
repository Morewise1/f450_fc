/* End-to-end host test: RC sticks -> attitude loop -> rate PID -> Quad-X mixer. */

#include "ctl_attitude.h"
#include "ctl_mixer.h"
#include "ctl_rate.h"
#include "ctl_rc_map.h"
#include "fc_board.h"
#include "fc_config.h"

static int run_stick(int16_t roll,
                     int16_t pitch,
                     int16_t yaw,
                     FcControlOutput_t *control,
                     FcMotorOutput_t *motors)
{
    FcRcInput_t rc = {0};
    FcPilotCommand_t pilot;
    FcControlTarget_t target = {0};
    FcAttitude_t attitude = {0};
    FcImuData_t imu = {0};
    FcVector3f_t target_rate;

    rc.roll = roll;
    rc.pitch = pitch;
    rc.yaw = yaw;
    rc.throttle = 400U;
    rc.link_valid = true;
    rc.arm_switch = true;
    rc.safety_switch = true;
    if (Ctl_RcMapUpdate(&rc, &pilot) != FC_STATUS_OK) { return 1; }

    target.throttle_us = 1400U;
    target.roll_deg = pilot.roll * FC_MAX_TARGET_TILT_DEG;
    target.pitch_deg = pilot.pitch * FC_MAX_TARGET_TILT_DEG;
    target.yaw_rate_dps = pilot.yaw * FC_MAX_TARGET_YAW_RATE_DPS;
    target.mode = FC_MODE_STABILIZE;
    attitude.valid = true;
    if (Ctl_AttitudeUpdate(&target, &attitude, FC_ATTITUDE_DT_S, &target_rate) != FC_STATUS_OK)
    {
        return 2;
    }

    imu.valid = true;
    Ctl_RateReset();
    if (Ctl_RateUpdate(&target_rate, &imu, FC_CONTROL_DT_S, control) != FC_STATUS_OK)
    {
        return 3;
    }
    if (Ctl_MixerQuadX(target.throttle_us,
                       control->roll_cmd_us,
                       control->pitch_cmd_us,
                       control->yaw_cmd_us,
                       motors) != FC_STATUS_OK)
    {
        return 4;
    }
    return 0;
}

static int motors_in_range(const FcMotorOutput_t *motors)
{
    uint32_t index;
    for (index = 0U; index < FC_MOTOR_COUNT; ++index)
    {
        if ((motors->motor_us[index] < FC_ESC_MIN_US) ||
            (motors->motor_us[index] > FC_ESC_COMMAND_MAX_US))
        {
            return 0;
        }
    }
    return motors->valid ? 1 : 0;
}

int main(void)
{
    FcControlOutput_t control;
    FcMotorOutput_t motors;
    FcVector3f_t target_rate = {0};
    FcImuData_t imu = {0};

    if (Ctl_AttitudeInit() != FC_STATUS_OK) { return 1; }
    if (Ctl_RateInit() != FC_STATUS_OK) { return 2; }

    /* Positive roll: left motors M3/M4 rise, right motors M1/M2 fall. */
    if (run_stick(FC_RC_AXIS_MAX, 0, 0, &control, &motors) != 0) { return 3; }
    if (!(control.roll_cmd_us > 1.0f) ||
        !(motors.motor_us[FC_MOTOR_INDEX_M3] > motors.motor_us[FC_MOTOR_INDEX_M1]) ||
        !(motors.motor_us[FC_MOTOR_INDEX_M4] > motors.motor_us[FC_MOTOR_INDEX_M2]) ||
        !motors_in_range(&motors)) { return 4; }

    /* Positive pitch: front M1/M4 rise, rear M2/M3 fall. */
    if (run_stick(0, FC_RC_AXIS_MAX, 0, &control, &motors) != 0) { return 5; }
    if (!(control.pitch_cmd_us > 1.0f) ||
        !(motors.motor_us[FC_MOTOR_INDEX_M1] > motors.motor_us[FC_MOTOR_INDEX_M2]) ||
        !(motors.motor_us[FC_MOTOR_INDEX_M4] > motors.motor_us[FC_MOTOR_INDEX_M3]) ||
        !motors_in_range(&motors)) { return 6; }

    /* Positive yaw: the CCW motor pair M1/M3 rises. */
    if (run_stick(0, 0, FC_RC_AXIS_MAX, &control, &motors) != 0) { return 7; }
    if (!(control.yaw_cmd_us > 1.0f) ||
        !(motors.motor_us[FC_MOTOR_INDEX_M1] > motors.motor_us[FC_MOTOR_INDEX_M2]) ||
        !(motors.motor_us[FC_MOTOR_INDEX_M3] > motors.motor_us[FC_MOTOR_INDEX_M4]) ||
        !motors_in_range(&motors)) { return 8; }

    /* Measured positive roll with a zero target must command negative roll. */
    imu.valid = true;
    imu.gyro_dps.x = 50.0f;
    Ctl_RateReset();
    if (Ctl_RateUpdate(&target_rate, &imu, FC_CONTROL_DT_S, &control) != FC_STATUS_OK) { return 9; }
    if (!(control.roll_cmd_us < -1.0f)) { return 10; }
    if (Ctl_MixerQuadX(1400U,
                       control.roll_cmd_us,
                       control.pitch_cmd_us,
                       control.yaw_cmd_us,
                       &motors) != FC_STATUS_OK) { return 11; }
    if (!(motors.motor_us[FC_MOTOR_INDEX_M1] > motors.motor_us[FC_MOTOR_INDEX_M3]) ||
        !(motors.motor_us[FC_MOTOR_INDEX_M2] > motors.motor_us[FC_MOTOR_INDEX_M4]) ||
        !motors_in_range(&motors)) { return 12; }

    return 0;
}
