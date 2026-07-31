#ifndef FC_CONFIG_H
#define FC_CONFIG_H

/* Project-wide timing, units, limits, and conservative safety defaults. */

#ifndef FC_USE_STM32_HAL
#define FC_USE_STM32_HAL                    0U
#endif
#ifndef FC_BOARD_HAL_BINDINGS_COMPLETE
#define FC_BOARD_HAL_BINDINGS_COMPLETE      0U
#endif

#define FC_SCHEDULER_TICK_HZ             1000U
#define FC_CONTROL_RATE_HZ                500U
#define FC_ATTITUDE_RATE_HZ               250U
#define FC_RC_UPDATE_RATE_HZ              100U
#define FC_BATTERY_UPDATE_RATE_HZ         100U
#define FC_ALTITUDE_RATE_HZ                50U
#define FC_HOUSEKEEPING_RATE_HZ             10U

#define FC_CONTROL_DT_S       (1.0f / (float)FC_CONTROL_RATE_HZ)
#define FC_ATTITUDE_DT_S      (1.0f / (float)FC_ATTITUDE_RATE_HZ)
#define FC_ALTITUDE_DT_S      (1.0f / (float)FC_ALTITUDE_RATE_HZ)

#define FC_MOTOR_COUNT                      4U
#define FC_ESC_STOP_US                   1000U
#define FC_ESC_MIN_US                    1000U
#ifndef FC_ESC_IDLE_US
#define FC_ESC_IDLE_US                   1100U
#endif
#define FC_ESC_MAX_US                    2000U
#ifndef FC_ESC_COMMAND_MAX_US
#define FC_ESC_COMMAND_MAX_US            2000U
#endif
#ifndef FC_MOTOR_TEST_MAX_US
#define FC_MOTOR_TEST_MAX_US             1200U
#endif
#ifndef FC_ENABLE_MOTOR_TEST
#define FC_ENABLE_MOTOR_TEST                0U
#endif
#ifndef FC_ESC_PWM_FRAME_HZ
#define FC_ESC_PWM_FRAME_HZ               400U
#endif
#ifndef FC_ESC_TIMER_COUNTER_HZ
#define FC_ESC_TIMER_COUNTER_HZ        1000000U
#endif
#define FC_READY_USE_IDLE_OUTPUT            0U

#define FC_RC_AXIS_MIN                   (-500)
#define FC_RC_AXIS_MAX                     500
#define FC_RC_THROTTLE_MIN                   0U
#define FC_RC_THROTTLE_MAX                1000U
#define FC_RC_THROTTLE_ARM_MAX              50U
#define FC_RC_THROTTLE_TAKEOFF_MIN         100U
#define FC_RC_TIMEOUT_MS                   100U
#define FC_RC_AXIS_DEADBAND                  25
#define FC_RC_AXIS_EXPO                     0.30f
#define FC_RC_THROTTLE_DEADBAND             30U
#define FC_RC_THROTTLE_EXPO                 0.30f
#define FC_RC_MAX_CLIMB_RATE_MPS             2.0f

#define FC_IBUS_FRAME_LENGTH                32U
#define FC_IBUS_CHANNEL_COUNT               14U
#define FC_IBUS_LENGTH_BYTE               0x20U
#define FC_IBUS_COMMAND_BYTE              0x40U
#define FC_IBUS_INTERBYTE_TIMEOUT_MS         3U

#define FC_IMU_DATA_TIMEOUT_MS              20U
#define FC_IMU_CALIBRATION_SAMPLE_COUNT   2000U
#define FC_IMU_CAL_MAX_GYRO_DPS              3.0f
#define FC_IMU_CAL_ACCEL_MAG_MIN_SQ           0.7225f
#define FC_IMU_CAL_ACCEL_MAG_MAX_SQ           1.3225f
#define FC_IMU_BIAS_TRACK_ALPHA                0.0002f

/* Software filtering after body-axis mapping and gyro bias correction. */
#define FC_IMU_ACCEL_LPF_HZ                    30.0f
#define FC_IMU_GYRO_LPF_HZ                     80.0f
#define FC_IMU_FILTER_MAX_DT_S                   0.02f

#define FC_AHRS_MAHONY_KP                      2.0f
#define FC_AHRS_MAHONY_KI                      0.05f
#define FC_AHRS_INTEGRAL_LIMIT_RAD_S           0.20f
#define FC_AHRS_ACCEL_MIN_NORM_SQ              0.25f
#define FC_AHRS_ACCEL_MAX_NORM_SQ              2.25f

#define FC_BMI088_I2C_TIMEOUT_MS               2U
#define FC_BMI088_ACCEL_I2C_ADDRESS_LOW      0x18U
#define FC_BMI088_ACCEL_I2C_ADDRESS_HIGH     0x19U
#define FC_BMI088_GYRO_I2C_ADDRESS_LOW       0x68U
#define FC_BMI088_GYRO_I2C_ADDRESS_HIGH      0x69U
#define FC_BMI088_STARTUP_DELAY_MS            10U
#define FC_BMI088_ACCEL_RESET_DELAY_MS         5U
#define FC_BMI088_ACCEL_POWER_DELAY_MS        50U
#define FC_BMI088_GYRO_RESET_DELAY_MS         30U
#define FC_BMI088_EXPECTED_ACCEL_CHIP_ID     0x1EU
#define FC_BMI088_EXPECTED_GYRO_CHIP_ID      0x0FU

#define FC_MAX_TARGET_TILT_DEG             25.0f
#define FC_MAX_TARGET_YAW_RATE_DPS        180.0f
#define FC_MAX_TARGET_RATE_DPS            250.0f
#define FC_SAFETY_MAX_TILT_DEG             60.0f

#define FC_MIXER_ROLL_LIMIT_US             400.0f
#define FC_MIXER_PITCH_LIMIT_US            400.0f
#define FC_MIXER_YAW_LIMIT_US              300.0f

#define FC_BATTERY_CELL_COUNT                3U
#define FC_BATTERY_WARNING_PER_CELL_V        3.50f
#define FC_BATTERY_CRITICAL_PER_CELL_V       3.30f

#if (FC_SCHEDULER_TICK_HZ == 0U) || (FC_CONTROL_RATE_HZ == 0U) || \
    (FC_ATTITUDE_RATE_HZ == 0U) || (FC_RC_UPDATE_RATE_HZ == 0U) || \
    (FC_ALTITUDE_RATE_HZ == 0U) || (FC_HOUSEKEEPING_RATE_HZ == 0U)
#error "Scheduler frequencies must be non-zero"
#endif

#if (FC_CONTROL_RATE_HZ > FC_SCHEDULER_TICK_HZ) || \
    (FC_ATTITUDE_RATE_HZ > FC_SCHEDULER_TICK_HZ) || \
    (FC_RC_UPDATE_RATE_HZ > FC_SCHEDULER_TICK_HZ) || \
    (FC_ALTITUDE_RATE_HZ > FC_SCHEDULER_TICK_HZ) || \
    (FC_HOUSEKEEPING_RATE_HZ > FC_SCHEDULER_TICK_HZ)
#error "Scheduler task frequencies cannot exceed the base tick"
#endif

#if (FC_SCHEDULER_TICK_HZ % FC_CONTROL_RATE_HZ) != 0U || \
    (FC_SCHEDULER_TICK_HZ % FC_ATTITUDE_RATE_HZ) != 0U || \
    (FC_SCHEDULER_TICK_HZ % FC_RC_UPDATE_RATE_HZ) != 0U || \
    (FC_SCHEDULER_TICK_HZ % FC_ALTITUDE_RATE_HZ) != 0U || \
    (FC_SCHEDULER_TICK_HZ % FC_HOUSEKEEPING_RATE_HZ) != 0U
#error "Scheduler task rates must divide the 1 kHz tick exactly"
#endif

#if (FC_RC_THROTTLE_TAKEOFF_MIN <= FC_RC_THROTTLE_ARM_MAX) || \
    (FC_RC_THROTTLE_TAKEOFF_MIN > FC_RC_THROTTLE_MAX)
#error "Takeoff throttle threshold must be above the arm threshold"
#endif

#if (FC_ESC_STOP_US != 1000U) || (FC_ESC_MIN_US != 1000U) || (FC_ESC_MAX_US != 2000U)
#error "Standard ESC pulse range must remain 1000-2000 us"
#endif

#if (FC_ESC_IDLE_US < FC_ESC_MIN_US) || (FC_ESC_IDLE_US > FC_ESC_MAX_US)
#error "FC_ESC_IDLE_US must be inside the ESC range"
#endif

#if (FC_ESC_COMMAND_MAX_US < FC_ESC_IDLE_US) || (FC_ESC_COMMAND_MAX_US > FC_ESC_MAX_US)
#error "FC_ESC_COMMAND_MAX_US must be between idle and the protocol maximum"
#endif

#if (FC_MOTOR_TEST_MAX_US < FC_ESC_MIN_US) || (FC_MOTOR_TEST_MAX_US > FC_ESC_COMMAND_MAX_US)
#error "FC_MOTOR_TEST_MAX_US must be inside the allowed command range"
#endif

#if (FC_RC_AXIS_DEADBAND < 0) || (FC_RC_AXIS_DEADBAND >= FC_RC_AXIS_MAX)
#error "FC_RC_AXIS_DEADBAND must be inside the normalized stick range"
#endif

#if (FC_RC_THROTTLE_DEADBAND >= FC_RC_THROTTLE_MAX)
#error "FC_RC_THROTTLE_DEADBAND must be below full throttle"
#endif

#if (FC_ESC_TIMER_COUNTER_HZ == 0U) || (FC_ESC_PWM_FRAME_HZ == 0U)
#error "ESC timer counter and frame frequencies must be non-zero"
#endif

#if (FC_MOTOR_COUNT != 4U)
#error "The first version supports exactly four Quad-X motors"
#endif

#endif /* FC_CONFIG_H */
