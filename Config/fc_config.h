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

#define FC_THROTTLE_CURVE_MODE_LINEAR         0U
#define FC_THROTTLE_CURVE_MODE_EXPO           1U
#ifndef FC_THROTTLE_CURVE_MODE
#define FC_THROTTLE_CURVE_MODE FC_THROTTLE_CURVE_MODE_LINEAR
#endif

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
#define FC_IMU_GYRO_LPF_HZ                     55.0f
#define FC_IMU_FILTER_MAX_DT_S                   0.02f

#define FC_AHRS_MAHONY_KP                      2.0f
#define FC_AHRS_MAHONY_KI                      0.05f
#define FC_AHRS_INTEGRAL_LIMIT_RAD_S           0.20f
#define FC_AHRS_ACCEL_MIN_NORM_SQ              0.25f
#define FC_AHRS_ACCEL_MAX_NORM_SQ              2.25f

/* Establish the take-off surface as zero while the aircraft is stationary. */
#define FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT       250U
#define FC_ATTITUDE_LEVEL_CAL_MAX_GYRO_DPS         1.5f
#define FC_ATTITUDE_LEVEL_CAL_ACCEL_MIN_SQ          0.81f
#define FC_ATTITUDE_LEVEL_CAL_ACCEL_MAX_SQ          1.21f
#define FC_ATTITUDE_LEVEL_MAX_TRIM_DEG              12.0f

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

#define FC_SENSOR_I2C_TIMEOUT_MS                2U

/*
 * Independent GPIO bit-banged buses for BMP388 and MMC5983MA.  A 5 us
 * half-period targets 100 kHz and leaves timing margin for HAL GPIO calls.
 * Clock stretching or a stuck SCL line is abandoned after 100 us so a bad
 * sensor cannot stall the flight loop indefinitely.
 */
#define FC_SOFT_I2C_HALF_PERIOD_US              5U
#define FC_SOFT_I2C_STRETCH_TIMEOUT_US        100U
#define FC_BMP388_I2C_ADDRESS_LOW             0x76U
#define FC_BMP388_I2C_ADDRESS_HIGH            0x77U
#define FC_BMP388_EXPECTED_CHIP_ID             0x50U
#define FC_BMP388_STARTUP_DELAY_MS               10U
#define FC_BMP388_RESET_DELAY_MS                 10U
#define FC_MMC5983MA_I2C_ADDRESS               0x30U
#define FC_MMC5983MA_EXPECTED_PRODUCT_ID        0x30U
#define FC_MMC5983MA_STARTUP_DELAY_MS             10U
#define FC_MMC5983MA_RESET_DELAY_MS               10U
#define FC_MMC5983MA_SET_DELAY_MS                  1U

/*
 * MMC5983MA module orientation.  The default assumes sensor X points to the
 * aircraft nose, Y points right, and Z points down.  Change only these macros
 * after checking the actual breakout-board axis arrows.
 */
#define FC_MAG_BODY_X_SOURCE                        0U
#define FC_MAG_BODY_Y_SOURCE                        1U
#define FC_MAG_BODY_Z_SOURCE                        2U
#define FC_MAG_BODY_X_SIGN                         1.0f
#define FC_MAG_BODY_Y_SIGN                         1.0f
#define FC_MAG_BODY_Z_SIGN                         1.0f
#define FC_MAG_OFFSET_X_UT                         0.0f
#define FC_MAG_OFFSET_Y_UT                         0.0f
#define FC_MAG_OFFSET_Z_UT                         0.0f
#define FC_MAG_SCALE_X                             1.0f
#define FC_MAG_SCALE_Y                             1.0f
#define FC_MAG_SCALE_Z                             1.0f
#define FC_MAG_VALID_MIN_FIELD_UT                 15.0f
#define FC_MAG_VALID_MAX_FIELD_UT                100.0f
#define FC_MAG_CAL_MIN_SAMPLES                    300U
#define FC_MAG_CAL_MIN_AXIS_SPAN_UT               20.0f

/*
 * Keep magnetic yaw correction disabled until axis direction, full-rotation
 * calibration, and motor-current interference have been verified on hardware.
 * The fusion implementation is present; set this to 1U only after those tests.
 */
#ifndef FC_ENABLE_MAG_YAW_FUSION
#define FC_ENABLE_MAG_YAW_FUSION                    0U
#endif
#define FC_MAG_YAW_KP                               0.8f
#define FC_MAG_YAW_MAX_CORRECTION_DPS              20.0f
#define FC_MAG_DATA_TIMEOUT_MS                     100U

/*
 * BMP388 pressure-filter profile.  This changes only the barometer path;
 * BMI088 and MMC5983MA filtering remain untouched.
 *
 * FAST:
 *   BMP388 IIR coefficient 1, 8 Hz software pressure LPF, 5 Hz velocity LPF.
 *   Lowest delay; use for bench response checks or a well-shielded barometer.
 * BALANCED (recommended default):
 *   BMP388 IIR coefficient 3, 5 Hz pressure LPF, 3 Hz velocity LPF.
 *   Best starting compromise for a small F450 with moderate propeller wash.
 * STRONG:
 *   BMP388 IIR coefficient 7, 3 Hz pressure LPF, 2 Hz velocity LPF.
 *   Use when altitude visibly jitters; it is smoother but reacts later.
 *
 * Change FC_BARO_FILTER_MODE, rebuild the whole Keil project, and re-flash.
 */
#define FC_BARO_FILTER_MODE_FAST                   0U
#define FC_BARO_FILTER_MODE_BALANCED               1U
#define FC_BARO_FILTER_MODE_STRONG                 2U
#ifndef FC_BARO_FILTER_MODE
#define FC_BARO_FILTER_MODE FC_BARO_FILTER_MODE_BALANCED
#endif

#if FC_BARO_FILTER_MODE == FC_BARO_FILTER_MODE_FAST
#define FC_BMP388_IIR_REGISTER_VALUE             0x02U
#define FC_BARO_PRESSURE_LPF_HZ                    8.0f
#define FC_BARO_VELOCITY_LPF_HZ                    5.0f
#elif FC_BARO_FILTER_MODE == FC_BARO_FILTER_MODE_BALANCED
#define FC_BMP388_IIR_REGISTER_VALUE             0x04U
#define FC_BARO_PRESSURE_LPF_HZ                    5.0f
#define FC_BARO_VELOCITY_LPF_HZ                    3.0f
#elif FC_BARO_FILTER_MODE == FC_BARO_FILTER_MODE_STRONG
#define FC_BMP388_IIR_REGISTER_VALUE             0x06U
#define FC_BARO_PRESSURE_LPF_HZ                    3.0f
#define FC_BARO_VELOCITY_LPF_HZ                    2.0f
#else
#error "FC_BARO_FILTER_MODE must be FAST, BALANCED, or STRONG"
#endif

/* BMP388 + BMI088 relative-altitude estimator and conservative controller. */
#define FC_BARO_REFERENCE_SAMPLE_COUNT             100U
#define FC_BARO_MAX_SAMPLE_STEP_M                    1.5f
#define FC_BARO_INNOVATION_LIMIT_M                   2.0f
#define FC_VERTICAL_BARO_POSITION_BLEND              0.10f
#define FC_VERTICAL_BARO_VELOCITY_BLEND              0.08f
#define FC_VERTICAL_ACCEL_MIN_NORM_SQ                 0.36f
#define FC_VERTICAL_ACCEL_MAX_NORM_SQ                 2.89f
#define FC_VERTICAL_ACCEL_LIMIT_MPS2                 12.0f
#define FC_VERTICAL_MAX_PREDICT_DT_S                  0.02f
/* A shaky aircraft may bridge short pressure dropouts for 0.4 s using inertia. */
#define FC_BARO_INERTIAL_HOLD_TIMEOUT_MS             400U
#define FC_GRAVITY_MPS2                          9.80665f
#define FC_ALTITUDE_POSITION_KP                     1.0f
#define FC_ALTITUDE_VELOCITY_KP                    90.0f
#define FC_ALTITUDE_VELOCITY_KI                    25.0f
#define FC_ALTITUDE_VELOCITY_I_LIMIT_US           100.0f
#define FC_ALTITUDE_CORRECTION_LIMIT_US           220.0f
#define FC_ALTITUDE_ERROR_DEADBAND_M                0.05f
#define FC_ALTITUDE_MAX_VERTICAL_SPEED_MPS          0.8f
#define FC_ALTITUDE_STICK_DEADBAND                   0.06f
#define FC_ALTITUDE_MIN_TARGET_M                     0.10f
#define FC_ALTITUDE_MAX_TARGET_M                     8.0f

#define FC_MAX_TARGET_TILT_DEG             25.0f
#define FC_MAX_TARGET_YAW_RATE_DPS        120.0f
#define FC_MAX_TARGET_RATE_DPS            250.0f
#define FC_SAFETY_MAX_TILT_DEG             60.0f

#define FC_YAW_CONTROL_MODE_RATE             0U
#define FC_YAW_CONTROL_MODE_HEADING_HOLD     1U
#ifndef FC_YAW_CONTROL_MODE
#define FC_YAW_CONTROL_MODE FC_YAW_CONTROL_MODE_HEADING_HOLD
#endif

#define FC_MIXER_ROLL_LIMIT_US             400.0f
#define FC_MIXER_PITCH_LIMIT_US            400.0f
#define FC_MIXER_YAW_LIMIT_US              120.0f

#ifndef FC_ENABLE_BATTERY_MONITOR
#define FC_ENABLE_BATTERY_MONITOR             0U
#endif
#define FC_BATTERY_CELL_COUNT                3U
#define FC_BATTERY_WARNING_PER_CELL_V        3.50f
#define FC_BATTERY_CRITICAL_PER_CELL_V       3.30f

#if FC_ENABLE_BATTERY_MONITOR > 1U
#error "FC_ENABLE_BATTERY_MONITOR must be 0 or 1"
#endif

#if FC_ENABLE_MAG_YAW_FUSION > 1U
#error "FC_ENABLE_MAG_YAW_FUSION must be 0 or 1"
#endif

#if (FC_MAG_BODY_X_SOURCE > 2U) || (FC_MAG_BODY_Y_SOURCE > 2U) || \
    (FC_MAG_BODY_Z_SOURCE > 2U) || \
    (FC_MAG_BODY_X_SOURCE == FC_MAG_BODY_Y_SOURCE) || \
    (FC_MAG_BODY_X_SOURCE == FC_MAG_BODY_Z_SOURCE) || \
    (FC_MAG_BODY_Y_SOURCE == FC_MAG_BODY_Z_SOURCE)
#error "Magnetometer body-axis mapping must be a permutation of X, Y, and Z"
#endif

#if FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT == 0U
#error "FC_ATTITUDE_LEVEL_CAL_SAMPLE_COUNT must be non-zero"
#endif

#if (FC_THROTTLE_CURVE_MODE != FC_THROTTLE_CURVE_MODE_LINEAR) && \
    (FC_THROTTLE_CURVE_MODE != FC_THROTTLE_CURVE_MODE_EXPO)
#error "FC_THROTTLE_CURVE_MODE is invalid"
#endif

#if (FC_YAW_CONTROL_MODE != FC_YAW_CONTROL_MODE_RATE) && \
    (FC_YAW_CONTROL_MODE != FC_YAW_CONTROL_MODE_HEADING_HOLD)
#error "FC_YAW_CONTROL_MODE is invalid"
#endif

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
