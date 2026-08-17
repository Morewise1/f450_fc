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

/*
 * USART6/ESP telemetry is always available.  Remote motor control remains
 * disabled until the link timeout, dead-man switch, disarm, and emergency
 * stop paths have all passed no-prop tests.
 */
#ifndef FC_ENABLE_APP_CONTROL
#define FC_ENABLE_APP_CONTROL                 0U
#endif
#define FC_APP_CONTROL_TIMEOUT_MS           150U

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

/* IMU通用低通：同时服务姿态环，定高通道不要通过降低这里的频率来调试。 */
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
#define FC_MAG_BODY_Y_SIGN                         -1.0f
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
 * Gyroscope integration remains the primary yaw source.  When enabled, the
 * magnetometer contributes only bounded low-frequency drift correction and is
 * automatically rejected if its horizontal projection becomes unreliable.
 */
#ifndef FC_ENABLE_MAG_YAW_FUSION
#define FC_ENABLE_MAG_YAW_FUSION                    1U
#endif
#define FC_MAG_YAW_KP                               0.8f
#define FC_MAG_YAW_MAX_CORRECTION_DPS              20.0f
#define FC_MAG_DATA_TIMEOUT_MS                     100U
#define FC_MAG_MIN_HORIZONTAL_FIELD_UT               5.0f
#define FC_MAG_MIN_HORIZONTAL_RATIO                  0.10f
#define FC_MAG_YAW_MAX_TILT_DEG                     70.0f
#define FC_MAG_YAW_ERROR_REJECT_DEG                 45.0f

/*
 * BMP388滤波模式，只改变气压高度链路，不影响BMI088和MMC5983MA。
 *
 * FAST：硬件IIR系数1，软件气压低通5Hz，气压速度低通3Hz。
 *       延迟最小，用于响应检查或气压计防风良好的情况。
 * BALANCED（默认推荐）：硬件IIR系数3，气压低通2Hz，速度低通1Hz。
 *       适合室内调试和F450首轮低空飞行。
 * STRONG：硬件IIR系数7，气压低通0.8Hz，速度低通0.4Hz。
 *       只用于台架诊断或气流噪声很大时，延迟明显，不能直接首飞。
 *
 * 修改FC_BARO_FILTER_MODE后必须完整编译Keil工程并重新烧录。
 */
#define FC_BARO_FILTER_MODE_FAST                   0U
#define FC_BARO_FILTER_MODE_BALANCED               1U
#define FC_BARO_FILTER_MODE_STRONG                 2U
#ifndef FC_BARO_FILTER_MODE
#define FC_BARO_FILTER_MODE FC_BARO_FILTER_MODE_BALANCED
#endif

#if FC_BARO_FILTER_MODE == FC_BARO_FILTER_MODE_FAST
#define FC_BMP388_IIR_REGISTER_VALUE             0x02U
#define FC_BARO_PRESSURE_LPF_HZ                    5.0f
#define FC_BARO_VELOCITY_LPF_HZ                    3.0f
#elif FC_BARO_FILTER_MODE == FC_BARO_FILTER_MODE_BALANCED
#define FC_BMP388_IIR_REGISTER_VALUE             0x04U
#define FC_BARO_PRESSURE_LPF_HZ                    4.0f
#define FC_BARO_VELOCITY_LPF_HZ                    1.0f
#elif FC_BARO_FILTER_MODE == FC_BARO_FILTER_MODE_STRONG
#define FC_BMP388_IIR_REGISTER_VALUE             0x06U
#define FC_BARO_PRESSURE_LPF_HZ                    0.8f
#define FC_BARO_VELOCITY_LPF_HZ                    0.4f
#else
#error "FC_BARO_FILTER_MODE must be FAST, BALANCED, or STRONG"
#endif

/* --------------------------------------------------------------------------
 * BMP388 + BMI088 相对高度估计
 * -------------------------------------------------------------------------- */

/* 高度估计器切换：0=互补滤波（回退/对比），1=三状态卡尔曼（默认）。 */
#define FC_ALT_ESTIMATOR_MODE_COMPLEMENTARY          0U
#define FC_ALT_ESTIMATOR_MODE_KALMAN                 1U
#ifndef FC_ALT_ESTIMATOR_MODE
#define FC_ALT_ESTIMATOR_MODE FC_ALT_ESTIMATOR_MODE_KALMAN
#endif

/* 功能开关：改成0可分别关闭，便于A/B对比；修改后需完整编译并重新烧录。 */
#ifndef FC_ALT_ENABLE_ACCEL_LPF
#define FC_ALT_ENABLE_ACCEL_LPF                       1U
#endif
#ifndef FC_ALT_ENABLE_BARO_DELAY_COMPENSATION
#define FC_ALT_ENABLE_BARO_DELAY_COMPENSATION         1U
#endif
#ifndef FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING
#define FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING       1U
#endif

/* 起飞零面：50Hz下500点约为10秒；之后仅在STOP/READY继续慢速跟踪。 */
#define FC_BARO_REFERENCE_SAMPLE_COUNT               500U
#define FC_ALT_GROUND_REFERENCE_TIME_CONSTANT_S       10.0f

/* 高度专用加速度低通，不影响姿态环使用的30Hz通用IMU低通。 */
#define FC_ALT_ACCEL_LPF_HZ                             8.0f
/* 世界竖直加速度超过此值直接拒绝该帧，不截幅后继续积分。 */
#define FC_ALT_MAX_TRUSTED_ACCEL_MPS2                   4.0f

/* 气压链路等效延迟：用先验速度前推气压观测；建议每次调0.02秒。 */
#define FC_ALT_BARO_DELAY_S                              0.12f

/* 气压异常保护：单点跳变门限和相对当前估计的创新绝对上限。 */
#define FC_BARO_MAX_SAMPLE_STEP_M                        0.50f
#define FC_BARO_INNOVATION_LIMIT_M                       2.0f

/* 三状态卡尔曼参数：[高度，垂直速度，竖直加速度零偏]。 */
/* 气压高度标准差：越大越不信气压；应按静止日志的标准差设置。 */
#define FC_ALT_KF_BARO_STD_M                          0.35f
/* 加速度过程噪声：越大越允许气压观测修正惯性预测。 */
#define FC_ALT_KF_ACCEL_STD_MPS2                      1.50f
/* 加速度零偏随机游走：越大零偏跟踪越快，也更容易追随气压扰动。 */
#define FC_ALT_KF_ACCEL_BIAS_RW_MPS2_SQRT_S           0.03f
/* 以下三个初始标准差只影响启动后的收敛速度，首轮调试保持默认。 */
#define FC_ALT_KF_INITIAL_HEIGHT_STD_M                 0.50f
#define FC_ALT_KF_INITIAL_VELOCITY_STD_MPS             0.75f
#define FC_ALT_KF_INITIAL_BIAS_STD_MPS2                0.30f
/* 加速度零偏物理限幅，防止滤波器把持续真实运动全部吸收到零偏。 */
#define FC_ALT_KF_MAX_ACCEL_BIAS_MPS2                  1.50f
/* 创新门限=标准差倍数，并受最小门限和绝对上限共同约束。 */
#define FC_ALT_KF_INNOVATION_GATE_SIGMA                4.0f
#define FC_ALT_KF_MIN_INNOVATION_GATE_M                0.75f
/* 协方差数值保护，不作为飞行调参项。 */
#define FC_ALT_KF_MIN_VARIANCE                         0.000001f
#define FC_ALT_KF_MAX_VARIANCE                      1000.0f

/* 仅供互补滤波模式使用。 */
#define FC_VERTICAL_BARO_POSITION_BLEND              0.5f
#define FC_VERTICAL_BARO_VELOCITY_BLEND              0.48f
/* IMU总加速度模长可信范围：0.6g~1.7g，超出时不参与高度预测。 */
#define FC_VERTICAL_ACCEL_MIN_NORM_SQ                 0.36f
#define FC_VERTICAL_ACCEL_MAX_NORM_SQ                 2.89f
#define FC_VERTICAL_MAX_PREDICT_DT_S                  0.02f
/* 气压短时丢失最多依靠惯性桥接0.4秒，超时后退出定高。 */
#define FC_BARO_INERTIAL_HOLD_TIMEOUT_MS             400U
#define FC_GRAVITY_MPS2                          9.80665f

/* 估计结果保护范围；越界时判无效并退出定高，不把状态硬裁到边界。 */
#define FC_ALT_ESTIMATE_MIN_M                         -2.0f
#define FC_ALT_ESTIMATE_MAX_M                         12.0f
#define FC_ALT_MAX_ESTIMATED_VELOCITY_MPS              3.0f

/* 定高控制器参数：确认估计器正常后再调这一组。 */
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

#if FC_ENABLE_APP_CONTROL > 1U
#error "FC_ENABLE_APP_CONTROL must be 0 or 1"
#endif

#if FC_ENABLE_MAG_YAW_FUSION > 1U
#error "FC_ENABLE_MAG_YAW_FUSION must be 0 or 1"
#endif

#if (FC_ALT_ESTIMATOR_MODE != FC_ALT_ESTIMATOR_MODE_COMPLEMENTARY) && \
    (FC_ALT_ESTIMATOR_MODE != FC_ALT_ESTIMATOR_MODE_KALMAN)
#error "FC_ALT_ESTIMATOR_MODE is invalid"
#endif

#if FC_ALT_ENABLE_ACCEL_LPF > 1U
#error "FC_ALT_ENABLE_ACCEL_LPF must be 0 or 1"
#endif

#if FC_ALT_ENABLE_BARO_DELAY_COMPENSATION > 1U
#error "FC_ALT_ENABLE_BARO_DELAY_COMPENSATION must be 0 or 1"
#endif

#if FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING > 1U
#error "FC_ALT_ENABLE_GROUND_REFERENCE_TRACKING must be 0 or 1"
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
