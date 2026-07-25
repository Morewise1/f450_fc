#ifndef FC_BOARD_H
#define FC_BOARD_H

/* Central sensor orientation, HAL pin/channel binding, and motor layout. */

#include "fc_config.h"

#define FC_AXIS_SOURCE_X                    0U
#define FC_AXIS_SOURCE_Y                    1U
#define FC_AXIS_SOURCE_Z                    2U

/* Sensor-axis to body-axis mapping. Body frame: X forward, Y right, Z down. */
#ifndef FC_IMU_BODY_X_SOURCE
#define FC_IMU_BODY_X_SOURCE      FC_AXIS_SOURCE_X
#endif
#ifndef FC_IMU_BODY_Y_SOURCE
#define FC_IMU_BODY_Y_SOURCE      FC_AXIS_SOURCE_Y
#endif
#ifndef FC_IMU_BODY_Z_SOURCE
#define FC_IMU_BODY_Z_SOURCE      FC_AXIS_SOURCE_Z
#endif
#ifndef FC_IMU_BODY_X_SIGN
#define FC_IMU_BODY_X_SIGN                    1.0f
#endif
#ifndef FC_IMU_BODY_Y_SIGN
#define FC_IMU_BODY_Y_SIGN                    1.0f
#endif
#ifndef FC_IMU_BODY_Z_SIGN
#define FC_IMU_BODY_Z_SIGN                    1.0f
#endif

#if FC_USE_STM32_HAL
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/*
 * BMI088 contains separate accelerometer and gyroscope dies. They share
 * SPI1 SCK/MISO/MOSI, but each die requires its own active-low CS pin.
 * The defaults below match CubeMX labels BMI088_ACC_CS and BMI088_GYRO_CS.
 */
#ifndef FC_BMI088_ACCEL_CS_GPIO_PORT
#if defined(BMI088_ACC_CS_GPIO_Port) && defined(BMI088_ACC_CS_Pin)
#define FC_BMI088_ACCEL_CS_GPIO_PORT BMI088_ACC_CS_GPIO_Port
#define FC_BMI088_ACCEL_CS_PIN       BMI088_ACC_CS_Pin
#else
#error "Create CubeMX GPIO label BMI088_ACC_CS or define BMI088 accel CS bindings"
#endif
#endif
#ifndef FC_BMI088_ACCEL_CS_PIN
#error "Define FC_BMI088_ACCEL_CS_PIN for the actual board"
#endif

#ifndef FC_BMI088_GYRO_CS_GPIO_PORT
#if defined(BMI088_GYRO_CS_GPIO_Port) && defined(BMI088_GYRO_CS_Pin)
#define FC_BMI088_GYRO_CS_GPIO_PORT BMI088_GYRO_CS_GPIO_Port
#define FC_BMI088_GYRO_CS_PIN       BMI088_GYRO_CS_Pin
#else
#error "Create CubeMX GPIO label BMI088_GYRO_CS or define BMI088 gyro CS bindings"
#endif
#endif
#ifndef FC_BMI088_GYRO_CS_PIN
#error "Define FC_BMI088_GYRO_CS_PIN for the actual board"
#endif

#define FC_BMI088_ACCEL_CS_LOW()  HAL_GPIO_WritePin(FC_BMI088_ACCEL_CS_GPIO_PORT, FC_BMI088_ACCEL_CS_PIN, GPIO_PIN_RESET)
#define FC_BMI088_ACCEL_CS_HIGH() HAL_GPIO_WritePin(FC_BMI088_ACCEL_CS_GPIO_PORT, FC_BMI088_ACCEL_CS_PIN, GPIO_PIN_SET)
#define FC_BMI088_GYRO_CS_LOW()   HAL_GPIO_WritePin(FC_BMI088_GYRO_CS_GPIO_PORT, FC_BMI088_GYRO_CS_PIN, GPIO_PIN_RESET)
#define FC_BMI088_GYRO_CS_HIGH()  HAL_GPIO_WritePin(FC_BMI088_GYRO_CS_GPIO_PORT, FC_BMI088_GYRO_CS_PIN, GPIO_PIN_SET)

/*
 * ESC mapping example for one timer:
 *
 * #define FC_ESC_M1_TIM_HANDLE  htim3
 * #define FC_ESC_M1_TIM_CHANNEL TIM_CHANNEL_1
 * #define FC_ESC_M2_TIM_HANDLE  htim3
 * #define FC_ESC_M2_TIM_CHANNEL TIM_CHANNEL_2
 * #define FC_ESC_M3_TIM_HANDLE  htim3
 * #define FC_ESC_M3_TIM_CHANNEL TIM_CHANNEL_3
 * #define FC_ESC_M4_TIM_HANDLE  htim3
 * #define FC_ESC_M4_TIM_CHANNEL TIM_CHANNEL_4
 *
 * Counter-rate macros default to FC_ESC_TIMER_COUNTER_HZ. Define a per-motor
 * value when multiple timers use different prescalers.
 */
#if FC_BOARD_HAL_BINDINGS_COMPLETE
#ifndef FC_ESC_M1_TIM_HANDLE
#error "Define FC_ESC_M1_TIM_HANDLE for the actual board"
#endif
#ifndef FC_ESC_M1_TIM_CHANNEL
#error "Define FC_ESC_M1_TIM_CHANNEL for the actual board"
#endif
#ifndef FC_ESC_M2_TIM_HANDLE
#error "Define FC_ESC_M2_TIM_HANDLE for the actual board"
#endif
#ifndef FC_ESC_M2_TIM_CHANNEL
#error "Define FC_ESC_M2_TIM_CHANNEL for the actual board"
#endif
#ifndef FC_ESC_M3_TIM_HANDLE
#error "Define FC_ESC_M3_TIM_HANDLE for the actual board"
#endif
#ifndef FC_ESC_M3_TIM_CHANNEL
#error "Define FC_ESC_M3_TIM_CHANNEL for the actual board"
#endif
#ifndef FC_ESC_M4_TIM_HANDLE
#error "Define FC_ESC_M4_TIM_HANDLE for the actual board"
#endif
#ifndef FC_ESC_M4_TIM_CHANNEL
#error "Define FC_ESC_M4_TIM_CHANNEL for the actual board"
#endif

#ifndef FC_ESC_M1_COUNTER_HZ
#define FC_ESC_M1_COUNTER_HZ FC_ESC_TIMER_COUNTER_HZ
#endif
#ifndef FC_ESC_M2_COUNTER_HZ
#define FC_ESC_M2_COUNTER_HZ FC_ESC_TIMER_COUNTER_HZ
#endif
#ifndef FC_ESC_M3_COUNTER_HZ
#define FC_ESC_M3_COUNTER_HZ FC_ESC_TIMER_COUNTER_HZ
#endif
#ifndef FC_ESC_M4_COUNTER_HZ
#define FC_ESC_M4_COUNTER_HZ FC_ESC_TIMER_COUNTER_HZ
#endif

extern TIM_HandleTypeDef FC_ESC_M1_TIM_HANDLE;
extern TIM_HandleTypeDef FC_ESC_M2_TIM_HANDLE;
extern TIM_HandleTypeDef FC_ESC_M3_TIM_HANDLE;
extern TIM_HandleTypeDef FC_ESC_M4_TIM_HANDLE;
#endif /* FC_BOARD_HAL_BINDINGS_COMPLETE */
#else
#define FC_BMI088_ACCEL_CS_LOW()  ((void)0)
#define FC_BMI088_ACCEL_CS_HIGH() ((void)0)
#define FC_BMI088_GYRO_CS_LOW()   ((void)0)
#define FC_BMI088_GYRO_CS_HIGH()  ((void)0)
#endif

#define FC_MOTOR_INDEX_M1                     0U
#define FC_MOTOR_INDEX_M2                     1U
#define FC_MOTOR_INDEX_M3                     2U
#define FC_MOTOR_INDEX_M4                     3U

#define FC_MOTOR_SPIN_CW                       0U
#define FC_MOTOR_SPIN_CCW                      1U
#define FC_MOTOR_M1_SPIN             FC_MOTOR_SPIN_CCW
#define FC_MOTOR_M2_SPIN             FC_MOTOR_SPIN_CW
#define FC_MOTOR_M3_SPIN             FC_MOTOR_SPIN_CCW
#define FC_MOTOR_M4_SPIN             FC_MOTOR_SPIN_CW

#define FC_MIX_M1_ROLL                       (-1.0f)
#define FC_MIX_M1_PITCH                        1.0f
#define FC_MIX_M1_YAW                          1.0f
#define FC_MIX_M2_ROLL                       (-1.0f)
#define FC_MIX_M2_PITCH                      (-1.0f)
#define FC_MIX_M2_YAW                        (-1.0f)
#define FC_MIX_M3_ROLL                         1.0f
#define FC_MIX_M3_PITCH                      (-1.0f)
#define FC_MIX_M3_YAW                          1.0f
#define FC_MIX_M4_ROLL                         1.0f
#define FC_MIX_M4_PITCH                        1.0f
#define FC_MIX_M4_YAW                        (-1.0f)

#endif /* FC_BOARD_H */
