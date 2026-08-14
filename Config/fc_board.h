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
#define FC_IMU_BODY_X_SIGN                   (-1.0f)
#endif
#ifndef FC_IMU_BODY_Y_SIGN
#define FC_IMU_BODY_Y_SIGN                    1.0f
#endif
#ifndef FC_IMU_BODY_Z_SIGN
#define FC_IMU_BODY_Z_SIGN                   (-1.0f)
#endif

#if FC_USE_STM32_HAL
#include "main.h"

/* The purchased BMI088-V1.0 board exposes SCL/SDA only. */
extern I2C_HandleTypeDef hi2c2;
#ifndef FC_BMI088_I2C_HANDLE
#define FC_BMI088_I2C_HANDLE hi2c2
#endif

/*
 * BMP388 and MMC5983MA use two independent software-I2C buses.  These pins
 * are initialized by BSP_SoftI2c_Init(), so CubeMX must leave PB6-PB9 free.
 */
#ifndef FC_BMP388_SOFT_I2C_SCL_GPIO_PORT
#define FC_BMP388_SOFT_I2C_SCL_GPIO_PORT GPIOB
#endif
#ifndef FC_BMP388_SOFT_I2C_SCL_PIN
#define FC_BMP388_SOFT_I2C_SCL_PIN       GPIO_PIN_6
#endif
#ifndef FC_BMP388_SOFT_I2C_SDA_GPIO_PORT
#define FC_BMP388_SOFT_I2C_SDA_GPIO_PORT GPIOB
#endif
#ifndef FC_BMP388_SOFT_I2C_SDA_PIN
#define FC_BMP388_SOFT_I2C_SDA_PIN       GPIO_PIN_7
#endif
#ifndef FC_BMP388_SOFT_I2C_GPIO_CLOCK_ENABLE
#define FC_BMP388_SOFT_I2C_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

#ifndef FC_MMC5983MA_SOFT_I2C_SCL_GPIO_PORT
#define FC_MMC5983MA_SOFT_I2C_SCL_GPIO_PORT GPIOB
#endif
#ifndef FC_MMC5983MA_SOFT_I2C_SCL_PIN
#define FC_MMC5983MA_SOFT_I2C_SCL_PIN       GPIO_PIN_8
#endif
#ifndef FC_MMC5983MA_SOFT_I2C_SDA_GPIO_PORT
#define FC_MMC5983MA_SOFT_I2C_SDA_GPIO_PORT GPIOB
#endif
#ifndef FC_MMC5983MA_SOFT_I2C_SDA_PIN
#define FC_MMC5983MA_SOFT_I2C_SDA_PIN       GPIO_PIN_9
#endif
#ifndef FC_MMC5983MA_SOFT_I2C_GPIO_CLOCK_ENABLE
#define FC_MMC5983MA_SOFT_I2C_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

/* ESC mapping: TIM3 CH1-CH4 on PA6, PA7, PB0, and PB1. */
#ifndef FC_ESC_M1_TIM_HANDLE
#define FC_ESC_M1_TIM_HANDLE  htim3
#endif
#ifndef FC_ESC_M1_TIM_CHANNEL
#define FC_ESC_M1_TIM_CHANNEL TIM_CHANNEL_1
#endif
#ifndef FC_ESC_M2_TIM_HANDLE
#define FC_ESC_M2_TIM_HANDLE  htim3
#endif
#ifndef FC_ESC_M2_TIM_CHANNEL
#define FC_ESC_M2_TIM_CHANNEL TIM_CHANNEL_2
#endif
#ifndef FC_ESC_M3_TIM_HANDLE
#define FC_ESC_M3_TIM_HANDLE  htim3
#endif
#ifndef FC_ESC_M3_TIM_CHANNEL
#define FC_ESC_M3_TIM_CHANNEL TIM_CHANNEL_3
#endif
#ifndef FC_ESC_M4_TIM_HANDLE
#define FC_ESC_M4_TIM_HANDLE  htim3
#endif
#ifndef FC_ESC_M4_TIM_CHANNEL
#define FC_ESC_M4_TIM_CHANNEL TIM_CHANNEL_4
#endif
/* Counter-rate macros default to FC_ESC_TIMER_COUNTER_HZ. */

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
#endif

#define FC_MOTOR_INDEX_M1                     0U
#define FC_MOTOR_INDEX_M2                     1U
#define FC_MOTOR_INDEX_M3                     2U
#define FC_MOTOR_INDEX_M4                     3U

/* Viewed from above, nose forward: M1 front-right, M2 rear-right,
 * M3 rear-left, M4 front-left. Verify this against the real wiring. */
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
