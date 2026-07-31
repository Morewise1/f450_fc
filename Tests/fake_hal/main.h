#ifndef TEST_FAKE_HAL_MAIN_H
#define TEST_FAKE_HAL_MAIN_H

#include <stdint.h>

typedef struct { uint32_t instance; } SPI_HandleTypeDef;
typedef struct { uint32_t instance; } I2C_HandleTypeDef;
typedef struct { uint32_t instance; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } GPIO_TypeDef;

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
#define I2C_MEMADD_SIZE_8BIT 1U

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern GPIO_TypeDef g_fake_qmi_cs_port;
extern GPIO_TypeDef g_fake_bmi_accel_cs_port;
extern GPIO_TypeDef g_fake_bmi_gyro_cs_port;

#define FC_QMI8658_CS_GPIO_PORT (&g_fake_qmi_cs_port)
#define FC_QMI8658_CS_PIN       ((uint16_t)0x0010U)

#define FC_BMI088_ACCEL_CS_GPIO_PORT (&g_fake_bmi_accel_cs_port)
#define FC_BMI088_ACCEL_CS_PIN       ((uint16_t)0x0020U)
#define FC_BMI088_GYRO_CS_GPIO_PORT  (&g_fake_bmi_gyro_cs_port)
#define FC_BMI088_GYRO_CS_PIN        ((uint16_t)0x0040U)

#define FC_ESC_M1_TIM_HANDLE  htim3
#define FC_ESC_M1_TIM_CHANNEL TIM_CHANNEL_1
#define FC_ESC_M2_TIM_HANDLE  htim3
#define FC_ESC_M2_TIM_CHANNEL TIM_CHANNEL_2
#define FC_ESC_M3_TIM_HANDLE  htim4
#define FC_ESC_M3_TIM_CHANNEL TIM_CHANNEL_1
#define FC_ESC_M4_TIM_HANDLE  htim4
#define FC_ESC_M4_TIM_CHANNEL TIM_CHANNEL_2

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle,
                                   uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout_ms);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *handle,
                                  uint8_t *data,
                                  uint16_t length,
                                  uint32_t timeout_ms);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle,
                                    uint16_t device_address,
                                    uint16_t memory_address,
                                    uint16_t memory_address_size,
                                    uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout_ms);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle,
                                   uint16_t device_address,
                                   uint16_t memory_address,
                                   uint16_t memory_address_size,
                                   uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout_ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *handle, uint32_t channel);
void FakeHalTimSetCompare(TIM_HandleTypeDef *handle, uint32_t channel, uint32_t compare);
#define __HAL_TIM_SET_COMPARE(handle, channel, compare) FakeHalTimSetCompare((handle), (channel), (compare))

#endif /* TEST_FAKE_HAL_MAIN_H */
