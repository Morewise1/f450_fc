/* Host test for BMI088 dual-CS SPI access, scaling, mapping, and calibration. */

#include <stdbool.h>
#include <stdint.h>
#include "drv_bmi088.h"
#include "fc_config.h"
#include "main.h"

#define TEST_ACCEL_CHIP_ID_REG   0x00U
#define TEST_ACCEL_DATA_REG      0x12U
#define TEST_ACCEL_TEMP_REG      0x22U
#define TEST_ACCEL_CONF_REG      0x40U
#define TEST_ACCEL_RANGE_REG     0x41U
#define TEST_ACCEL_PWR_CONF_REG  0x7CU
#define TEST_ACCEL_PWR_CTRL_REG  0x7DU
#define TEST_ACCEL_RESET_REG     0x7EU

#define TEST_GYRO_CHIP_ID_REG    0x00U
#define TEST_GYRO_DATA_REG       0x02U
#define TEST_GYRO_RANGE_REG      0x0FU
#define TEST_GYRO_BW_REG         0x10U
#define TEST_GYRO_LPM1_REG       0x11U
#define TEST_GYRO_RESET_REG      0x14U

typedef enum
{
    TEST_DEVICE_NONE = 0,
    TEST_DEVICE_ACCEL,
    TEST_DEVICE_GYRO
} TestDevice_t;

SPI_HandleTypeDef hspi1;
GPIO_TypeDef g_fake_qmi_cs_port;
GPIO_TypeDef g_fake_bmi_accel_cs_port;
GPIO_TypeDef g_fake_bmi_gyro_cs_port;

static uint8_t s_accel_registers[256];
static uint8_t s_gyro_registers[256];
static TestDevice_t s_active_device;
static uint8_t s_read_register;
static bool s_accel_dummy_pending;
static uint32_t s_tick_ms;
static uint32_t s_delay_total_ms;
static bool s_fail_transmit;
static bool s_fail_receive;

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool nearly_equal(float actual, float expected)
{
    return absolute_float(actual - expected) < 0.001f;
}

static void set_i16(uint8_t *registers, uint8_t reg, int16_t value)
{
    registers[reg] = (uint8_t)((uint16_t)value & 0xFFU);
    registers[reg + 1U] = (uint8_t)((uint16_t)value >> 8U);
}

static void prepare_stationary_sample(void)
{
    set_i16(s_accel_registers, TEST_ACCEL_DATA_REG, 0);
    set_i16(s_accel_registers, TEST_ACCEL_DATA_REG + 2U, 0);
    set_i16(s_accel_registers, TEST_ACCEL_DATA_REG + 4U, 5461);
    set_i16(s_gyro_registers, TEST_GYRO_DATA_REG, 16);
    set_i16(s_gyro_registers, TEST_GYRO_DATA_REG + 2U, -32);
    set_i16(s_gyro_registers, TEST_GYRO_DATA_REG + 4U, 8);
    s_accel_registers[TEST_ACCEL_TEMP_REG] = 2U;
    s_accel_registers[TEST_ACCEL_TEMP_REG + 1U] = 0U;
}

static void prepare_moving_sample(void)
{
    prepare_stationary_sample();
    set_i16(s_gyro_registers, TEST_GYRO_DATA_REG, 100);
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle,
                                   uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout_ms)
{
    uint8_t *registers;
    (void)handle;
    (void)timeout_ms;

    if ((s_active_device == TEST_DEVICE_NONE) || (data == 0) || s_fail_transmit)
    {
        return HAL_ERROR;
    }
    registers = (s_active_device == TEST_DEVICE_ACCEL) ?
                    s_accel_registers : s_gyro_registers;

    if (length == 1U)
    {
        if ((data[0] & 0x80U) == 0U) { return HAL_ERROR; }
        s_read_register = (uint8_t)(data[0] & 0x7FU);
        s_accel_dummy_pending = s_active_device == TEST_DEVICE_ACCEL;
        return HAL_OK;
    }
    if (length == 2U)
    {
        if ((data[0] & 0x80U) != 0U) { return HAL_ERROR; }
        registers[data[0] & 0x7FU] = data[1];
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *handle,
                                  uint8_t *data,
                                  uint16_t length,
                                  uint32_t timeout_ms)
{
    uint8_t *registers;
    uint16_t index;
    (void)handle;
    (void)timeout_ms;

    if ((s_active_device == TEST_DEVICE_NONE) || (data == 0) || s_fail_receive)
    {
        return HAL_ERROR;
    }
    if (s_accel_dummy_pending)
    {
        if (length != 1U) { return HAL_ERROR; }
        data[0] = 0U;
        s_accel_dummy_pending = false;
        return HAL_OK;
    }

    registers = (s_active_device == TEST_DEVICE_ACCEL) ?
                    s_accel_registers : s_gyro_registers;
    for (index = 0U; index < length; ++index)
    {
        data[index] = registers[s_read_register++];
    }
    return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)pin;
    if (state == GPIO_PIN_RESET)
    {
        if (port == &g_fake_bmi_accel_cs_port) { s_active_device = TEST_DEVICE_ACCEL; }
        if (port == &g_fake_bmi_gyro_cs_port) { s_active_device = TEST_DEVICE_GYRO; }
    }
    else if (((port == &g_fake_bmi_accel_cs_port) && (s_active_device == TEST_DEVICE_ACCEL)) ||
             ((port == &g_fake_bmi_gyro_cs_port) && (s_active_device == TEST_DEVICE_GYRO)))
    {
        s_active_device = TEST_DEVICE_NONE;
    }
}

uint32_t HAL_GetTick(void)
{
    return s_tick_ms;
}

void HAL_Delay(uint32_t delay_ms)
{
    s_delay_total_ms += delay_ms;
    s_tick_ms += delay_ms;
}

static void prepare_device(void)
{
    uint32_t index;
    for (index = 0U; index < 256U; ++index)
    {
        s_accel_registers[index] = 0U;
        s_gyro_registers[index] = 0U;
    }
    s_accel_registers[TEST_ACCEL_CHIP_ID_REG] = FC_BMI088_EXPECTED_ACCEL_CHIP_ID;
    s_gyro_registers[TEST_GYRO_CHIP_ID_REG] = FC_BMI088_EXPECTED_GYRO_CHIP_ID;
    prepare_stationary_sample();
}

int main(void)
{
    FcImuData_t imu = {0};
    FcVector3f_t bias;
    Bmi088ChipIds_t ids;
    uint32_t sample;

    if (Drv_Bmi088_Read(0) != FC_STATUS_INVALID_ARGUMENT) { return 1; }
    if (Drv_Bmi088_Read(&imu) != FC_STATUS_NOT_READY || imu.valid) { return 2; }
    if (Drv_Bmi088_SetBiasTrackingEnabled(true) != FC_STATUS_NOT_READY) { return 34; }
    prepare_device();

    if (Drv_Bmi088_Init() != FC_STATUS_OK) { return 3; }
    if (!Drv_Bmi088_IsReady()) { return 4; }
    if (Drv_Bmi088_GetChipIds(&ids) != FC_STATUS_OK) { return 5; }
    if ((ids.accel != FC_BMI088_EXPECTED_ACCEL_CHIP_ID) ||
        (ids.gyro != FC_BMI088_EXPECTED_GYRO_CHIP_ID)) { return 6; }
    if (s_active_device != TEST_DEVICE_NONE) { return 7; }
    if (s_delay_total_ms != (FC_BMI088_STARTUP_DELAY_MS +
                             FC_BMI088_ACCEL_RESET_DELAY_MS +
                             FC_BMI088_GYRO_RESET_DELAY_MS +
                             FC_BMI088_ACCEL_POWER_DELAY_MS)) { return 8; }
    if ((s_accel_registers[TEST_ACCEL_RESET_REG] != 0xB6U) ||
        (s_accel_registers[TEST_ACCEL_PWR_CTRL_REG] != 0x04U) ||
        (s_accel_registers[TEST_ACCEL_PWR_CONF_REG] != 0x00U) ||
        (s_accel_registers[TEST_ACCEL_CONF_REG] != 0xABU) ||
        (s_accel_registers[TEST_ACCEL_RANGE_REG] != 0x01U) ||
        (s_gyro_registers[TEST_GYRO_RESET_REG] != 0xB6U) ||
        (s_gyro_registers[TEST_GYRO_LPM1_REG] != 0x00U) ||
        (s_gyro_registers[TEST_GYRO_RANGE_REG] != 0x00U) ||
        (s_gyro_registers[TEST_GYRO_BW_REG] != 0x02U)) { return 9; }

    s_tick_ms = 100U;
    if (Drv_Bmi088_Read(&imu) != FC_STATUS_OK) { return 10; }
    if (!imu.valid || imu.calibrated || (imu.timestamp_ms != 100U)) { return 11; }
    if (!nearly_equal(imu.accel_g.x, 0.0f) ||
        !nearly_equal(imu.accel_g.y, 0.0f) ||
        !nearly_equal(imu.accel_g.z, -0.999939f)) { return 12; }
    if (!nearly_equal(imu.gyro_dps.x, 1.953125f) ||
        !nearly_equal(imu.gyro_dps.y, 0.9765625f) ||
        !nearly_equal(imu.gyro_dps.z, -0.48828125f)) { return 13; }
    if ((imu.accel_raw[2] != -5461) ||
        (imu.gyro_raw[0] != 32) ||
        (imu.gyro_raw[1] != 16) ||
        (imu.gyro_raw[2] != -8)) { return 14; }
    if (!nearly_equal(imu.temperature_c, 25.0f)) { return 15; }

    if (Drv_Bmi088_CalibrateGyro() != FC_STATUS_OK) { return 16; }
    if ((Drv_Bmi088_Read(&imu) != FC_STATUS_BUSY) || imu.valid) { return 17; }
    prepare_moving_sample();
    if ((Drv_Bmi088_Read(&imu) != FC_STATUS_BUSY) || imu.valid) { return 18; }
    prepare_stationary_sample();

    for (sample = 0U; sample < FC_IMU_CALIBRATION_SAMPLE_COUNT; ++sample)
    {
        FcStatus_t status = Drv_Bmi088_Read(&imu);
        if ((sample + 1U) < FC_IMU_CALIBRATION_SAMPLE_COUNT)
        {
            if ((status != FC_STATUS_BUSY) || imu.valid) { return 19; }
        }
        else if ((status != FC_STATUS_OK) || !imu.valid || !imu.calibrated)
        {
            return 20;
        }
    }

    if (!Drv_Bmi088_IsCalibrationComplete()) { return 21; }
    if (Drv_Bmi088_GetGyroBias(&bias) != FC_STATUS_OK) { return 22; }
    if (!nearly_equal(bias.x, 1.953125f) ||
        !nearly_equal(bias.y, 0.9765625f) ||
        !nearly_equal(bias.z, -0.48828125f)) { return 23; }
    if (!nearly_equal(imu.gyro_dps.x, 0.0f) ||
        !nearly_equal(imu.gyro_dps.y, 0.0f) ||
        !nearly_equal(imu.gyro_dps.z, 0.0f)) { return 24; }

    /* A small stationary offset may track only when the App explicitly enables it. */
    set_i16(s_gyro_registers, TEST_GYRO_DATA_REG + 2U, -48);
    if (Drv_Bmi088_SetBiasTrackingEnabled(true) != FC_STATUS_OK) { return 35; }
    for (sample = 0U; sample < 1000U; ++sample)
    {
        if (Drv_Bmi088_Read(&imu) != FC_STATUS_OK) { return 36; }
    }
    if (Drv_Bmi088_GetGyroBias(&bias) != FC_STATUS_OK) { return 37; }
    if ((bias.x <= 1.953125f) || (bias.x >= 2.9296875f) ||
        (imu.gyro_dps.x <= 0.0f) || (imu.gyro_dps.x >= 0.9765625f)) { return 38; }

    if (Drv_Bmi088_SetBiasTrackingEnabled(false) != FC_STATUS_OK) { return 39; }
    {
        FcVector3f_t held_bias = bias;
        set_i16(s_gyro_registers, TEST_GYRO_DATA_REG + 2U, -64);
        if (Drv_Bmi088_Read(&imu) != FC_STATUS_OK) { return 40; }
        if (Drv_Bmi088_GetGyroBias(&bias) != FC_STATUS_OK ||
            !nearly_equal(bias.x, held_bias.x)) { return 41; }
    }

    s_tick_ms = 200U;
    if (Drv_Bmi088_Read(&imu) != FC_STATUS_OK) { return 25; }
    if (!Drv_Bmi088_IsDataValid(200U)) { return 26; }
    if (Drv_Bmi088_IsDataValid(221U)) { return 27; }

    s_fail_receive = true;
    if (Drv_Bmi088_Read(&imu) != FC_STATUS_ERROR) { return 28; }
    if (imu.valid || Drv_Bmi088_IsDataValid(200U) ||
        (s_active_device != TEST_DEVICE_NONE)) { return 29; }
    s_fail_receive = false;

    prepare_device();
    s_accel_registers[TEST_ACCEL_CHIP_ID_REG] = 0U;
    if (Drv_Bmi088_Init() != FC_STATUS_INVALID_DATA) { return 30; }
    if (Drv_Bmi088_IsReady()) { return 31; }

    prepare_device();
    s_fail_transmit = true;
    if (Drv_Bmi088_Init() != FC_STATUS_ERROR) { return 32; }
    if (Drv_Bmi088_IsReady() || (s_active_device != TEST_DEVICE_NONE)) { return 33; }
    return 0;
}
