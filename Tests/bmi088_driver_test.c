/* Host test for BMI088 I2C probing, scaling, mapping, calibration, and errors. */

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

I2C_HandleTypeDef hi2c2;

static uint8_t s_accel_registers[256];
static uint8_t s_gyro_registers[256];
static uint8_t s_accel_address = FC_BMI088_ACCEL_I2C_ADDRESS_HIGH;
static uint8_t s_gyro_address = FC_BMI088_GYRO_I2C_ADDRESS_LOW;
static uint32_t s_tick_ms;
static uint32_t s_delay_total_ms;
static bool s_fail_read;
static bool s_fail_write;

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

static uint8_t *registers_for_address(uint16_t address_8bit)
{
    uint8_t address_7bit = (uint8_t)(address_8bit >> 1U);
    if (address_7bit == s_accel_address) { return s_accel_registers; }
    if (address_7bit == s_gyro_address) { return s_gyro_registers; }
    return 0;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle,
                                    uint16_t device_address,
                                    uint16_t memory_address,
                                    uint16_t memory_address_size,
                                    uint8_t *data,
                                    uint16_t length,
                                    uint32_t timeout_ms)
{
    uint8_t *registers = registers_for_address(device_address);
    (void)handle;
    (void)timeout_ms;
    if ((registers == 0) || (data == 0) || (length != 1U) ||
        (memory_address_size != I2C_MEMADD_SIZE_8BIT) || s_fail_write)
    {
        return HAL_ERROR;
    }
    registers[(uint8_t)memory_address] = data[0];
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle,
                                   uint16_t device_address,
                                   uint16_t memory_address,
                                   uint16_t memory_address_size,
                                   uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout_ms)
{
    uint8_t *registers = registers_for_address(device_address);
    uint16_t index;
    (void)handle;
    (void)timeout_ms;
    if ((registers == 0) || (data == 0) || (length == 0U) ||
        (memory_address_size != I2C_MEMADD_SIZE_8BIT) || s_fail_read)
    {
        return HAL_ERROR;
    }
    for (index = 0U; index < length; ++index)
    {
        data[index] = registers[(uint8_t)(memory_address + index)];
        if ((registers == s_gyro_registers) &&
            (((uint8_t)memory_address + (uint8_t)index) == TEST_GYRO_BW_REG))
        {
            /* Real BMI088 parts may return fixed reserved bit 7 as one. */
            data[index] |= 0x80U;
        }
    }
    return HAL_OK;
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
    Bmi088Debug_t debug;
    uint32_t sample;

    if (Drv_Bmi088_Read(0) != FC_STATUS_INVALID_ARGUMENT) { return 1; }
    if (Drv_Bmi088_Read(&imu) != FC_STATUS_NOT_READY || imu.valid) { return 2; }
    prepare_device();

    if (Drv_Bmi088_Init() != FC_STATUS_OK) { return 3; }
    if (!Drv_Bmi088_IsReady()) { return 4; }
    if (Drv_Bmi088_GetChipIds(&ids) != FC_STATUS_OK) { return 5; }
    if ((ids.accel != FC_BMI088_EXPECTED_ACCEL_CHIP_ID) ||
        (ids.gyro != FC_BMI088_EXPECTED_GYRO_CHIP_ID)) { return 6; }
    if (Drv_Bmi088_GetDebug(&debug) != FC_STATUS_OK) { return 7; }
    if ((debug.accel_address_7bit != s_accel_address) ||
        (debug.gyro_address_7bit != s_gyro_address) || !debug.ready) { return 8; }
    if (s_delay_total_ms != (FC_BMI088_STARTUP_DELAY_MS +
                             FC_BMI088_ACCEL_RESET_DELAY_MS +
                             FC_BMI088_GYRO_RESET_DELAY_MS +
                             FC_BMI088_ACCEL_POWER_DELAY_MS)) { return 9; }
    if ((s_accel_registers[TEST_ACCEL_RESET_REG] != 0xB6U) ||
        (s_accel_registers[TEST_ACCEL_PWR_CTRL_REG] != 0x04U) ||
        (s_accel_registers[TEST_ACCEL_PWR_CONF_REG] != 0x00U) ||
        (s_accel_registers[TEST_ACCEL_CONF_REG] != 0xABU) ||
        (s_accel_registers[TEST_ACCEL_RANGE_REG] != 0x01U) ||
        (s_gyro_registers[TEST_GYRO_RESET_REG] != 0xB6U) ||
        (s_gyro_registers[TEST_GYRO_LPM1_REG] != 0x00U) ||
        (s_gyro_registers[TEST_GYRO_RANGE_REG] != 0x00U) ||
        (s_gyro_registers[TEST_GYRO_BW_REG] != 0x02U)) { return 10; }

    s_tick_ms = 100U;
    if (Drv_Bmi088_Read(&imu) != FC_STATUS_OK || !imu.valid || imu.calibrated) { return 11; }
    if (!nearly_equal(imu.accel_g.z, -0.999939f) ||
        !nearly_equal(imu.gyro_dps.x, 1.953125f) ||
        !nearly_equal(imu.gyro_dps.y, 0.9765625f) ||
        !nearly_equal(imu.gyro_dps.z, -0.48828125f)) { return 12; }

    if (Drv_Bmi088_CalibrateGyro() != FC_STATUS_OK) { return 13; }
    for (sample = 0U; sample < FC_IMU_CALIBRATION_SAMPLE_COUNT; ++sample)
    {
        FcStatus_t status = Drv_Bmi088_Read(&imu);
        if ((sample + 1U < FC_IMU_CALIBRATION_SAMPLE_COUNT) &&
            ((status != FC_STATUS_BUSY) || imu.valid)) { return 14; }
        if ((sample + 1U == FC_IMU_CALIBRATION_SAMPLE_COUNT) &&
            ((status != FC_STATUS_OK) || !imu.valid || !imu.calibrated)) { return 15; }
    }
    if (Drv_Bmi088_GetGyroBias(&bias) != FC_STATUS_OK) { return 16; }
    if (!nearly_equal(bias.x, 1.953125f) ||
        !nearly_equal(bias.y, 0.9765625f) ||
        !nearly_equal(bias.z, -0.48828125f)) { return 17; }

    set_i16(s_gyro_registers, TEST_GYRO_DATA_REG + 2U, -48);
    if (Drv_Bmi088_SetBiasTrackingEnabled(true) != FC_STATUS_OK) { return 18; }
    for (sample = 0U; sample < 1000U; ++sample)
    {
        if (Drv_Bmi088_Read(&imu) != FC_STATUS_OK) { return 19; }
    }
    if (Drv_Bmi088_GetGyroBias(&bias) != FC_STATUS_OK ||
        (bias.x <= 1.953125f) || (bias.x >= 2.9296875f)) { return 20; }

    s_fail_read = true;
    if (Drv_Bmi088_Read(&imu) != FC_STATUS_ERROR || imu.valid) { return 21; }
    s_fail_read = false;
    if (Drv_Bmi088_GetDebug(&debug) != FC_STATUS_OK ||
        (debug.failed_read_count == 0U)) { return 22; }

    prepare_device();
    s_accel_registers[TEST_ACCEL_CHIP_ID_REG] = 0U;
    if (Drv_Bmi088_Init() != FC_STATUS_NOT_READY || Drv_Bmi088_IsReady()) { return 23; }

    prepare_device();
    s_fail_write = true;
    if (Drv_Bmi088_Init() != FC_STATUS_ERROR || Drv_Bmi088_IsReady()) { return 24; }
    return 0;
}
