/* Host test for QMI8658C using fake HAL and non-trivial body-axis mapping. */

#include <stdbool.h>
#include <stdint.h>
#include "drv_qmi8658.h"
#include "fc_config.h"
#include "main.h"

#define TEST_REG_WHO_AM_I  0x00U
#define TEST_REG_CTRL1     0x02U
#define TEST_REG_CTRL2     0x03U
#define TEST_REG_CTRL3     0x04U
#define TEST_REG_CTRL5     0x06U
#define TEST_REG_CTRL7     0x08U
#define TEST_REG_TEMP_L    0x33U
#define TEST_REG_RESET     0x60U

SPI_HandleTypeDef hspi1;
GPIO_TypeDef g_fake_qmi_cs_port;

static uint8_t s_registers[128];
static uint8_t s_read_register;
static uint32_t s_tick_ms;
static uint32_t s_delay_total_ms;
static bool s_cs_active;
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

static void set_i16(uint8_t reg, int16_t value)
{
    s_registers[reg] = (uint8_t)((uint16_t)value & 0xFFU);
    s_registers[reg + 1U] = (uint8_t)((uint16_t)value >> 8U);
}

static void prepare_stationary_sample(void)
{
    set_i16(TEST_REG_TEMP_L, (int16_t)(25 * 256));
    set_i16(TEST_REG_TEMP_L + 2U, 0);
    set_i16(TEST_REG_TEMP_L + 4U, 0);
    set_i16(TEST_REG_TEMP_L + 6U, 4096);
    set_i16(TEST_REG_TEMP_L + 8U, 16);
    set_i16(TEST_REG_TEMP_L + 10U, -32);
    set_i16(TEST_REG_TEMP_L + 12U, 8);
}

static void prepare_moving_sample(void)
{
    prepare_stationary_sample();
    set_i16(TEST_REG_TEMP_L + 8U, 64);
}

static void prepare_stuck_bus_sample(uint8_t value)
{
    uint32_t index;
    for (index = 0U; index < 14U; ++index)
    {
        s_registers[TEST_REG_TEMP_L + index] = value;
    }
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle,
                                   uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout_ms)
{
    (void)handle;
    (void)timeout_ms;

    if (!s_cs_active || (data == 0) || s_fail_transmit)
    {
        return HAL_ERROR;
    }
    if (length == 1U)
    {
        if ((data[0] & 0x80U) == 0U) { return HAL_ERROR; }
        s_read_register = (uint8_t)(data[0] & 0x7FU);
        return HAL_OK;
    }
    if (length == 2U)
    {
        if ((data[0] & 0x80U) != 0U) { return HAL_ERROR; }
        s_registers[data[0] & 0x7FU] = data[1];
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *handle,
                                  uint8_t *data,
                                  uint16_t length,
                                  uint32_t timeout_ms)
{
    uint16_t index;
    (void)handle;
    (void)timeout_ms;

    if (!s_cs_active || (data == 0) || s_fail_receive)
    {
        return HAL_ERROR;
    }
    for (index = 0U; index < length; ++index)
    {
        data[index] = s_registers[s_read_register++];
    }
    return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port;
    (void)pin;
    s_cs_active = state == GPIO_PIN_RESET;
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

int main(void)
{
    FcImuData_t imu = {0};
    FcVector3f_t bias;
    uint8_t who_am_i = 0U;
    uint32_t sample;

    if (Drv_Qmi8658_Read(0) != FC_STATUS_INVALID_ARGUMENT) { return 1; }
    if (Drv_Qmi8658_Read(&imu) != FC_STATUS_NOT_READY || imu.valid) { return 2; }

    s_registers[TEST_REG_WHO_AM_I] = FC_QMI8658_EXPECTED_WHO_AM_I;
    prepare_stationary_sample();

    if (Drv_Qmi8658_Init() != FC_STATUS_OK) { return 3; }
    if (!Drv_Qmi8658_IsReady()) { return 4; }
    if (Drv_Qmi8658_GetWhoAmI() != FC_QMI8658_EXPECTED_WHO_AM_I) { return 5; }
    if (s_cs_active) { return 6; }
    if (s_delay_total_ms != FC_QMI8658_RESET_DELAY_MS) { return 7; }
    if (s_registers[TEST_REG_RESET] != 0xB0U) { return 8; }
    if ((s_registers[TEST_REG_CTRL1] != 0x60U) ||
        (s_registers[TEST_REG_CTRL2] != 0x23U) ||
        (s_registers[TEST_REG_CTRL3] != 0x73U) ||
        (s_registers[TEST_REG_CTRL5] != 0x11U) ||
        (s_registers[TEST_REG_CTRL7] != 0x03U)) { return 9; }
    if (Drv_Qmi8658_ReadWhoAmI(&who_am_i) != FC_STATUS_OK) { return 10; }
    if (who_am_i != FC_QMI8658_EXPECTED_WHO_AM_I) { return 11; }

    s_tick_ms = 20U;
    if (Drv_Qmi8658_Read(&imu) != FC_STATUS_OK) { return 12; }
    if (!imu.valid || imu.calibrated || (imu.timestamp_ms != 20U)) { return 13; }
    if (!nearly_equal(imu.accel_g.x, 0.0f) ||
        !nearly_equal(imu.accel_g.y, 0.0f) ||
        !nearly_equal(imu.accel_g.z, -1.0f)) { return 14; }
    if (!nearly_equal(imu.gyro_dps.x, 2.0f) ||
        !nearly_equal(imu.gyro_dps.y, 1.0f) ||
        !nearly_equal(imu.gyro_dps.z, -0.5f)) { return 15; }
    if ((imu.accel_raw[2] != -4096) ||
        (imu.gyro_raw[0] != 32) ||
        (imu.gyro_raw[1] != 16) ||
        (imu.gyro_raw[2] != -8)) { return 16; }
    if (!nearly_equal(imu.temperature_c, 25.0f)) { return 17; }

    if (Drv_Qmi8658_CalibrateGyro() != FC_STATUS_OK) { return 18; }
    if ((Drv_Qmi8658_Read(&imu) != FC_STATUS_BUSY) || imu.valid) { return 19; }
    prepare_moving_sample();
    if ((Drv_Qmi8658_Read(&imu) != FC_STATUS_BUSY) || imu.valid) { return 20; }
    prepare_stationary_sample();

    for (sample = 0U; sample < FC_IMU_CALIBRATION_SAMPLE_COUNT; ++sample)
    {
        FcStatus_t status = Drv_Qmi8658_Read(&imu);
        if ((sample + 1U) < FC_IMU_CALIBRATION_SAMPLE_COUNT)
        {
            if ((status != FC_STATUS_BUSY) || imu.valid) { return 21; }
        }
        else if ((status != FC_STATUS_OK) || !imu.valid || !imu.calibrated)
        {
            return 22;
        }
    }

    if (!Drv_Qmi8658_IsCalibrationComplete()) { return 23; }
    if (Drv_Qmi8658_GetGyroBias(&bias) != FC_STATUS_OK) { return 24; }
    if (!nearly_equal(bias.x, 2.0f) ||
        !nearly_equal(bias.y, 1.0f) ||
        !nearly_equal(bias.z, -0.5f)) { return 25; }
    if (!nearly_equal(imu.gyro_dps.x, 0.0f) ||
        !nearly_equal(imu.gyro_dps.y, 0.0f) ||
        !nearly_equal(imu.gyro_dps.z, 0.0f)) { return 26; }

    s_tick_ms = 30U;
    if (Drv_Qmi8658_Read(&imu) != FC_STATUS_OK) { return 27; }
    if (!Drv_Qmi8658_IsDataValid(30U)) { return 28; }
    if (Drv_Qmi8658_IsDataValid(51U)) { return 29; }

    s_fail_receive = true;
    if (Drv_Qmi8658_Read(&imu) != FC_STATUS_ERROR) { return 30; }
    if (imu.valid || Drv_Qmi8658_IsDataValid(30U) || s_cs_active) { return 31; }
    s_fail_receive = false;

    prepare_stuck_bus_sample(0xFFU);
    if (Drv_Qmi8658_Read(&imu) != FC_STATUS_INVALID_DATA || imu.valid) { return 32; }
    prepare_stuck_bus_sample(0x00U);
    if (Drv_Qmi8658_Read(&imu) != FC_STATUS_INVALID_DATA || imu.valid) { return 33; }
    prepare_stationary_sample();

    s_registers[TEST_REG_WHO_AM_I] = 0x00U;
    if (Drv_Qmi8658_Init() != FC_STATUS_INVALID_DATA) { return 34; }
    if (Drv_Qmi8658_IsReady()) { return 35; }

    s_registers[TEST_REG_WHO_AM_I] = FC_QMI8658_EXPECTED_WHO_AM_I;
    s_fail_transmit = true;
    if (Drv_Qmi8658_Init() != FC_STATUS_ERROR) { return 36; }
    if (Drv_Qmi8658_IsReady() || s_cs_active) { return 37; }
    return 0;
}
