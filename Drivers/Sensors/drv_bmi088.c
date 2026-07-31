/* BMI088 I2C driver with address probing, fail-closed reads, and RAM bias. */

#include <stddef.h>
#include "drv_bmi088.h"
#include "fc_board.h"
#include "fc_config.h"

#define BMI088_SOFTRESET_VALUE               0xB6U

#define BMI088_ACCEL_REG_CHIP_ID             0x00U
#define BMI088_ACCEL_REG_DATA_X_LSB          0x12U
#define BMI088_ACCEL_REG_TEMP_MSB            0x22U
#define BMI088_ACCEL_REG_CONF                0x40U
#define BMI088_ACCEL_REG_RANGE               0x41U
#define BMI088_ACCEL_REG_PWR_CONF            0x7CU
#define BMI088_ACCEL_REG_PWR_CTRL            0x7DU
#define BMI088_ACCEL_REG_SOFTRESET           0x7EU

#define BMI088_GYRO_REG_CHIP_ID              0x00U
#define BMI088_GYRO_REG_DATA_X_LSB           0x02U
#define BMI088_GYRO_REG_RANGE                0x0FU
#define BMI088_GYRO_REG_BANDWIDTH            0x10U
#define BMI088_GYRO_REG_LPM1                 0x11U
#define BMI088_GYRO_REG_SOFTRESET            0x14U

/* Accel: normal filter, 800 Hz ODR, +/-6 g. */
#define BMI088_ACCEL_CONF_VALUE              0xABU
#define BMI088_ACCEL_RANGE_VALUE             0x01U
#define BMI088_ACCEL_PWR_CONF_ACTIVE         0x00U
#define BMI088_ACCEL_PWR_CTRL_ENABLE         0x04U

/* Gyro: +/-2000 dps, 1000 Hz ODR / 116 Hz bandwidth, normal mode. */
#define BMI088_GYRO_RANGE_VALUE              0x00U
#define BMI088_GYRO_BANDWIDTH_VALUE          0x02U
#define BMI088_GYRO_BANDWIDTH_MASK           0x07U
#define BMI088_GYRO_LPM1_NORMAL              0x00U

#define BMI088_ACCEL_LSB_PER_G       (32768.0f / 6.0f)
#define BMI088_GYRO_LSB_PER_DPS      (32768.0f / 2000.0f)
#define BMI088_VECTOR_DATA_LENGTH             6U
#define BMI088_TEMP_DATA_LENGTH               2U
#define TWO_PI_F                              6.2831853071795864769f

typedef enum
{
    BMI088_DEVICE_ACCEL = 0,
    BMI088_DEVICE_GYRO
} Bmi088Device_t;

typedef struct
{
    bool ready;
    bool calibrating;
    bool calibrated;
    bool bias_tracking_enabled;
    bool last_data_valid;
    Bmi088ChipIds_t chip_ids;
    uint8_t accel_address_7bit;
    uint8_t gyro_address_7bit;
    uint32_t last_valid_ms;
    uint32_t last_filter_ms;
    uint32_t calibration_count;
    FcVector3f_t gyro_bias_dps;
    FcVector3f_t gyro_sum_dps;
    FcVector3f_t filtered_accel_g;
    FcVector3f_t filtered_gyro_dps;
    bool filter_initialized;
} Bmi088State_t;

typedef struct
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_raw;
} Bmi088RawSample_t;

static Bmi088State_t s_state;
volatile Bmi088Debug_t g_bmi088_debug;

static void publish_debug(void)
{
    g_bmi088_debug.accel_address_7bit = s_state.accel_address_7bit;
    g_bmi088_debug.gyro_address_7bit = s_state.gyro_address_7bit;
    g_bmi088_debug.chip_ids = s_state.chip_ids;
    g_bmi088_debug.ready = s_state.ready;
    g_bmi088_debug.calibrated = s_state.calibrated;
}

static void reset_driver_state(void)
{
    s_state = (Bmi088State_t){0};
    g_bmi088_debug = (Bmi088Debug_t){0};
    g_bmi088_debug.init_status = FC_STATUS_NOT_INITIALIZED;
    g_bmi088_debug.last_read_status = FC_STATUS_NOT_INITIALIZED;
}

static void invalidate_output(FcImuData_t *imu)
{
    *imu = (FcImuData_t){0};
    imu->calibrated = s_state.calibrated;
    imu->valid = false;
}

#if FC_USE_STM32_HAL
static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t make_i16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}

static uint8_t device_address_7bit(Bmi088Device_t device)
{
    return (device == BMI088_DEVICE_ACCEL) ?
        s_state.accel_address_7bit : s_state.gyro_address_7bit;
}

static FcStatus_t hal_status_to_fc(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) { return FC_STATUS_OK; }
    if (status == HAL_BUSY) { return FC_STATUS_BUSY; }
    if (status == HAL_TIMEOUT) { return FC_STATUS_TIMEOUT; }
    return FC_STATUS_ERROR;
}

static FcStatus_t i2c_write_register(Bmi088Device_t device,
                                     uint8_t reg,
                                     uint8_t value)
{
    uint8_t address = device_address_7bit(device);
    HAL_StatusTypeDef hal_status;

    if (address == 0U) { return FC_STATUS_NOT_READY; }
    hal_status = HAL_I2C_Mem_Write(&FC_BMI088_I2C_HANDLE,
                                  (uint16_t)address << 1U,
                                  reg,
                                  I2C_MEMADD_SIZE_8BIT,
                                  &value,
                                  1U,
                                  FC_BMI088_I2C_TIMEOUT_MS);
    return hal_status_to_fc(hal_status);
}
#endif

static FcStatus_t i2c_read_registers(Bmi088Device_t device,
                                     uint8_t reg,
                                     uint8_t *data,
                                     uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

#if !FC_USE_STM32_HAL
    (void)device;
    (void)reg;
    return FC_STATUS_NOT_READY;
#else
    uint8_t address = device_address_7bit(device);
    HAL_StatusTypeDef hal_status;

    if (address == 0U) { return FC_STATUS_NOT_READY; }
    hal_status = HAL_I2C_Mem_Read(&FC_BMI088_I2C_HANDLE,
                                 (uint16_t)address << 1U,
                                 reg,
                                 I2C_MEMADD_SIZE_8BIT,
                                 data,
                                 length,
                                 FC_BMI088_I2C_TIMEOUT_MS);
    return hal_status_to_fc(hal_status);
#endif
}

#if FC_USE_STM32_HAL
static FcStatus_t probe_device_address(Bmi088Device_t device,
                                       uint8_t first_address,
                                       uint8_t second_address,
                                       uint8_t expected_chip_id)
{
    const uint8_t addresses[2] = {first_address, second_address};
    uint32_t index;

    for (index = 0U; index < 2U; ++index)
    {
        uint8_t chip_id = 0U;
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&FC_BMI088_I2C_HANDLE,
                                                    (uint16_t)addresses[index] << 1U,
                                                    0x00U,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    &chip_id,
                                                    1U,
                                                    FC_BMI088_I2C_TIMEOUT_MS);
        if ((status == HAL_OK) && (chip_id == expected_chip_id))
        {
            if (device == BMI088_DEVICE_ACCEL)
            {
                s_state.accel_address_7bit = addresses[index];
            }
            else
            {
                s_state.gyro_address_7bit = addresses[index];
            }
            publish_debug();
            return FC_STATUS_OK;
        }
    }
    return FC_STATUS_NOT_READY;
}

static bool axis_mapping_is_valid(void)
{
    bool sources_in_range = (FC_IMU_BODY_X_SOURCE <= FC_AXIS_SOURCE_Z) &&
                            (FC_IMU_BODY_Y_SOURCE <= FC_AXIS_SOURCE_Z) &&
                            (FC_IMU_BODY_Z_SOURCE <= FC_AXIS_SOURCE_Z);
    bool sources_unique = (FC_IMU_BODY_X_SOURCE != FC_IMU_BODY_Y_SOURCE) &&
                          (FC_IMU_BODY_X_SOURCE != FC_IMU_BODY_Z_SOURCE) &&
                          (FC_IMU_BODY_Y_SOURCE != FC_IMU_BODY_Z_SOURCE);
    bool signs_valid = ((FC_IMU_BODY_X_SIGN == 1.0f) || (FC_IMU_BODY_X_SIGN == -1.0f)) &&
                       ((FC_IMU_BODY_Y_SIGN == 1.0f) || (FC_IMU_BODY_Y_SIGN == -1.0f)) &&
                       ((FC_IMU_BODY_Z_SIGN == 1.0f) || (FC_IMU_BODY_Z_SIGN == -1.0f));
    return sources_in_range && sources_unique && signs_valid;
}

static float select_axis(const FcVector3f_t *vector, uint32_t source)
{
    if (source == FC_AXIS_SOURCE_X) { return vector->x; }
    if (source == FC_AXIS_SOURCE_Y) { return vector->y; }
    return vector->z;
}

static FcVector3f_t map_vector_to_body(const FcVector3f_t *sensor)
{
    FcVector3f_t body;
    body.x = select_axis(sensor, FC_IMU_BODY_X_SOURCE) * FC_IMU_BODY_X_SIGN;
    body.y = select_axis(sensor, FC_IMU_BODY_Y_SOURCE) * FC_IMU_BODY_Y_SIGN;
    body.z = select_axis(sensor, FC_IMU_BODY_Z_SOURCE) * FC_IMU_BODY_Z_SIGN;
    return body;
}

static int16_t map_raw_axis(const int16_t raw[3], uint32_t source, float sign)
{
    int32_t value = raw[source];

    if (sign < 0.0f)
    {
        value = -value;
    }
    if (value > 32767) { value = 32767; }
    if (value < -32768) { value = -32768; }
    return (int16_t)value;
}

static FcStatus_t write_and_verify_masked(Bmi088Device_t device,
                                          uint8_t reg,
                                          uint8_t value,
                                          uint8_t mask)
{
    uint8_t readback = 0U;
    FcStatus_t status = i2c_write_register(device, reg, value);

    if (status != FC_STATUS_OK)
    {
        return status;
    }
    status = i2c_read_registers(device, reg, &readback, 1U);
    if (status != FC_STATUS_OK)
    {
        return status;
    }
    return ((readback & mask) == (value & mask)) ?
        FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

static FcStatus_t write_and_verify(Bmi088Device_t device,
                                   uint8_t reg,
                                   uint8_t value)
{
    return write_and_verify_masked(device, reg, value, 0xFFU);
}

static FcStatus_t reset_devices(void)
{
    FcStatus_t status;

    status = i2c_write_register(BMI088_DEVICE_ACCEL,
                                BMI088_ACCEL_REG_SOFTRESET,
                                BMI088_SOFTRESET_VALUE);
    if (status != FC_STATUS_OK) { return status; }
    HAL_Delay(FC_BMI088_ACCEL_RESET_DELAY_MS);

    status = i2c_write_register(BMI088_DEVICE_GYRO,
                                BMI088_GYRO_REG_SOFTRESET,
                                BMI088_SOFTRESET_VALUE);
    if (status != FC_STATUS_OK) { return status; }
    HAL_Delay(FC_BMI088_GYRO_RESET_DELAY_MS);
    return FC_STATUS_OK;
}

static FcStatus_t configure_accelerometer(void)
{
    FcStatus_t status;

    status = i2c_write_register(BMI088_DEVICE_ACCEL,
                                BMI088_ACCEL_REG_PWR_CTRL,
                                BMI088_ACCEL_PWR_CTRL_ENABLE);
    if (status != FC_STATUS_OK) { return status; }
    status = i2c_write_register(BMI088_DEVICE_ACCEL,
                                BMI088_ACCEL_REG_PWR_CONF,
                                BMI088_ACCEL_PWR_CONF_ACTIVE);
    if (status != FC_STATUS_OK) { return status; }
    HAL_Delay(FC_BMI088_ACCEL_POWER_DELAY_MS);

    status = write_and_verify(BMI088_DEVICE_ACCEL,
                              BMI088_ACCEL_REG_PWR_CTRL,
                              BMI088_ACCEL_PWR_CTRL_ENABLE);
    if (status != FC_STATUS_OK) { return status; }
    status = write_and_verify(BMI088_DEVICE_ACCEL,
                              BMI088_ACCEL_REG_PWR_CONF,
                              BMI088_ACCEL_PWR_CONF_ACTIVE);
    if (status != FC_STATUS_OK) { return status; }
    status = write_and_verify(BMI088_DEVICE_ACCEL,
                              BMI088_ACCEL_REG_CONF,
                              BMI088_ACCEL_CONF_VALUE);
    if (status != FC_STATUS_OK) { return status; }
    return write_and_verify(BMI088_DEVICE_ACCEL,
                            BMI088_ACCEL_REG_RANGE,
                            BMI088_ACCEL_RANGE_VALUE);
}

static FcStatus_t configure_gyroscope(void)
{
    FcStatus_t status;

    status = write_and_verify(BMI088_DEVICE_GYRO,
                              BMI088_GYRO_REG_LPM1,
                              BMI088_GYRO_LPM1_NORMAL);
    if (status != FC_STATUS_OK) { return status; }
    status = write_and_verify(BMI088_DEVICE_GYRO,
                              BMI088_GYRO_REG_RANGE,
                              BMI088_GYRO_RANGE_VALUE);
    if (status != FC_STATUS_OK) { return status; }
    /* Bit 7 can read as 1 on BMI088; only BW[2:0] is configurable. */
    return write_and_verify_masked(BMI088_DEVICE_GYRO,
                                   BMI088_GYRO_REG_BANDWIDTH,
                                   BMI088_GYRO_BANDWIDTH_VALUE,
                                   BMI088_GYRO_BANDWIDTH_MASK);
}

static bool data_looks_like_bus_fault(const uint8_t accel[6],
                                      const uint8_t gyro[6])
{
    uint32_t index;
    bool all_zero = true;
    bool all_ff = true;

    for (index = 0U; index < BMI088_VECTOR_DATA_LENGTH; ++index)
    {
        if ((accel[index] != 0x00U) || (gyro[index] != 0x00U)) { all_zero = false; }
        if ((accel[index] != 0xFFU) || (gyro[index] != 0xFFU)) { all_ff = false; }
    }
    return all_zero || all_ff;
}

static int16_t decode_temperature(uint8_t msb, uint8_t lsb)
{
    int16_t raw = (int16_t)(((uint16_t)msb << 3U) | ((uint16_t)lsb >> 5U));
    if (raw > 1023)
    {
        raw = (int16_t)(raw - 2048);
    }
    return raw;
}

static FcStatus_t read_raw_sample(Bmi088RawSample_t *raw)
{
    uint8_t accel[BMI088_VECTOR_DATA_LENGTH];
    uint8_t gyro[BMI088_VECTOR_DATA_LENGTH];
    uint8_t temperature[BMI088_TEMP_DATA_LENGTH];
    FcStatus_t status;

    status = i2c_read_registers(BMI088_DEVICE_ACCEL,
                                BMI088_ACCEL_REG_DATA_X_LSB,
                                accel,
                                BMI088_VECTOR_DATA_LENGTH);
    if (status != FC_STATUS_OK) { return status; }
    status = i2c_read_registers(BMI088_DEVICE_GYRO,
                                BMI088_GYRO_REG_DATA_X_LSB,
                                gyro,
                                BMI088_VECTOR_DATA_LENGTH);
    if (status != FC_STATUS_OK) { return status; }
    status = i2c_read_registers(BMI088_DEVICE_ACCEL,
                                BMI088_ACCEL_REG_TEMP_MSB,
                                temperature,
                                BMI088_TEMP_DATA_LENGTH);
    if (status != FC_STATUS_OK) { return status; }

    if (data_looks_like_bus_fault(accel, gyro))
    {
        return FC_STATUS_INVALID_DATA;
    }

    raw->accel_raw[0] = make_i16(accel[0], accel[1]);
    raw->accel_raw[1] = make_i16(accel[2], accel[3]);
    raw->accel_raw[2] = make_i16(accel[4], accel[5]);
    raw->gyro_raw[0] = make_i16(gyro[0], gyro[1]);
    raw->gyro_raw[1] = make_i16(gyro[2], gyro[3]);
    raw->gyro_raw[2] = make_i16(gyro[4], gyro[5]);
    raw->temperature_raw = decode_temperature(temperature[0], temperature[1]);
    return FC_STATUS_OK;
}

static void convert_raw_sample(const Bmi088RawSample_t *raw,
                               FcImuData_t *sample,
                               FcVector3f_t *gyro_uncorrected_body_dps)
{
    FcVector3f_t accel_sensor_g;
    FcVector3f_t gyro_sensor_dps;

    accel_sensor_g.x = (float)raw->accel_raw[0] / BMI088_ACCEL_LSB_PER_G;
    accel_sensor_g.y = (float)raw->accel_raw[1] / BMI088_ACCEL_LSB_PER_G;
    accel_sensor_g.z = (float)raw->accel_raw[2] / BMI088_ACCEL_LSB_PER_G;
    gyro_sensor_dps.x = (float)raw->gyro_raw[0] / BMI088_GYRO_LSB_PER_DPS;
    gyro_sensor_dps.y = (float)raw->gyro_raw[1] / BMI088_GYRO_LSB_PER_DPS;
    gyro_sensor_dps.z = (float)raw->gyro_raw[2] / BMI088_GYRO_LSB_PER_DPS;

    sample->accel_raw[0] = map_raw_axis(raw->accel_raw, FC_IMU_BODY_X_SOURCE, FC_IMU_BODY_X_SIGN);
    sample->accel_raw[1] = map_raw_axis(raw->accel_raw, FC_IMU_BODY_Y_SOURCE, FC_IMU_BODY_Y_SIGN);
    sample->accel_raw[2] = map_raw_axis(raw->accel_raw, FC_IMU_BODY_Z_SOURCE, FC_IMU_BODY_Z_SIGN);
    sample->gyro_raw[0] = map_raw_axis(raw->gyro_raw, FC_IMU_BODY_X_SOURCE, FC_IMU_BODY_X_SIGN);
    sample->gyro_raw[1] = map_raw_axis(raw->gyro_raw, FC_IMU_BODY_Y_SOURCE, FC_IMU_BODY_Y_SIGN);
    sample->gyro_raw[2] = map_raw_axis(raw->gyro_raw, FC_IMU_BODY_Z_SOURCE, FC_IMU_BODY_Z_SIGN);
    sample->accel_g = map_vector_to_body(&accel_sensor_g);
    *gyro_uncorrected_body_dps = map_vector_to_body(&gyro_sensor_dps);
    sample->temperature_c = 23.0f + ((float)raw->temperature_raw * 0.125f);
    sample->timestamp_ms = HAL_GetTick();
}

static bool sample_is_stationary(const FcVector3f_t *accel_g,
                                 const FcVector3f_t *gyro_dps)
{
    float accel_magnitude_sq = (accel_g->x * accel_g->x) +
                               (accel_g->y * accel_g->y) +
                               (accel_g->z * accel_g->z);

    return (absolute_float(gyro_dps->x) <= FC_IMU_CAL_MAX_GYRO_DPS) &&
           (absolute_float(gyro_dps->y) <= FC_IMU_CAL_MAX_GYRO_DPS) &&
           (absolute_float(gyro_dps->z) <= FC_IMU_CAL_MAX_GYRO_DPS) &&
           (accel_magnitude_sq >= FC_IMU_CAL_ACCEL_MAG_MIN_SQ) &&
           (accel_magnitude_sq <= FC_IMU_CAL_ACCEL_MAG_MAX_SQ);
}

static FcStatus_t update_gyro_calibration(const FcVector3f_t *accel_g,
                                          const FcVector3f_t *gyro_dps)
{
    if (!s_state.calibrating)
    {
        return FC_STATUS_OK;
    }
    if (!sample_is_stationary(accel_g, gyro_dps))
    {
        s_state.calibration_count = 0U;
        s_state.gyro_sum_dps = (FcVector3f_t){0};
        return FC_STATUS_BUSY;
    }

    s_state.gyro_sum_dps.x += gyro_dps->x;
    s_state.gyro_sum_dps.y += gyro_dps->y;
    s_state.gyro_sum_dps.z += gyro_dps->z;
    ++s_state.calibration_count;

    if (s_state.calibration_count < FC_IMU_CALIBRATION_SAMPLE_COUNT)
    {
        return FC_STATUS_BUSY;
    }

    s_state.gyro_bias_dps.x = s_state.gyro_sum_dps.x / (float)s_state.calibration_count;
    s_state.gyro_bias_dps.y = s_state.gyro_sum_dps.y / (float)s_state.calibration_count;
    s_state.gyro_bias_dps.z = s_state.gyro_sum_dps.z / (float)s_state.calibration_count;
    s_state.calibrating = false;
    s_state.calibrated = true;
    return FC_STATUS_OK;
}

static void update_stationary_gyro_bias(const FcVector3f_t *accel_g,
                                        const FcVector3f_t *gyro_dps)
{
    float alpha;

    if (!s_state.bias_tracking_enabled || !s_state.calibrated ||
        !sample_is_stationary(accel_g, gyro_dps))
    {
        return;
    }

    alpha = FC_IMU_BIAS_TRACK_ALPHA;
    if (alpha < 0.0f) { alpha = 0.0f; }
    if (alpha > 1.0f) { alpha = 1.0f; }
    s_state.gyro_bias_dps.x += alpha * (gyro_dps->x - s_state.gyro_bias_dps.x);
    s_state.gyro_bias_dps.y += alpha * (gyro_dps->y - s_state.gyro_bias_dps.y);
    s_state.gyro_bias_dps.z += alpha * (gyro_dps->z - s_state.gyro_bias_dps.z);
}

static float low_pass_alpha(float cutoff_hz, float dt_s)
{
    float omega_dt;

    if ((cutoff_hz <= 0.0f) || (dt_s <= 0.0f))
    {
        return 1.0f;
    }
    omega_dt = TWO_PI_F * cutoff_hz * dt_s;
    return omega_dt / (1.0f + omega_dt);
}

static void filter_output_sample(FcImuData_t *sample)
{
    float dt_s = FC_CONTROL_DT_S;
    float accel_alpha;
    float gyro_alpha;

    if (s_state.filter_initialized)
    {
        uint32_t elapsed_ms = sample->timestamp_ms - s_state.last_filter_ms;
        if (elapsed_ms > 0U)
        {
            dt_s = (float)elapsed_ms * 0.001f;
        }
        if (dt_s > FC_IMU_FILTER_MAX_DT_S)
        {
            s_state.filter_initialized = false;
        }
    }

    if (!s_state.filter_initialized)
    {
        s_state.filtered_accel_g = sample->accel_g;
        s_state.filtered_gyro_dps = sample->gyro_dps;
        s_state.filter_initialized = true;
    }
    else
    {
        accel_alpha = low_pass_alpha(FC_IMU_ACCEL_LPF_HZ, dt_s);
        gyro_alpha = low_pass_alpha(FC_IMU_GYRO_LPF_HZ, dt_s);

        s_state.filtered_accel_g.x += accel_alpha * (sample->accel_g.x - s_state.filtered_accel_g.x);
        s_state.filtered_accel_g.y += accel_alpha * (sample->accel_g.y - s_state.filtered_accel_g.y);
        s_state.filtered_accel_g.z += accel_alpha * (sample->accel_g.z - s_state.filtered_accel_g.z);
        s_state.filtered_gyro_dps.x += gyro_alpha * (sample->gyro_dps.x - s_state.filtered_gyro_dps.x);
        s_state.filtered_gyro_dps.y += gyro_alpha * (sample->gyro_dps.y - s_state.filtered_gyro_dps.y);
        s_state.filtered_gyro_dps.z += gyro_alpha * (sample->gyro_dps.z - s_state.filtered_gyro_dps.z);
    }

    s_state.last_filter_ms = sample->timestamp_ms;
    sample->accel_g = s_state.filtered_accel_g;
    sample->gyro_dps = s_state.filtered_gyro_dps;
}
#endif

FcStatus_t Drv_Bmi088_ReadChipIds(Bmi088ChipIds_t *ids)
{
    FcStatus_t status;

    if (ids == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *ids = (Bmi088ChipIds_t){0};

    status = i2c_read_registers(BMI088_DEVICE_ACCEL,
                                BMI088_ACCEL_REG_CHIP_ID,
                                &ids->accel,
                                1U);
    if (status != FC_STATUS_OK)
    {
        *ids = (Bmi088ChipIds_t){0};
        return status;
    }
    status = i2c_read_registers(BMI088_DEVICE_GYRO,
                                BMI088_GYRO_REG_CHIP_ID,
                                &ids->gyro,
                                1U);
    if (status != FC_STATUS_OK)
    {
        *ids = (Bmi088ChipIds_t){0};
    }
    return status;
}

FcStatus_t Drv_Bmi088_Init(void)
{
    reset_driver_state();

#if !FC_USE_STM32_HAL
    g_bmi088_debug.init_status = FC_STATUS_NOT_READY;
    return FC_STATUS_NOT_READY;
#else
    FcStatus_t status;

    if (!axis_mapping_is_valid())
    {
        g_bmi088_debug.init_status = FC_STATUS_INVALID_DATA;
        return FC_STATUS_INVALID_DATA;
    }

    HAL_Delay(FC_BMI088_STARTUP_DELAY_MS);

    status = probe_device_address(BMI088_DEVICE_ACCEL,
                                  FC_BMI088_ACCEL_I2C_ADDRESS_LOW,
                                  FC_BMI088_ACCEL_I2C_ADDRESS_HIGH,
                                  FC_BMI088_EXPECTED_ACCEL_CHIP_ID);
    if (status == FC_STATUS_OK)
    {
        status = probe_device_address(BMI088_DEVICE_GYRO,
                                      FC_BMI088_GYRO_I2C_ADDRESS_LOW,
                                      FC_BMI088_GYRO_I2C_ADDRESS_HIGH,
                                      FC_BMI088_EXPECTED_GYRO_CHIP_ID);
    }
    if (status == FC_STATUS_OK)
    {
        status = reset_devices();
    }
    if (status == FC_STATUS_OK)
    {
        status = Drv_Bmi088_ReadChipIds(&s_state.chip_ids);
    }
    if (status != FC_STATUS_OK)
    {
        g_bmi088_debug.init_status = status;
        publish_debug();
        return status;
    }
    if ((s_state.chip_ids.accel != FC_BMI088_EXPECTED_ACCEL_CHIP_ID) ||
        (s_state.chip_ids.gyro != FC_BMI088_EXPECTED_GYRO_CHIP_ID))
    {
        g_bmi088_debug.init_status = FC_STATUS_INVALID_DATA;
        publish_debug();
        return FC_STATUS_INVALID_DATA;
    }

    status = configure_accelerometer();
    if (status == FC_STATUS_OK)
    {
        status = configure_gyroscope();
    }
    if (status != FC_STATUS_OK)
    {
        g_bmi088_debug.init_status = status;
        publish_debug();
        return status;
    }

    s_state.ready = true;
    g_bmi088_debug.init_status = FC_STATUS_OK;
    g_bmi088_debug.last_read_status = FC_STATUS_NOT_READY;
    publish_debug();
    return FC_STATUS_OK;
#endif
}

FcStatus_t Drv_Bmi088_Read(FcImuData_t *imu)
{
    if (imu == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    invalidate_output(imu);

#if !FC_USE_STM32_HAL
    return FC_STATUS_NOT_READY;
#else
    Bmi088RawSample_t raw;
    FcImuData_t sample = {0};
    FcVector3f_t gyro_uncorrected_body_dps;
    FcStatus_t status;

    if (!s_state.ready)
    {
        g_bmi088_debug.last_read_status = FC_STATUS_NOT_READY;
        return FC_STATUS_NOT_READY;
    }

    status = read_raw_sample(&raw);
    if (status != FC_STATUS_OK)
    {
        s_state.last_data_valid = false;
        g_bmi088_debug.last_read_status = status;
        ++g_bmi088_debug.failed_read_count;
        publish_debug();
        return status;
    }

    convert_raw_sample(&raw, &sample, &gyro_uncorrected_body_dps);
    status = update_gyro_calibration(&sample.accel_g, &gyro_uncorrected_body_dps);
    if (status != FC_STATUS_OK)
    {
        s_state.last_data_valid = false;
        g_bmi088_debug.last_read_status = status;
        publish_debug();
        return status;
    }

    /* Runtime bias tracking is enabled only while the App is disarmed. */
    update_stationary_gyro_bias(&sample.accel_g, &gyro_uncorrected_body_dps);

    sample.gyro_dps.x = gyro_uncorrected_body_dps.x - s_state.gyro_bias_dps.x;
    sample.gyro_dps.y = gyro_uncorrected_body_dps.y - s_state.gyro_bias_dps.y;
    sample.gyro_dps.z = gyro_uncorrected_body_dps.z - s_state.gyro_bias_dps.z;
    filter_output_sample(&sample);
    sample.calibrated = s_state.calibrated;
    sample.valid = true;

    *imu = sample;
    s_state.last_valid_ms = sample.timestamp_ms;
    s_state.last_data_valid = true;
    g_bmi088_debug.last_read_status = FC_STATUS_OK;
    ++g_bmi088_debug.valid_read_count;
    publish_debug();
    return FC_STATUS_OK;
#endif
}

FcStatus_t Drv_Bmi088_CalibrateGyro(void)
{
    if (!s_state.ready)
    {
        return FC_STATUS_NOT_READY;
    }

    s_state.calibrating = true;
    s_state.calibrated = false;
    s_state.last_data_valid = false;
    s_state.calibration_count = 0U;
    s_state.gyro_bias_dps = (FcVector3f_t){0};
    s_state.gyro_sum_dps = (FcVector3f_t){0};
    s_state.filtered_accel_g = (FcVector3f_t){0};
    s_state.filtered_gyro_dps = (FcVector3f_t){0};
    s_state.last_filter_ms = 0U;
    s_state.filter_initialized = false;
    publish_debug();
    return FC_STATUS_OK;
}

FcStatus_t Drv_Bmi088_SetBiasTrackingEnabled(bool enabled)
{
    if (!s_state.ready)
    {
        s_state.bias_tracking_enabled = false;
        return FC_STATUS_NOT_READY;
    }
    s_state.bias_tracking_enabled = enabled;
    return FC_STATUS_OK;
}

bool Drv_Bmi088_IsReady(void)
{
    return s_state.ready;
}

FcStatus_t Drv_Bmi088_GetChipIds(Bmi088ChipIds_t *ids)
{
    if (ids == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *ids = s_state.chip_ids;
    return s_state.ready ? FC_STATUS_OK : FC_STATUS_NOT_READY;
}

bool Drv_Bmi088_IsCalibrationComplete(void)
{
    return s_state.ready && s_state.calibrated && !s_state.calibrating;
}

bool Drv_Bmi088_IsDataValid(uint32_t now_ms)
{
    return s_state.ready && s_state.last_data_valid &&
           ((uint32_t)(now_ms - s_state.last_valid_ms) <= FC_IMU_DATA_TIMEOUT_MS);
}

FcStatus_t Drv_Bmi088_GetGyroBias(FcVector3f_t *bias_dps)
{
    if (bias_dps == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    *bias_dps = s_state.gyro_bias_dps;
    if (!s_state.ready) { return FC_STATUS_NOT_READY; }
    return s_state.calibrated ? FC_STATUS_OK : FC_STATUS_BUSY;
}

FcStatus_t Drv_Bmi088_GetDebug(Bmi088Debug_t *debug)
{
    if (debug == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    debug->accel_address_7bit = g_bmi088_debug.accel_address_7bit;
    debug->gyro_address_7bit = g_bmi088_debug.gyro_address_7bit;
    debug->chip_ids.accel = g_bmi088_debug.chip_ids.accel;
    debug->chip_ids.gyro = g_bmi088_debug.chip_ids.gyro;
    debug->init_status = g_bmi088_debug.init_status;
    debug->last_read_status = g_bmi088_debug.last_read_status;
    debug->valid_read_count = g_bmi088_debug.valid_read_count;
    debug->failed_read_count = g_bmi088_debug.failed_read_count;
    debug->ready = g_bmi088_debug.ready;
    debug->calibrated = g_bmi088_debug.calibrated;
    return s_state.ready ? FC_STATUS_OK : FC_STATUS_NOT_READY;
}
