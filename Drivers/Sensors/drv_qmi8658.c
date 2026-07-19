/* QMI8658C SPI driver with fail-closed reads, body-axis mapping, and RAM bias. */

#include <stddef.h>
#include "drv_qmi8658.h"
#include "fc_board.h"
#include "fc_config.h"

#define QMI8658_REG_WHO_AM_I       0x00U
#define QMI8658_REG_CTRL1          0x02U
#define QMI8658_REG_CTRL2          0x03U
#define QMI8658_REG_CTRL3          0x04U
#define QMI8658_REG_CTRL5          0x06U
#define QMI8658_REG_CTRL7          0x08U
#define QMI8658_REG_TEMP_L         0x33U
#define QMI8658_REG_RESET          0x60U

#define QMI8658_SPI_READ_BIT       0x80U
#define QMI8658_RESET_VALUE        0xB0U

/*
 * CTRL1: 4-wire SPI and register address auto-increment.
 * CTRL2: accel 1000 Hz, +/-8 g.
 * CTRL3: gyro 1000 Hz, +/-2048 deg/s.
 * CTRL5: accel and gyro low-pass filters enabled.
 * CTRL7: accel and gyro enabled.
 */
#define QMI8658_CTRL1_VALUE        0x60U
#define QMI8658_CTRL2_VALUE        0x23U
#define QMI8658_CTRL3_VALUE        0x73U
#define QMI8658_CTRL5_VALUE        0x11U
#define QMI8658_CTRL7_DISABLE      0x00U
#define QMI8658_CTRL7_ENABLE       0x03U

#define QMI8658_ACCEL_LSB_PER_G    4096.0f
#define QMI8658_GYRO_LSB_PER_DPS     16.0f
#define QMI8658_TEMP_LSB_PER_C      256.0f
#define QMI8658_BURST_LENGTH         14U

typedef struct
{
    bool ready;
    bool calibrating;
    bool calibrated;
    bool last_data_valid;
    uint8_t who_am_i;
    uint32_t last_valid_ms;
    uint32_t calibration_count;
    FcVector3f_t gyro_bias_dps;
    FcVector3f_t gyro_sum_dps;
} Qmi8658State_t;

static Qmi8658State_t s_state;

static void reset_driver_state(void)
{
    s_state = (Qmi8658State_t){0};
}

static void invalidate_output(FcImuData_t *imu)
{
    *imu = (FcImuData_t){0};
    imu->calibrated = s_state.calibrated;
    imu->valid = false;
}

#if FC_USE_STM32_HAL
typedef struct
{
    int16_t temperature_raw;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
} Qmi8658RawSample_t;

typedef struct
{
    uint8_t reg;
    uint8_t value;
} Qmi8658RegisterConfig_t;

static const Qmi8658RegisterConfig_t s_register_config[] = {
    {QMI8658_REG_CTRL7, QMI8658_CTRL7_DISABLE},
    {QMI8658_REG_CTRL1, QMI8658_CTRL1_VALUE},
    {QMI8658_REG_CTRL2, QMI8658_CTRL2_VALUE},
    {QMI8658_REG_CTRL3, QMI8658_CTRL3_VALUE},
    {QMI8658_REG_CTRL5, QMI8658_CTRL5_VALUE},
    {QMI8658_REG_CTRL7, QMI8658_CTRL7_ENABLE}
};

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t make_i16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
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

static FcStatus_t hal_status_to_fc(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) { return FC_STATUS_OK; }
    if (status == HAL_BUSY) { return FC_STATUS_BUSY; }
    if (status == HAL_TIMEOUT) { return FC_STATUS_TIMEOUT; }
    return FC_STATUS_ERROR;
}

static FcStatus_t spi_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & (uint8_t)~QMI8658_SPI_READ_BIT), value};
    HAL_StatusTypeDef hal_status;

    FC_QMI8658_CS_LOW();
    hal_status = HAL_SPI_Transmit(&hspi1, tx, 2U, FC_QMI8658_SPI_TIMEOUT_MS);
    FC_QMI8658_CS_HIGH();
    return hal_status_to_fc(hal_status);
}
#endif

static FcStatus_t spi_read_registers(uint8_t reg, uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

#if !FC_USE_STM32_HAL
    (void)reg;
    return FC_STATUS_NOT_READY;
#else
    uint8_t command = (uint8_t)(reg | QMI8658_SPI_READ_BIT);
    HAL_StatusTypeDef hal_status;

    FC_QMI8658_CS_LOW();
    hal_status = HAL_SPI_Transmit(&hspi1, &command, 1U, FC_QMI8658_SPI_TIMEOUT_MS);
    if (hal_status == HAL_OK)
    {
        hal_status = HAL_SPI_Receive(&hspi1, data, length, FC_QMI8658_SPI_TIMEOUT_MS);
    }
    FC_QMI8658_CS_HIGH();
    return hal_status_to_fc(hal_status);
#endif
}

#if FC_USE_STM32_HAL
static FcStatus_t write_and_verify(uint8_t reg, uint8_t value)
{
    uint8_t readback = 0U;
    FcStatus_t status = spi_write_register(reg, value);

    if (status != FC_STATUS_OK)
    {
        return status;
    }
    status = spi_read_registers(reg, &readback, 1U);
    if (status != FC_STATUS_OK)
    {
        return status;
    }
    return (readback == value) ? FC_STATUS_OK : FC_STATUS_INVALID_DATA;
}

static FcStatus_t reset_device(void)
{
    FcStatus_t status = spi_write_register(QMI8658_REG_RESET, QMI8658_RESET_VALUE);

    if (status != FC_STATUS_OK)
    {
        return status;
    }

    /* Initialization only. Drv_Qmi8658_Read() never calls HAL_Delay(). */
    HAL_Delay(FC_QMI8658_RESET_DELAY_MS);
    return FC_STATUS_OK;
}

static FcStatus_t configure_device(void)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)(sizeof(s_register_config) / sizeof(s_register_config[0])); ++index)
    {
        FcStatus_t status = write_and_verify(s_register_config[index].reg,
                                             s_register_config[index].value);
        if (status != FC_STATUS_OK)
        {
            return status;
        }
    }
    return FC_STATUS_OK;
}

static bool burst_looks_like_bus_fault(const uint8_t data[QMI8658_BURST_LENGTH])
{
    uint32_t index;
    bool all_zero = true;
    bool all_ff = true;

    for (index = 0U; index < QMI8658_BURST_LENGTH; ++index)
    {
        if (data[index] != 0x00U) { all_zero = false; }
        if (data[index] != 0xFFU) { all_ff = false; }
    }
    return all_zero || all_ff;
}

static FcStatus_t read_raw_sample(Qmi8658RawSample_t *raw)
{
    uint8_t data[QMI8658_BURST_LENGTH];
    FcStatus_t status = spi_read_registers(QMI8658_REG_TEMP_L, data, QMI8658_BURST_LENGTH);

    if (status != FC_STATUS_OK)
    {
        return status;
    }
    if (burst_looks_like_bus_fault(data))
    {
        return FC_STATUS_INVALID_DATA;
    }

    raw->temperature_raw = make_i16(data[0], data[1]);
    raw->accel_raw[0] = make_i16(data[2], data[3]);
    raw->accel_raw[1] = make_i16(data[4], data[5]);
    raw->accel_raw[2] = make_i16(data[6], data[7]);
    raw->gyro_raw[0] = make_i16(data[8], data[9]);
    raw->gyro_raw[1] = make_i16(data[10], data[11]);
    raw->gyro_raw[2] = make_i16(data[12], data[13]);
    return FC_STATUS_OK;
}

static void convert_raw_sample(const Qmi8658RawSample_t *raw,
                               FcImuData_t *sample,
                               FcVector3f_t *gyro_uncorrected_body_dps)
{
    FcVector3f_t accel_sensor_g;
    FcVector3f_t gyro_sensor_dps;

    accel_sensor_g.x = (float)raw->accel_raw[0] / QMI8658_ACCEL_LSB_PER_G;
    accel_sensor_g.y = (float)raw->accel_raw[1] / QMI8658_ACCEL_LSB_PER_G;
    accel_sensor_g.z = (float)raw->accel_raw[2] / QMI8658_ACCEL_LSB_PER_G;
    gyro_sensor_dps.x = (float)raw->gyro_raw[0] / QMI8658_GYRO_LSB_PER_DPS;
    gyro_sensor_dps.y = (float)raw->gyro_raw[1] / QMI8658_GYRO_LSB_PER_DPS;
    gyro_sensor_dps.z = (float)raw->gyro_raw[2] / QMI8658_GYRO_LSB_PER_DPS;

    sample->accel_raw[0] = map_raw_axis(raw->accel_raw, FC_IMU_BODY_X_SOURCE, FC_IMU_BODY_X_SIGN);
    sample->accel_raw[1] = map_raw_axis(raw->accel_raw, FC_IMU_BODY_Y_SOURCE, FC_IMU_BODY_Y_SIGN);
    sample->accel_raw[2] = map_raw_axis(raw->accel_raw, FC_IMU_BODY_Z_SOURCE, FC_IMU_BODY_Z_SIGN);
    sample->gyro_raw[0] = map_raw_axis(raw->gyro_raw, FC_IMU_BODY_X_SOURCE, FC_IMU_BODY_X_SIGN);
    sample->gyro_raw[1] = map_raw_axis(raw->gyro_raw, FC_IMU_BODY_Y_SOURCE, FC_IMU_BODY_Y_SIGN);
    sample->gyro_raw[2] = map_raw_axis(raw->gyro_raw, FC_IMU_BODY_Z_SOURCE, FC_IMU_BODY_Z_SIGN);
    sample->accel_g = map_vector_to_body(&accel_sensor_g);
    *gyro_uncorrected_body_dps = map_vector_to_body(&gyro_sensor_dps);
    sample->temperature_c = (float)raw->temperature_raw / QMI8658_TEMP_LSB_PER_C;
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
#endif

FcStatus_t Drv_Qmi8658_ReadWhoAmI(uint8_t *who_am_i)
{
    if (who_am_i == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    *who_am_i = 0U;
    return spi_read_registers(QMI8658_REG_WHO_AM_I, who_am_i, 1U);
}

FcStatus_t Drv_Qmi8658_Init(void)
{
    reset_driver_state();

#if !FC_USE_STM32_HAL
    return FC_STATUS_NOT_READY;
#else
    FcStatus_t status;

    if (!axis_mapping_is_valid())
    {
        return FC_STATUS_INVALID_DATA;
    }

    FC_QMI8658_CS_HIGH();
    status = reset_device();
    if (status == FC_STATUS_OK)
    {
        status = Drv_Qmi8658_ReadWhoAmI(&s_state.who_am_i);
    }
    if (status != FC_STATUS_OK)
    {
        return status;
    }
    if (s_state.who_am_i != FC_QMI8658_EXPECTED_WHO_AM_I)
    {
        return FC_STATUS_INVALID_DATA;
    }

    status = configure_device();
    if (status != FC_STATUS_OK)
    {
        return status;
    }

    s_state.ready = true;
    return FC_STATUS_OK;
#endif
}

FcStatus_t Drv_Qmi8658_Read(FcImuData_t *imu)
{
    if (imu == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    invalidate_output(imu);

#if !FC_USE_STM32_HAL
    return FC_STATUS_NOT_READY;
#else
    Qmi8658RawSample_t raw;
    FcImuData_t sample = {0};
    FcVector3f_t gyro_uncorrected_body_dps;
    FcStatus_t status;

    if (!s_state.ready)
    {
        return FC_STATUS_NOT_READY;
    }

    status = read_raw_sample(&raw);
    if (status != FC_STATUS_OK)
    {
        s_state.last_data_valid = false;
        return status;
    }

    convert_raw_sample(&raw, &sample, &gyro_uncorrected_body_dps);
    status = update_gyro_calibration(&sample.accel_g, &gyro_uncorrected_body_dps);
    if (status != FC_STATUS_OK)
    {
        s_state.last_data_valid = false;
        return status;
    }

    sample.gyro_dps.x = gyro_uncorrected_body_dps.x - s_state.gyro_bias_dps.x;
    sample.gyro_dps.y = gyro_uncorrected_body_dps.y - s_state.gyro_bias_dps.y;
    sample.gyro_dps.z = gyro_uncorrected_body_dps.z - s_state.gyro_bias_dps.z;
    sample.calibrated = s_state.calibrated;
    sample.valid = true;

    *imu = sample;
    s_state.last_valid_ms = sample.timestamp_ms;
    s_state.last_data_valid = true;
    return FC_STATUS_OK;
#endif
}

FcStatus_t Drv_Qmi8658_CalibrateGyro(void)
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
    return FC_STATUS_OK;
}

bool Drv_Qmi8658_IsReady(void)
{
    return s_state.ready;
}

uint8_t Drv_Qmi8658_GetWhoAmI(void)
{
    return s_state.who_am_i;
}

bool Drv_Qmi8658_IsCalibrationComplete(void)
{
    return s_state.ready && s_state.calibrated && !s_state.calibrating;
}

bool Drv_Qmi8658_IsDataValid(uint32_t now_ms)
{
    return s_state.ready && s_state.last_data_valid &&
           ((uint32_t)(now_ms - s_state.last_valid_ms) <= FC_IMU_DATA_TIMEOUT_MS);
}

FcStatus_t Drv_Qmi8658_GetGyroBias(FcVector3f_t *bias_dps)
{
    if (bias_dps == NULL)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }

    *bias_dps = s_state.gyro_bias_dps;
    if (!s_state.ready) { return FC_STATUS_NOT_READY; }
    return s_state.calibrated ? FC_STATUS_OK : FC_STATUS_BUSY;
}
