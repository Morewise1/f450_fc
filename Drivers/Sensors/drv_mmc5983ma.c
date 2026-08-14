/* MMC5983MA I2C driver with 18-bit conversion and periodic automatic SET. */

#include <stddef.h>
#include "bsp_soft_i2c.h"
#include "drv_mmc5983ma.h"
#include "fc_board.h"
#include "fc_config.h"

#define MMC5983MA_REG_XOUT0                0x00U
#define MMC5983MA_REG_STATUS               0x08U
#define MMC5983MA_REG_CONTROL_0            0x09U
#define MMC5983MA_REG_CONTROL_1            0x0AU
#define MMC5983MA_REG_CONTROL_2            0x0BU
#define MMC5983MA_REG_PRODUCT_ID           0x2FU

#define MMC5983MA_STATUS_MEAS_M_DONE       0x01U
#define MMC5983MA_STATUS_OTP_READ_DONE     0x10U
#define MMC5983MA_CONTROL_0_SET            0x08U
#define MMC5983MA_CONTROL_0_AUTO_SR_ENABLE 0x20U
#define MMC5983MA_CONTROL_1_BW_200HZ       0x01U
#define MMC5983MA_CONTROL_1_SOFT_RESET     0x80U
/* Periodic SET every 100 samples, continuous mode, 100 Hz. */
#define MMC5983MA_CONTROL_2_CONTINUOUS     0xBDU
#define MMC5983MA_DATA_LENGTH                 7U
#define MMC5983MA_ZERO_COUNT             131072L
#define MMC5983MA_MAX_COUNT              262143UL
#define MMC5983MA_COUNTS_PER_GAUSS      16384.0f
#define MICROTESLA_PER_GAUSS              100.0f

typedef struct
{
    uint8_t product_id;
    FcVector3f_t offset_ut;
    FcVector3f_t scale;
    FcVector3f_t calibration_min_ut;
    FcVector3f_t calibration_max_ut;
    uint32_t calibration_sample_count;
    bool calibration_active;
    bool calibration_valid;
    bool ready;
} Mmc5983maState_t;

static Mmc5983maState_t s_state;
volatile DrvMmc5983maDebug_t g_mmc5983ma_debug;

static void reset_state(void)
{
    s_state = (Mmc5983maState_t){0};
    s_state.offset_ut.x = FC_MAG_OFFSET_X_UT;
    s_state.offset_ut.y = FC_MAG_OFFSET_Y_UT;
    s_state.offset_ut.z = FC_MAG_OFFSET_Z_UT;
    s_state.scale.x = FC_MAG_SCALE_X;
    s_state.scale.y = FC_MAG_SCALE_Y;
    s_state.scale.z = FC_MAG_SCALE_Z;
    g_mmc5983ma_debug = (DrvMmc5983maDebug_t){0};
    g_mmc5983ma_debug.calibration_offset_ut = s_state.offset_ut;
    g_mmc5983ma_debug.calibration_scale = s_state.scale;
    g_mmc5983ma_debug.init_status = FC_STATUS_NOT_INITIALIZED;
    g_mmc5983ma_debug.last_read_status = FC_STATUS_NOT_INITIALIZED;
}

#if FC_USE_STM32_HAL
static float vector_component(const FcVector3f_t *vector, uint32_t index)
{
    if (index == 0U) { return vector->x; }
    if (index == 1U) { return vector->y; }
    return vector->z;
}

static int32_t raw_component(const int32_t raw[3], uint32_t index)
{
    return raw[index];
}

static void map_to_body_frame(const int32_t sensor_raw[3],
                              const FcVector3f_t *sensor_ut,
                              int32_t body_raw[3],
                              FcVector3f_t *body_ut)
{
    body_raw[0] = (int32_t)(FC_MAG_BODY_X_SIGN *
                           (float)raw_component(sensor_raw, FC_MAG_BODY_X_SOURCE));
    body_raw[1] = (int32_t)(FC_MAG_BODY_Y_SIGN *
                           (float)raw_component(sensor_raw, FC_MAG_BODY_Y_SOURCE));
    body_raw[2] = (int32_t)(FC_MAG_BODY_Z_SIGN *
                           (float)raw_component(sensor_raw, FC_MAG_BODY_Z_SOURCE));
    body_ut->x = FC_MAG_BODY_X_SIGN *
                 vector_component(sensor_ut, FC_MAG_BODY_X_SOURCE);
    body_ut->y = FC_MAG_BODY_Y_SIGN *
                 vector_component(sensor_ut, FC_MAG_BODY_Y_SOURCE);
    body_ut->z = FC_MAG_BODY_Z_SIGN *
                 vector_component(sensor_ut, FC_MAG_BODY_Z_SOURCE);
}

static void update_calibration_extrema(const FcVector3f_t *field_ut)
{
    if (s_state.calibration_sample_count == 0U)
    {
        s_state.calibration_min_ut = *field_ut;
        s_state.calibration_max_ut = *field_ut;
    }
    else
    {
        if (field_ut->x < s_state.calibration_min_ut.x) { s_state.calibration_min_ut.x = field_ut->x; }
        if (field_ut->y < s_state.calibration_min_ut.y) { s_state.calibration_min_ut.y = field_ut->y; }
        if (field_ut->z < s_state.calibration_min_ut.z) { s_state.calibration_min_ut.z = field_ut->z; }
        if (field_ut->x > s_state.calibration_max_ut.x) { s_state.calibration_max_ut.x = field_ut->x; }
        if (field_ut->y > s_state.calibration_max_ut.y) { s_state.calibration_max_ut.y = field_ut->y; }
        if (field_ut->z > s_state.calibration_max_ut.z) { s_state.calibration_max_ut.z = field_ut->z; }
    }
    ++s_state.calibration_sample_count;
    g_mmc5983ma_debug.calibration_sample_count = s_state.calibration_sample_count;
}

static bool field_strength_is_valid(const FcVector3f_t *field_ut)
{
    float magnitude_sq = (field_ut->x * field_ut->x) +
                         (field_ut->y * field_ut->y) +
                         (field_ut->z * field_ut->z);
    return (magnitude_sq >= (FC_MAG_VALID_MIN_FIELD_UT * FC_MAG_VALID_MIN_FIELD_UT)) &&
           (magnitude_sq <= (FC_MAG_VALID_MAX_FIELD_UT * FC_MAG_VALID_MAX_FIELD_UT));
}
#endif /* FC_USE_STM32_HAL */

static void invalidate_output(FcMagnetometerData_t *data, uint32_t timestamp_ms)
{
    *data = (FcMagnetometerData_t){0};
    data->timestamp_ms = timestamp_ms;
}

#if FC_USE_STM32_HAL
static FcStatus_t read_registers(uint8_t reg, uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) { return FC_STATUS_INVALID_ARGUMENT; }
    return BSP_SoftI2c_MemRead(BSP_SOFT_I2C_BUS_MMC5983MA,
                               FC_MMC5983MA_I2C_ADDRESS,
                               reg,
                               data,
                               length);
}

static FcStatus_t write_register(uint8_t reg, uint8_t value)
{
    return BSP_SoftI2c_MemWrite(BSP_SOFT_I2C_BUS_MMC5983MA,
                                FC_MMC5983MA_I2C_ADDRESS,
                                reg,
                                &value,
                                1U);
}

static uint32_t unpack_axis(uint8_t msb,
                            uint8_t middle,
                            uint8_t low_two_bits)
{
    return ((uint32_t)msb << 10U) |
           ((uint32_t)middle << 2U) |
           ((uint32_t)low_two_bits & 0x03U);
}

static bool raw_sample_is_saturated(const int32_t raw[3])
{
    const int32_t lower_limit = -MMC5983MA_ZERO_COUNT + 1L;
    const int32_t upper_limit = (int32_t)(MMC5983MA_MAX_COUNT -
                                         (uint32_t)MMC5983MA_ZERO_COUNT) - 1L;

    return (raw[0] <= lower_limit) || (raw[0] >= upper_limit) ||
           (raw[1] <= lower_limit) || (raw[1] >= upper_limit) ||
           (raw[2] <= lower_limit) || (raw[2] >= upper_limit);
}
#endif /* FC_USE_STM32_HAL */

FcStatus_t Drv_Mmc5983ma_Init(void)
{
    reset_state();

#if !FC_USE_STM32_HAL
    g_mmc5983ma_debug.init_status = FC_STATUS_NOT_READY;
    return FC_STATUS_NOT_READY;
#else
    uint8_t status_register = 0U;
    FcStatus_t status;

    status = BSP_SoftI2c_Init(BSP_SOFT_I2C_BUS_MMC5983MA);
    if (status == FC_STATUS_OK)
    {
        HAL_Delay(FC_MMC5983MA_STARTUP_DELAY_MS);
        status = read_registers(MMC5983MA_REG_PRODUCT_ID,
                                &s_state.product_id,
                                1U);
    }
    if ((status == FC_STATUS_OK) &&
        (s_state.product_id != FC_MMC5983MA_EXPECTED_PRODUCT_ID))
    {
        status = FC_STATUS_INVALID_DATA;
    }
    if (status == FC_STATUS_OK)
    {
        status = write_register(MMC5983MA_REG_CONTROL_1,
                                MMC5983MA_CONTROL_1_SOFT_RESET);
    }
    if (status == FC_STATUS_OK)
    {
        HAL_Delay(FC_MMC5983MA_RESET_DELAY_MS);
        status = read_registers(MMC5983MA_REG_PRODUCT_ID,
                                &s_state.product_id,
                                1U);
    }
    if ((status == FC_STATUS_OK) &&
        (s_state.product_id != FC_MMC5983MA_EXPECTED_PRODUCT_ID))
    {
        status = FC_STATUS_INVALID_DATA;
    }
    if (status == FC_STATUS_OK)
    {
        status = read_registers(MMC5983MA_REG_STATUS, &status_register, 1U);
    }
    if ((status == FC_STATUS_OK) &&
        ((status_register & MMC5983MA_STATUS_OTP_READ_DONE) == 0U))
    {
        status = FC_STATUS_NOT_READY;
    }
    if (status == FC_STATUS_OK)
    {
        status = write_register(MMC5983MA_REG_CONTROL_0,
                                MMC5983MA_CONTROL_0_SET);
    }
    if (status == FC_STATUS_OK)
    {
        HAL_Delay(FC_MMC5983MA_SET_DELAY_MS);
        status = write_register(MMC5983MA_REG_CONTROL_1,
                                MMC5983MA_CONTROL_1_BW_200HZ);
    }
    if (status == FC_STATUS_OK)
    {
        status = write_register(MMC5983MA_REG_CONTROL_0,
                                MMC5983MA_CONTROL_0_AUTO_SR_ENABLE);
    }
    if (status == FC_STATUS_OK)
    {
        status = write_register(MMC5983MA_REG_CONTROL_2,
                                MMC5983MA_CONTROL_2_CONTINUOUS);
    }

    s_state.ready = status == FC_STATUS_OK;
    g_mmc5983ma_debug.product_id = s_state.product_id;
    g_mmc5983ma_debug.ready = s_state.ready;
    g_mmc5983ma_debug.init_status = status;
    g_mmc5983ma_debug.last_read_status = s_state.ready ? FC_STATUS_NOT_READY : status;
    return status;
#endif
}

FcStatus_t Drv_Mmc5983ma_Read(FcMagnetometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    invalidate_output(data, timestamp_ms);

#if !FC_USE_STM32_HAL
    return FC_STATUS_NOT_READY;
#else
    uint8_t status_register = 0U;
    uint8_t bytes[MMC5983MA_DATA_LENGTH];
    uint32_t x_unsigned;
    uint32_t y_unsigned;
    uint32_t z_unsigned;
    int32_t sensor_raw[3];
    FcVector3f_t sensor_ut;
    FcVector3f_t body_ut;
    float count_to_microtesla = MICROTESLA_PER_GAUSS /
                                MMC5983MA_COUNTS_PER_GAUSS;
    FcStatus_t status;

    if (!s_state.ready) { return FC_STATUS_NOT_INITIALIZED; }
    status = read_registers(MMC5983MA_REG_STATUS, &status_register, 1U);
    if ((status == FC_STATUS_OK) &&
        ((status_register & MMC5983MA_STATUS_MEAS_M_DONE) == 0U))
    {
        status = FC_STATUS_BUSY;
    }
    if (status == FC_STATUS_OK)
    {
        status = read_registers(MMC5983MA_REG_XOUT0,
                                bytes,
                                MMC5983MA_DATA_LENGTH);
    }
    if (status == FC_STATUS_OK)
    {
        x_unsigned = unpack_axis(bytes[0], bytes[1], bytes[6] >> 6U);
        y_unsigned = unpack_axis(bytes[2], bytes[3], bytes[6] >> 4U);
        z_unsigned = unpack_axis(bytes[4], bytes[5], bytes[6] >> 2U);
        sensor_raw[0] = (int32_t)x_unsigned - MMC5983MA_ZERO_COUNT;
        sensor_raw[1] = (int32_t)y_unsigned - MMC5983MA_ZERO_COUNT;
        sensor_raw[2] = (int32_t)z_unsigned - MMC5983MA_ZERO_COUNT;
        sensor_ut.x = (float)sensor_raw[0] * count_to_microtesla;
        sensor_ut.y = (float)sensor_raw[1] * count_to_microtesla;
        sensor_ut.z = (float)sensor_raw[2] * count_to_microtesla;
        map_to_body_frame(sensor_raw, &sensor_ut, data->raw, &body_ut);
        data->overflow = raw_sample_is_saturated(sensor_raw);
        if (data->overflow)
        {
            status = FC_STATUS_INVALID_DATA;
            ++g_mmc5983ma_debug.saturated_read_count;
        }
        else
        {
            if (s_state.calibration_active) { update_calibration_extrema(&body_ut); }
            data->magnetic_ut.x = (body_ut.x - s_state.offset_ut.x) * s_state.scale.x;
            data->magnetic_ut.y = (body_ut.y - s_state.offset_ut.y) * s_state.scale.y;
            data->magnetic_ut.z = (body_ut.z - s_state.offset_ut.z) * s_state.scale.z;
            if (!s_state.calibration_active && !field_strength_is_valid(&data->magnetic_ut))
            {
                status = FC_STATUS_INVALID_DATA;
            }
        }
    }

    data->valid = status == FC_STATUS_OK;
    g_mmc5983ma_debug.last_read_status = status;
    if (status == FC_STATUS_OK) { ++g_mmc5983ma_debug.valid_read_count; }
    else if (status != FC_STATUS_BUSY) { ++g_mmc5983ma_debug.failed_read_count; }
    return status;
#endif
}

FcStatus_t Drv_Mmc5983ma_StartCalibration(void)
{
    if (!s_state.ready) { return FC_STATUS_NOT_INITIALIZED; }
    s_state.calibration_min_ut = (FcVector3f_t){0};
    s_state.calibration_max_ut = (FcVector3f_t){0};
    s_state.calibration_sample_count = 0U;
    s_state.calibration_active = true;
    s_state.calibration_valid = false;
    g_mmc5983ma_debug.calibration_sample_count = 0U;
    g_mmc5983ma_debug.calibration_active = true;
    g_mmc5983ma_debug.calibration_valid = false;
    return FC_STATUS_OK;
}

FcStatus_t Drv_Mmc5983ma_FinishCalibration(void)
{
    FcVector3f_t radius;
    float average_radius;

    if (!s_state.calibration_active) { return FC_STATUS_NOT_READY; }
    s_state.calibration_active = false;
    g_mmc5983ma_debug.calibration_active = false;
    radius.x = 0.5f * (s_state.calibration_max_ut.x - s_state.calibration_min_ut.x);
    radius.y = 0.5f * (s_state.calibration_max_ut.y - s_state.calibration_min_ut.y);
    radius.z = 0.5f * (s_state.calibration_max_ut.z - s_state.calibration_min_ut.z);
    if ((s_state.calibration_sample_count < FC_MAG_CAL_MIN_SAMPLES) ||
        (radius.x < (0.5f * FC_MAG_CAL_MIN_AXIS_SPAN_UT)) ||
        (radius.y < (0.5f * FC_MAG_CAL_MIN_AXIS_SPAN_UT)) ||
        (radius.z < (0.5f * FC_MAG_CAL_MIN_AXIS_SPAN_UT)))
    {
        return FC_STATUS_INVALID_DATA;
    }

    s_state.offset_ut.x = 0.5f * (s_state.calibration_max_ut.x + s_state.calibration_min_ut.x);
    s_state.offset_ut.y = 0.5f * (s_state.calibration_max_ut.y + s_state.calibration_min_ut.y);
    s_state.offset_ut.z = 0.5f * (s_state.calibration_max_ut.z + s_state.calibration_min_ut.z);
    average_radius = (radius.x + radius.y + radius.z) / 3.0f;
    s_state.scale.x = average_radius / radius.x;
    s_state.scale.y = average_radius / radius.y;
    s_state.scale.z = average_radius / radius.z;
    s_state.calibration_valid = true;
    g_mmc5983ma_debug.calibration_offset_ut = s_state.offset_ut;
    g_mmc5983ma_debug.calibration_scale = s_state.scale;
    g_mmc5983ma_debug.calibration_valid = true;
    return FC_STATUS_OK;
}

FcStatus_t Drv_Mmc5983ma_PerformSet(void)
{
#if !FC_USE_STM32_HAL
    return FC_STATUS_NOT_READY;
#else
    FcStatus_t status;

    if (!s_state.ready) { return FC_STATUS_NOT_INITIALIZED; }
    status = write_register(MMC5983MA_REG_CONTROL_2, 0U);
    if (status == FC_STATUS_OK)
    {
        status = write_register(MMC5983MA_REG_CONTROL_0,
                                MMC5983MA_CONTROL_0_SET);
    }
    if (status == FC_STATUS_OK)
    {
        HAL_Delay(FC_MMC5983MA_SET_DELAY_MS);
        status = write_register(MMC5983MA_REG_CONTROL_0,
                                MMC5983MA_CONTROL_0_AUTO_SR_ENABLE);
    }
    if (status == FC_STATUS_OK)
    {
        status = write_register(MMC5983MA_REG_CONTROL_2,
                                MMC5983MA_CONTROL_2_CONTINUOUS);
    }
    return status;
#endif
}

bool Drv_Mmc5983ma_IsReady(void)
{
    return s_state.ready;
}

FcStatus_t Drv_Mmc5983ma_GetDebug(DrvMmc5983maDebug_t *debug)
{
    if (debug == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    debug->product_id = g_mmc5983ma_debug.product_id;
    debug->init_status = g_mmc5983ma_debug.init_status;
    debug->last_read_status = g_mmc5983ma_debug.last_read_status;
    debug->valid_read_count = g_mmc5983ma_debug.valid_read_count;
    debug->failed_read_count = g_mmc5983ma_debug.failed_read_count;
    debug->saturated_read_count = g_mmc5983ma_debug.saturated_read_count;
    debug->calibration_offset_ut = g_mmc5983ma_debug.calibration_offset_ut;
    debug->calibration_scale = g_mmc5983ma_debug.calibration_scale;
    debug->calibration_sample_count = g_mmc5983ma_debug.calibration_sample_count;
    debug->calibration_active = g_mmc5983ma_debug.calibration_active;
    debug->calibration_valid = g_mmc5983ma_debug.calibration_valid;
    debug->ready = g_mmc5983ma_debug.ready;
    return s_state.ready ? FC_STATUS_OK : g_mmc5983ma_debug.init_status;
}
