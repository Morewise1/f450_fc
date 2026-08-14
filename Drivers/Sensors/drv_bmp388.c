/* BMP388 I2C driver using Bosch NVM compensation coefficients. */

#include <stddef.h>
#include "drv_bmp388.h"
#include "fc_board.h"
#include "fc_config.h"

#define BMP388_REG_CHIP_ID             0x00U
#define BMP388_REG_ERROR               0x02U
#define BMP388_REG_STATUS              0x03U
#define BMP388_REG_PRESSURE_XLSB       0x04U
#define BMP388_REG_PWR_CTRL            0x1BU
#define BMP388_REG_OSR                 0x1CU
#define BMP388_REG_ODR                 0x1DU
#define BMP388_REG_CONFIG              0x1FU
#define BMP388_REG_CALIB_DATA          0x31U
#define BMP388_REG_COMMAND             0x7EU

#define BMP388_COMMAND_SOFT_RESET      0xB6U
#define BMP388_STATUS_COMMAND_READY    0x10U
#define BMP388_STATUS_DATA_READY       0x60U
#define BMP388_ERROR_MASK              0x07U
#define BMP388_CALIB_DATA_LENGTH         21U
#define BMP388_SAMPLE_DATA_LENGTH          6U

/* Pressure x8, temperature x2, 50 Hz, selectable IIR, normal mode. */
#define BMP388_OSR_VALUE               0x0BU
#define BMP388_ODR_VALUE               0x02U
#define BMP388_CONFIG_VALUE            FC_BMP388_IIR_REGISTER_VALUE
#define BMP388_PWR_CTRL_VALUE          0x33U

typedef struct
{
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
    float t_lin;
} Bmp388Calibration_t;

typedef struct
{
    Bmp388Calibration_t calibration;
    uint8_t address_7bit;
    uint8_t chip_id;
    bool ready;
} Bmp388State_t;

static Bmp388State_t s_state;
volatile DrvBmp388Debug_t g_bmp388_debug;

#if FC_USE_STM32_HAL
static uint16_t make_u16(uint8_t low, uint8_t high)
{
    return (uint16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}

static int16_t make_i16(uint8_t low, uint8_t high)
{
    return (int16_t)make_u16(low, high);
}

static uint32_t make_u24(uint8_t low, uint8_t middle, uint8_t high)
{
    return ((uint32_t)high << 16U) | ((uint32_t)middle << 8U) | (uint32_t)low;
}

static void publish_debug(void)
{
    g_bmp388_debug.address_7bit = s_state.address_7bit;
    g_bmp388_debug.chip_id = s_state.chip_id;
    g_bmp388_debug.ready = s_state.ready;
}
#endif

static void reset_state(void)
{
    s_state = (Bmp388State_t){0};
    g_bmp388_debug = (DrvBmp388Debug_t){0};
    g_bmp388_debug.init_status = FC_STATUS_NOT_INITIALIZED;
    g_bmp388_debug.last_read_status = FC_STATUS_NOT_INITIALIZED;
}

static void invalidate_output(FcBarometerData_t *data, uint32_t timestamp_ms)
{
    *data = (FcBarometerData_t){0};
    data->timestamp_ms = timestamp_ms;
}

#if FC_USE_STM32_HAL
static FcStatus_t hal_status_to_fc(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) { return FC_STATUS_OK; }
    if (status == HAL_BUSY) { return FC_STATUS_BUSY; }
    if (status == HAL_TIMEOUT) { return FC_STATUS_TIMEOUT; }
    return FC_STATUS_ERROR;
}

static FcStatus_t read_registers(uint8_t reg, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U)) { return FC_STATUS_INVALID_ARGUMENT; }
    if (s_state.address_7bit == 0U) { return FC_STATUS_NOT_READY; }
    status = HAL_I2C_Mem_Read(&FC_BMP388_I2C_HANDLE,
                              (uint16_t)s_state.address_7bit << 1U,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              length,
                              FC_SENSOR_I2C_TIMEOUT_MS);
    return hal_status_to_fc(status);
}

static FcStatus_t write_register(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;

    if (s_state.address_7bit == 0U) { return FC_STATUS_NOT_READY; }
    status = HAL_I2C_Mem_Write(&FC_BMP388_I2C_HANDLE,
                               (uint16_t)s_state.address_7bit << 1U,
                               reg,
                               I2C_MEMADD_SIZE_8BIT,
                               &value,
                               1U,
                               FC_SENSOR_I2C_TIMEOUT_MS);
    return hal_status_to_fc(status);
}

static FcStatus_t probe_address(void)
{
    const uint8_t addresses[2] = {
        FC_BMP388_I2C_ADDRESS_LOW,
        FC_BMP388_I2C_ADDRESS_HIGH
    };
    uint32_t index;

    for (index = 0U; index < 2U; ++index)
    {
        uint8_t chip_id = 0U;
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&FC_BMP388_I2C_HANDLE,
                                                    (uint16_t)addresses[index] << 1U,
                                                    BMP388_REG_CHIP_ID,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    &chip_id,
                                                    1U,
                                                    FC_SENSOR_I2C_TIMEOUT_MS);
        if ((status == HAL_OK) && (chip_id == FC_BMP388_EXPECTED_CHIP_ID))
        {
            s_state.address_7bit = addresses[index];
            s_state.chip_id = chip_id;
            publish_debug();
            return FC_STATUS_OK;
        }
    }
    return FC_STATUS_NOT_READY;
}

static FcStatus_t load_calibration(void)
{
    uint8_t raw[BMP388_CALIB_DATA_LENGTH];
    Bmp388Calibration_t *calibration = &s_state.calibration;
    FcStatus_t status = read_registers(BMP388_REG_CALIB_DATA,
                                       raw,
                                       BMP388_CALIB_DATA_LENGTH);

    if (status != FC_STATUS_OK) { return status; }

    calibration->par_t1 = (float)make_u16(raw[0], raw[1]) * 256.0f;
    calibration->par_t2 = (float)make_u16(raw[2], raw[3]) / 1073741824.0f;
    calibration->par_t3 = (float)(int8_t)raw[4] / 281474976710656.0f;
    calibration->par_p1 = ((float)make_i16(raw[5], raw[6]) - 16384.0f) / 1048576.0f;
    calibration->par_p2 = ((float)make_i16(raw[7], raw[8]) - 16384.0f) / 536870912.0f;
    calibration->par_p3 = (float)(int8_t)raw[9] / 4294967296.0f;
    calibration->par_p4 = (float)(int8_t)raw[10] / 137438953472.0f;
    calibration->par_p5 = (float)make_u16(raw[11], raw[12]) * 8.0f;
    calibration->par_p6 = (float)make_u16(raw[13], raw[14]) / 64.0f;
    calibration->par_p7 = (float)(int8_t)raw[15] / 256.0f;
    calibration->par_p8 = (float)(int8_t)raw[16] / 32768.0f;
    calibration->par_p9 = (float)make_i16(raw[17], raw[18]) / 281474976710656.0f;
    calibration->par_p10 = (float)(int8_t)raw[19] / 281474976710656.0f;
    calibration->par_p11 = (float)(int8_t)raw[20] / 3.6893488147419103e19f;
    calibration->t_lin = 0.0f;
    return FC_STATUS_OK;
}

static float compensate_temperature(uint32_t raw_temperature)
{
    Bmp388Calibration_t *calibration = &s_state.calibration;
    float partial_data1 = (float)raw_temperature - calibration->par_t1;
    float partial_data2 = partial_data1 * calibration->par_t2;

    calibration->t_lin = partial_data2 +
                         (partial_data1 * partial_data1) * calibration->par_t3;
    return calibration->t_lin;
}

static float compensate_pressure(uint32_t raw_pressure)
{
    const Bmp388Calibration_t *calibration = &s_state.calibration;
    float t = calibration->t_lin;
    float t2 = t * t;
    float t3 = t2 * t;
    float raw = (float)raw_pressure;
    float raw2 = raw * raw;
    float partial_out1 = calibration->par_p5 +
                         (calibration->par_p6 * t) +
                         (calibration->par_p7 * t2) +
                         (calibration->par_p8 * t3);
    float partial_out2 = raw * (calibration->par_p1 +
                                (calibration->par_p2 * t) +
                                (calibration->par_p3 * t2) +
                                (calibration->par_p4 * t3));
    float nonlinear = raw2 * (calibration->par_p9 +
                              (calibration->par_p10 * t)) +
                      (raw2 * raw * calibration->par_p11);

    return partial_out1 + partial_out2 + nonlinear;
}
#endif /* FC_USE_STM32_HAL */

FcStatus_t Drv_Bmp388_Init(void)
{
    reset_state();

#if !FC_USE_STM32_HAL
    g_bmp388_debug.init_status = FC_STATUS_NOT_READY;
    return FC_STATUS_NOT_READY;
#else
    uint8_t status_register = 0U;
    uint8_t error_register = 0U;
    FcStatus_t status;

    HAL_Delay(FC_BMP388_STARTUP_DELAY_MS);
    status = probe_address();
    if (status == FC_STATUS_OK)
    {
        status = read_registers(BMP388_REG_STATUS, &status_register, 1U);
    }
    if ((status == FC_STATUS_OK) &&
        ((status_register & BMP388_STATUS_COMMAND_READY) == 0U))
    {
        status = FC_STATUS_BUSY;
    }
    if (status == FC_STATUS_OK)
    {
        status = write_register(BMP388_REG_COMMAND, BMP388_COMMAND_SOFT_RESET);
    }
    if (status == FC_STATUS_OK)
    {
        HAL_Delay(FC_BMP388_RESET_DELAY_MS);
        status = probe_address();
    }
    if (status == FC_STATUS_OK) { status = load_calibration(); }
    if (status == FC_STATUS_OK) { status = write_register(BMP388_REG_OSR, BMP388_OSR_VALUE); }
    if (status == FC_STATUS_OK) { status = write_register(BMP388_REG_ODR, BMP388_ODR_VALUE); }
    if (status == FC_STATUS_OK) { status = write_register(BMP388_REG_CONFIG, BMP388_CONFIG_VALUE); }
    if (status == FC_STATUS_OK) { status = write_register(BMP388_REG_PWR_CTRL, BMP388_PWR_CTRL_VALUE); }
    if (status == FC_STATUS_OK)
    {
        HAL_Delay(FC_BMP388_RESET_DELAY_MS);
        status = read_registers(BMP388_REG_ERROR, &error_register, 1U);
    }
    if ((status == FC_STATUS_OK) && ((error_register & BMP388_ERROR_MASK) != 0U))
    {
        status = FC_STATUS_INVALID_DATA;
    }

    s_state.ready = status == FC_STATUS_OK;
    g_bmp388_debug.init_status = status;
    g_bmp388_debug.last_read_status = s_state.ready ? FC_STATUS_NOT_READY : status;
    publish_debug();
    return status;
#endif
}

FcStatus_t Drv_Bmp388_Read(FcBarometerData_t *data, uint32_t timestamp_ms)
{
    if (data == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    invalidate_output(data, timestamp_ms);

#if !FC_USE_STM32_HAL
    return FC_STATUS_NOT_READY;
#else
    uint8_t status_register = 0U;
    uint8_t raw[BMP388_SAMPLE_DATA_LENGTH];
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    FcStatus_t status;

    if (!s_state.ready) { return FC_STATUS_NOT_INITIALIZED; }
    status = read_registers(BMP388_REG_STATUS, &status_register, 1U);
    if ((status == FC_STATUS_OK) &&
        ((status_register & BMP388_STATUS_DATA_READY) != BMP388_STATUS_DATA_READY))
    {
        status = FC_STATUS_BUSY;
    }
    if (status == FC_STATUS_OK)
    {
        status = read_registers(BMP388_REG_PRESSURE_XLSB,
                                raw,
                                BMP388_SAMPLE_DATA_LENGTH);
    }
    if (status == FC_STATUS_OK)
    {
        raw_pressure = make_u24(raw[0], raw[1], raw[2]);
        raw_temperature = make_u24(raw[3], raw[4], raw[5]);
        data->temperature_c = compensate_temperature(raw_temperature);
        data->pressure_pa = compensate_pressure(raw_pressure);
        if ((data->temperature_c < -40.0f) || (data->temperature_c > 85.0f) ||
            (data->pressure_pa < 30000.0f) || (data->pressure_pa > 125000.0f))
        {
            status = FC_STATUS_INVALID_DATA;
        }
    }

    data->valid = status == FC_STATUS_OK;
    g_bmp388_debug.last_read_status = status;
    if (status == FC_STATUS_OK) { ++g_bmp388_debug.valid_read_count; }
    else if (status != FC_STATUS_BUSY) { ++g_bmp388_debug.failed_read_count; }
    return status;
#endif
}

bool Drv_Bmp388_IsReady(void)
{
    return s_state.ready;
}

FcStatus_t Drv_Bmp388_GetDebug(DrvBmp388Debug_t *debug)
{
    if (debug == NULL) { return FC_STATUS_INVALID_ARGUMENT; }
    debug->address_7bit = g_bmp388_debug.address_7bit;
    debug->chip_id = g_bmp388_debug.chip_id;
    debug->init_status = g_bmp388_debug.init_status;
    debug->last_read_status = g_bmp388_debug.last_read_status;
    debug->valid_read_count = g_bmp388_debug.valid_read_count;
    debug->failed_read_count = g_bmp388_debug.failed_read_count;
    debug->ready = g_bmp388_debug.ready;
    return s_state.ready ? FC_STATUS_OK : g_bmp388_debug.init_status;
}
