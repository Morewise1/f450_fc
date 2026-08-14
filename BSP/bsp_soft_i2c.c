/* Two independent, timeout-bounded GPIO software-I2C buses. */

#include <stddef.h>
#include "bsp_soft_i2c.h"
#include "fc_board.h"
#include "fc_config.h"

volatile BspSoftI2cDebug_t
    g_soft_i2c_debug[BSP_SOFT_I2C_BUS_COUNT];

#if FC_USE_STM32_HAL

typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
} SoftI2cHardware_t;

static const SoftI2cHardware_t s_hardware[BSP_SOFT_I2C_BUS_COUNT] = {
    {
        FC_BMP388_SOFT_I2C_SCL_GPIO_PORT,
        FC_BMP388_SOFT_I2C_SCL_PIN,
        FC_BMP388_SOFT_I2C_SDA_GPIO_PORT,
        FC_BMP388_SOFT_I2C_SDA_PIN
    },
    {
        FC_MMC5983MA_SOFT_I2C_SCL_GPIO_PORT,
        FC_MMC5983MA_SOFT_I2C_SCL_PIN,
        FC_MMC5983MA_SOFT_I2C_SDA_GPIO_PORT,
        FC_MMC5983MA_SOFT_I2C_SDA_PIN
    }
};

static bool bus_is_valid(BspSoftI2cBus_t bus)
{
    return (uint32_t)bus < (uint32_t)BSP_SOFT_I2C_BUS_COUNT;
}

static void enable_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t cycles_for_us(uint32_t microseconds)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) { cycles_per_us = 1U; }
    return cycles_per_us * microseconds;
}

static void delay_us(uint32_t microseconds)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = cycles_for_us(microseconds);
    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
        /* DWT gives a clock-independent delay without consuming a timer. */
    }
}

static void delay_half_period(void)
{
    delay_us(FC_SOFT_I2C_HALF_PERIOD_US);
}

static void scl_low(const SoftI2cHardware_t *hardware)
{
    HAL_GPIO_WritePin(hardware->scl_port, hardware->scl_pin, GPIO_PIN_RESET);
}

static void scl_release(const SoftI2cHardware_t *hardware)
{
    HAL_GPIO_WritePin(hardware->scl_port, hardware->scl_pin, GPIO_PIN_SET);
}

static void sda_low(const SoftI2cHardware_t *hardware)
{
    HAL_GPIO_WritePin(hardware->sda_port, hardware->sda_pin, GPIO_PIN_RESET);
}

static void sda_release(const SoftI2cHardware_t *hardware)
{
    HAL_GPIO_WritePin(hardware->sda_port, hardware->sda_pin, GPIO_PIN_SET);
}

static bool scl_is_high(const SoftI2cHardware_t *hardware)
{
    return HAL_GPIO_ReadPin(hardware->scl_port,
                            hardware->scl_pin) == GPIO_PIN_SET;
}

static bool sda_is_high(const SoftI2cHardware_t *hardware)
{
    return HAL_GPIO_ReadPin(hardware->sda_port,
                            hardware->sda_pin) == GPIO_PIN_SET;
}

static FcStatus_t wait_scl_high(BspSoftI2cBus_t bus)
{
    const SoftI2cHardware_t *hardware = &s_hardware[bus];
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout_cycles = cycles_for_us(FC_SOFT_I2C_STRETCH_TIMEOUT_US);

    while (!scl_is_high(hardware))
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles)
        {
            ++g_soft_i2c_debug[bus].timeout_count;
            return FC_STATUS_TIMEOUT;
        }
    }
    return FC_STATUS_OK;
}

static FcStatus_t stop_condition(BspSoftI2cBus_t bus)
{
    const SoftI2cHardware_t *hardware = &s_hardware[bus];
    FcStatus_t status;

    sda_low(hardware);
    delay_half_period();
    scl_release(hardware);
    status = wait_scl_high(bus);
    delay_half_period();
    sda_release(hardware);
    delay_half_period();
    return status;
}

static FcStatus_t recover_bus(BspSoftI2cBus_t bus)
{
    const SoftI2cHardware_t *hardware = &s_hardware[bus];
    FcStatus_t status;
    uint32_t pulse;

    ++g_soft_i2c_debug[bus].recovery_count;
    sda_release(hardware);
    scl_release(hardware);
    status = wait_scl_high(bus);
    if (status != FC_STATUS_OK) { return status; }

    for (pulse = 0U; (pulse < 9U) && !sda_is_high(hardware); ++pulse)
    {
        scl_low(hardware);
        delay_half_period();
        scl_release(hardware);
        status = wait_scl_high(bus);
        if (status != FC_STATUS_OK) { return status; }
        delay_half_period();
    }

    status = stop_condition(bus);
    if (status != FC_STATUS_OK) { return status; }
    return sda_is_high(hardware) ? FC_STATUS_OK : FC_STATUS_BUSY;
}

static FcStatus_t start_condition(BspSoftI2cBus_t bus)
{
    const SoftI2cHardware_t *hardware = &s_hardware[bus];
    FcStatus_t status;

    sda_release(hardware);
    scl_release(hardware);
    status = wait_scl_high(bus);
    if (status != FC_STATUS_OK) { return status; }
    if (!sda_is_high(hardware))
    {
        status = recover_bus(bus);
        if (status != FC_STATUS_OK) { return status; }
    }

    delay_half_period();
    sda_low(hardware);
    delay_half_period();
    scl_low(hardware);
    delay_half_period();
    return FC_STATUS_OK;
}

static FcStatus_t write_byte(BspSoftI2cBus_t bus, uint8_t value)
{
    const SoftI2cHardware_t *hardware = &s_hardware[bus];
    FcStatus_t status;
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        if ((value & mask) != 0U) { sda_release(hardware); }
        else { sda_low(hardware); }
        delay_half_period();
        scl_release(hardware);
        status = wait_scl_high(bus);
        if (status != FC_STATUS_OK) { return status; }
        delay_half_period();
        scl_low(hardware);
        delay_half_period();
    }

    sda_release(hardware);
    delay_half_period();
    scl_release(hardware);
    status = wait_scl_high(bus);
    if (status != FC_STATUS_OK) { return status; }
    delay_half_period();
    if (sda_is_high(hardware))
    {
        scl_low(hardware);
        ++g_soft_i2c_debug[bus].nack_count;
        return FC_STATUS_NOT_READY;
    }
    scl_low(hardware);
    delay_half_period();
    return FC_STATUS_OK;
}

static FcStatus_t read_byte(BspSoftI2cBus_t bus,
                            uint8_t *value,
                            bool acknowledge)
{
    const SoftI2cHardware_t *hardware = &s_hardware[bus];
    FcStatus_t status;
    uint32_t bit;
    uint8_t result = 0U;

    sda_release(hardware);
    for (bit = 0U; bit < 8U; ++bit)
    {
        result <<= 1U;
        delay_half_period();
        scl_release(hardware);
        status = wait_scl_high(bus);
        if (status != FC_STATUS_OK) { return status; }
        delay_half_period();
        if (sda_is_high(hardware)) { result |= 1U; }
        scl_low(hardware);
        delay_half_period();
    }

    if (acknowledge) { sda_low(hardware); }
    else { sda_release(hardware); }
    delay_half_period();
    scl_release(hardware);
    status = wait_scl_high(bus);
    if (status != FC_STATUS_OK) { return status; }
    delay_half_period();
    scl_low(hardware);
    sda_release(hardware);
    delay_half_period();
    *value = result;
    return FC_STATUS_OK;
}

static FcStatus_t finish_transaction(BspSoftI2cBus_t bus, FcStatus_t status)
{
    FcStatus_t stop_status = stop_condition(bus);
    if ((status == FC_STATUS_OK) && (stop_status != FC_STATUS_OK))
    {
        status = stop_status;
    }
    g_soft_i2c_debug[bus].last_status = status;
    return status;
}

FcStatus_t BSP_SoftI2c_Init(BspSoftI2cBus_t bus)
{
    const SoftI2cHardware_t *hardware;
    GPIO_InitTypeDef gpio = {0};
    FcStatus_t status;

    if (!bus_is_valid(bus)) { return FC_STATUS_INVALID_ARGUMENT; }
    if (bus == BSP_SOFT_I2C_BUS_BMP388)
    {
        FC_BMP388_SOFT_I2C_GPIO_CLOCK_ENABLE();
    }
    else
    {
        FC_MMC5983MA_SOFT_I2C_GPIO_CLOCK_ENABLE();
    }

    enable_cycle_counter();
    hardware = &s_hardware[bus];
    scl_release(hardware);
    sda_release(hardware);

    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Pin = hardware->scl_pin;
    HAL_GPIO_Init(hardware->scl_port, &gpio);
    gpio.Pin = hardware->sda_pin;
    HAL_GPIO_Init(hardware->sda_port, &gpio);

    status = recover_bus(bus);
    g_soft_i2c_debug[bus].initialized = status == FC_STATUS_OK;
    g_soft_i2c_debug[bus].last_status = status;
    return status;
}

FcStatus_t BSP_SoftI2c_MemRead(BspSoftI2cBus_t bus,
                               uint8_t address_7bit,
                               uint8_t reg,
                               uint8_t *data,
                               uint16_t length)
{
    FcStatus_t status;
    uint16_t index;

    if (!bus_is_valid(bus) || (address_7bit > 0x7FU) ||
        (data == NULL) || (length == 0U))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (!g_soft_i2c_debug[bus].initialized)
    {
        return FC_STATUS_NOT_INITIALIZED;
    }

    ++g_soft_i2c_debug[bus].transaction_count;
    status = start_condition(bus);
    if (status == FC_STATUS_OK)
    {
        status = write_byte(bus, (uint8_t)(address_7bit << 1U));
    }
    if (status == FC_STATUS_OK) { status = write_byte(bus, reg); }
    if (status == FC_STATUS_OK) { status = start_condition(bus); }
    if (status == FC_STATUS_OK)
    {
        status = write_byte(bus, (uint8_t)((address_7bit << 1U) | 1U));
    }
    for (index = 0U; (index < length) && (status == FC_STATUS_OK); ++index)
    {
        status = read_byte(bus, &data[index], index + 1U < length);
    }
    return finish_transaction(bus, status);
}

FcStatus_t BSP_SoftI2c_MemWrite(BspSoftI2cBus_t bus,
                                uint8_t address_7bit,
                                uint8_t reg,
                                const uint8_t *data,
                                uint16_t length)
{
    FcStatus_t status;
    uint16_t index;

    if (!bus_is_valid(bus) || (address_7bit > 0x7FU) ||
        (data == NULL) || (length == 0U))
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    if (!g_soft_i2c_debug[bus].initialized)
    {
        return FC_STATUS_NOT_INITIALIZED;
    }

    ++g_soft_i2c_debug[bus].transaction_count;
    status = start_condition(bus);
    if (status == FC_STATUS_OK)
    {
        status = write_byte(bus, (uint8_t)(address_7bit << 1U));
    }
    if (status == FC_STATUS_OK) { status = write_byte(bus, reg); }
    for (index = 0U; (index < length) && (status == FC_STATUS_OK); ++index)
    {
        status = write_byte(bus, data[index]);
    }
    return finish_transaction(bus, status);
}

#else

FcStatus_t BSP_SoftI2c_Init(BspSoftI2cBus_t bus)
{
    if ((uint32_t)bus >= (uint32_t)BSP_SOFT_I2C_BUS_COUNT)
    {
        return FC_STATUS_INVALID_ARGUMENT;
    }
    return FC_STATUS_NOT_IMPLEMENTED;
}

FcStatus_t BSP_SoftI2c_MemRead(BspSoftI2cBus_t bus,
                               uint8_t address_7bit,
                               uint8_t reg,
                               uint8_t *data,
                               uint16_t length)
{
    (void)bus;
    (void)address_7bit;
    (void)reg;
    (void)data;
    (void)length;
    return FC_STATUS_NOT_IMPLEMENTED;
}

FcStatus_t BSP_SoftI2c_MemWrite(BspSoftI2cBus_t bus,
                                uint8_t address_7bit,
                                uint8_t reg,
                                const uint8_t *data,
                                uint16_t length)
{
    (void)bus;
    (void)address_7bit;
    (void)reg;
    (void)data;
    (void)length;
    return FC_STATUS_NOT_IMPLEMENTED;
}

#endif /* FC_USE_STM32_HAL */
