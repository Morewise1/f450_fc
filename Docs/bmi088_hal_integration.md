# BMI088 SPI/HAL 集成

BMI088 的加速度计和陀螺仪是两个独立 SPI 设备。它们共享 SCK、MISO、MOSI，但需要两个独立且默认拉高的 CS。

## 建议引脚

| BMI088 | STM32F407ZGT6 | CubeMX 标签 |
|---|---|---|
| SCK | PA5 / SPI1_SCK | - |
| SDO/MISO | PA6 / SPI1_MISO | - |
| SDI/MOSI | PA7 / SPI1_MOSI | - |
| CS_ACCEL / CSB1 | PA4 / GPIO Output | `BMI088_ACC_CS` |
| CS_GYRO / CSB2 | PC4 / GPIO Output | `BMI088_GYRO_CS` |
| VCC/VDDIO | 3.3 V | - |
| GND | GND | - |

模块上的 CS 可能写成 `CS_A/CS_G`、`CSB1/CSB2` 或 `ACC_CS/GYRO_CS`，接线前应按模块丝印或原理图确认。第一阶段不接中断引脚。

## CubeMX 配置

1. 启用 `SPI1 -> Full-Duplex Master`。
2. 使用 PA5、PA6、PA7，Hardware NSS 设为 Disable/Software。
3. 配置为 8 bit、MSB first、CPOL Low、CPHA 1 Edge。
4. Baud Rate Prescaler 先设为 16。当前 16 MHz 时为 1 MHz；未来 SPI1 时钟为 84 MHz 时为 5.25 MHz，仍低于 10 MHz。
5. PA4 和 PC4 配成 GPIO Output、Push-Pull、No Pull、Low Speed、初始 High，并使用上表标签。
6. 生成代码后确认存在全局 `SPI_HandleTypeDef hspi1`。

## 驱动行为

- 加速度计 `CHIP_ID` 应为 `0x1E`，陀螺仪应为 `0x0F`。
- 驱动已经处理加速度计 SPI 读操作所需的 dummy byte。
- 任意一个芯片初始化或读取失败时，`imu.valid` 都是 `false`，安全状态机保持 STOP。
- 初始化中的复位和上电等待允许使用 `HAL_Delay()`；500 Hz 读取路径不使用延时。

## Keil 配置

把 `Drivers/Sensors/drv_bmi088.c` 加入工程，并加入以下 Include Paths：

```text
../App
../BSP
../Config
../Control
../Drivers/Radio
../Drivers/Sensors
../Estimator
```

在 Preprocessor Symbols 中加入：

```text
FC_USE_STM32_HAL=1
```

第一次硬件测试只读取两个 CHIP_ID 和一帧 IMU 数据，始终无桨。平放静止时加速度合成值应约为 1 g，陀螺仪校准后应接近 0 dps。
