# BMI088-V1.0 I2C/HAL 集成

实物模块只引出了 `INT4 INT3 INT2 INT1 SCL SDA GND 3.3V`，属于 I2C 接口版本，不支持当前工程原先设计的 SPI 双 CS 接线。

## 接线

| BMI088-V1.0 | STM32F407ZGT6 | 说明 |
|---|---|---|
| 3.3V | 3.3V | 禁止接 5V |
| GND | GND | 必须共地 |
| SCL | PB10 / I2C2_SCL | 时钟 |
| SDA | PB11 / I2C2_SDA | 数据 |
| INT1-INT4 | 不接 | 第一阶段轮询读取 |

ESC PWM 已迁移到 TIM3：PA6、PA7、PB0、PB1。BMI088 继续使用 I2C2 的 PB10/PB11，与四路电机输出不冲突。

模块照片上能看到上拉电阻，但首次通电前仍建议用万用表电阻档测量 SCL 到 3.3V、SDA 到 3.3V。几千到十几千欧通常表示板载上拉存在；若接近开路，再外接两只 4.7 kOhm 上拉到 3.3V。

## CubeMX

1. 禁用 SPI1；PA5 恢复为未使用，PA6/PA7 改为 TIM3_CH1/TIM3_CH2。
2. 将原 PA4 `BMI088_ACC_CS`、PC4 `BMI088_GYRO_CS` 恢复为未使用。
3. 启用 `I2C2 -> I2C`，选择 PB10=SCL、PB11=SDA。
4. 设置 Fast Mode 400 kHz、7-bit addressing、Dual Address Disabled、General Call Disabled、Clock Stretching Enabled。
5. GPIO 使用 Alternate Function Open Drain，No Pull。上拉由模块提供。
6. 不启用 I2C 中断和 DMA。驱动在 500 Hz 任务中使用短时阻塞 HAL 读写。
7. 重新生成后确认存在 `I2C_HandleTypeDef hi2c2`、`MX_I2C2_Init()`，并且 Keil 工程含 `stm32f4xx_hal_i2c.c`。

100 kHz 对每次读取加速度、角速度和温度的 500 Hz 周期余量不足，因此使用 400 kHz，并保持接线尽量短。

## 地址和芯片 ID

BMI088 内部有两个 I2C 从设备。驱动会自动探测：

```text
Accelerometer address: 0x18 or 0x19, CHIP_ID = 0x1E
Gyroscope address:     0x68 or 0x69, CHIP_ID = 0x0F
```

不需要根据模块焊盘猜地址。Keil Watch 查看：

```text
g_bmi088_debug.accel_address_7bit
g_bmi088_debug.gyro_address_7bit
g_bmi088_debug.chip_ids.accel
g_bmi088_debug.chip_ids.gyro
g_bmi088_debug.init_status
g_bmi088_debug.last_read_status
g_bmi088_debug.valid_read_count
g_bmi088_debug.failed_read_count
g_bmi088_debug.ready
g_bmi088_debug.calibrated
```

正常值应为芯片 ID `0x1E/0x0F`、`ready=1`。静止约 4 秒完成 2000 个样本后 `calibrated=1`。

## 第一次无桨验证

1. 只连接 3.3V、GND、SCL、SDA，不接电调和桨。
2. 模块水平静止，下载后不要移动约 5 秒。
3. 确认两个 CHIP_ID 和地址正确，`valid_read_count` 增长，`failed_read_count` 不持续增长。
4. 查看 `g_fc_flight_debug.imu.accel_g`，合加速度约 1 g；校准后 `gyro_dps` 接近 0。
5. 再做前倾、右倾和偏航动作，确认轴方向。模块没有清晰轴标时，不提前猜测 `fc_board.h` 的轴交换和符号。
