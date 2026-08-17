# Hardware Wiring

| 模块 | 电气要求 | 计划接口 |
| --- | --- | --- |
| BMI088 | 3.3 V，确认模块逻辑电平 | I2C2：PB10=SCL、PB11=SDA |
| BMP388 | 3.3 V | 软件I2C：PB6=SCL、PB7=SDA |
| MMC5983MA | 3.3 V，远离动力线 | 软件I2C：PC0=SCL、PC1=SDA |
| FS-iA6B | 核对 i-BUS 输出电平 | UART RX |
| 4 路 ESC | 飞控与 ESC 必须共地 | 4 路定时器 PWM 信号 |
| 3S 电池检测 | ADC 端不得超过 3.3 V | ADC 分压 |

飞控和接收机使用独立 5 V UBEC。若 ESC 带 BEC，红色 5 V 线不要未经确认并联；通常只连接 ESC 信号线和地线到飞控。四个 ESC 的动力输入直接来自电池/PDB，不经过飞控开发板。
