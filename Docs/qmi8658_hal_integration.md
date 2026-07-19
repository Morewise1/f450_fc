# QMI8658C SPI/HAL 集成

## 驱动配置

- `WHO_AM_I`：寄存器 `0x00`，QMI8658C 期望值 `0x05`。
- 软件复位：向寄存器 `0x60` 写入 `0xB0`，初始化阶段等待 10 ms。
- 加速度计：1000 Hz、正负 8 g、4096 LSB/g。
- 陀螺仪：1000 Hz、正负 2048 deg/s、16 LSB/(deg/s)。
- 温度：256 LSB/摄氏度。
- 飞控读取频率：500 Hz。

`CTRL1=0x60` 用于 4-wire SPI 和连续寄存器地址递增。寄存器配置应再次与实际采购批次的数据手册核对。`WHO_AM_I` 不匹配时不要修改期望值绕过检查，应先检查型号、SPI 模式、供电、CS 和焊接。

## CubeMX 配置

1. 启用 `SPI1`，生成全局句柄 `hspi1`。
2. 配置为主机、全双工、8-bit、MSB first、软件 NSS、SPI Mode 0。
3. 初次联调使用 1-5 MHz SPI 时钟，确认稳定后再依据数据手册调整。
4. CS 使用推挽 GPIO 输出，上电默认高电平。
5. QMI8658C 使用 3.3 V 供电和 3.3 V 逻辑，不接 5 V 信号。
6. HAL 工程定义 `FC_USE_STM32_HAL=1`。

CubeMX 若将 CS 引脚标签命名为 `QMI8658_CS`，可在 `fc_board.h` 中映射：

```c
extern SPI_HandleTypeDef hspi1;

#define FC_QMI8658_CS_GPIO_PORT QMI8658_CS_GPIO_Port
#define FC_QMI8658_CS_PIN       QMI8658_CS_Pin
#define FC_QMI8658_CS_LOW()  HAL_GPIO_WritePin(FC_QMI8658_CS_GPIO_PORT, FC_QMI8658_CS_PIN, GPIO_PIN_RESET)
#define FC_QMI8658_CS_HIGH() HAL_GPIO_WritePin(FC_QMI8658_CS_GPIO_PORT, FC_QMI8658_CS_PIN, GPIO_PIN_SET)
```

驱动只引用 CubeMX 创建的 `hspi1`，不会在模块内部创建 HAL 句柄。

## 初始化和校准

```c
FcStatus_t status = Drv_Qmi8658_Init();
if (status == FC_STATUS_OK)
{
    status = Drv_Qmi8658_CalibrateGyro();
}
```

`Drv_Qmi8658_CalibrateGyro()` 只启动校准，不阻塞。500 Hz 的 `Drv_Qmi8658_Read()` 累计 2000 个连续静止样本，约 4 秒。校准期间移动机体会清空累计值并重新开始。

校准偏置仅保存在 RAM。重新上电后必须再次校准。校准未完成时 `imu.valid=false`、`imu.calibrated=false`，安全层禁止解锁，电机保持 1000 us。

## 500 Hz 任务

工程中的 `App_FlightTask500Hz()` 使用如下 fail-closed 方式：

```c
void App_FlightTask500Hz(void)
{
    FcStatus_t status = Drv_Qmi8658_Read(&s_imu);

    if ((status != FC_STATUS_OK) || !s_imu.valid)
    {
        s_imu.valid = false;
        force_stop();
        return;
    }

    /* 有效数据才允许进入姿态估计和控制器。 */
}
```

高频读取路径不调用 `HAL_Delay()`。SPI 错误、超时、忙状态、校准进行中或疑似总线固定为全 `0x00/0xFF` 时，本帧保持全零且 `valid=false`，不会复用上一帧冒充新数据。

## 调试打印

驱动内部和 500 Hz 任务中不打印。可在 10-20 Hz 低频任务中输出：

- `Drv_Qmi8658_GetWhoAmI()` 和 `Drv_Qmi8658_IsReady()`。
- `accel_g.x/y/z`、`gyro_dps.x/y/z`、`temperature_c`。
- `valid`、`calibrated` 和安全故障位。

坐标轴实测见 `Docs/imu_axis_convention.md`。
