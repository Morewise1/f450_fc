# FS-iA6B i-BUS HAL Integration

## UART 参数

- 波特率：115200。
- 数据位：8。
- 校验：None。
- 停止位：1。
- 模式：RX 即可。
- 不使用软件流控。
- 接收机、飞控和电调必须共地。

连接前核对接收机端口标识和信号电平。若 i-BUS 信号超过所选 STM32F407 RX 引脚允许电压，应增加电平转换，不能只凭“常见模块”假设安全。

## 字节中断方式

HAL 句柄由 CubeMX 在 `Core` 中创建，驱动不创建句柄：

```c
extern UART_HandleTypeDef huart2;
static uint8_t ibus_rx_byte;

void IbusUart_Start(void)
{
    (void)Drv_Ibus_Init();
    HAL_UART_Receive_IT(&huart2, &ibus_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        (void)Drv_Ibus_ProcessByte(ibus_rx_byte, App_SchedulerGetTickMs());
        HAL_UART_Receive_IT(&huart2, &ibus_rx_byte, 1U);
    }
}
```

中断中只喂字节并重新启动接收，不打印、不做状态机和 PID。

## DMA Receive-to-Idle 方式

```c
extern UART_HandleTypeDef huart2;
static uint8_t ibus_dma_buffer[64];

void IbusDma_Start(void)
{
    (void)Drv_Ibus_Init();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ibus_dma_buffer, sizeof(ibus_dma_buffer));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart2)
    {
        (void)Drv_Ibus_ProcessBuffer(ibus_dma_buffer,
                                     (size_t)size,
                                     App_SchedulerGetTickMs());
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ibus_dma_buffer, sizeof(ibus_dma_buffer));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}
```

具体 HAL 版本对 Receive-to-Idle DMA 的重启要求可能不同，应以所用 STM32CubeF4 版本为准。

## App 集成

`App_FlightTask100Hz()` 保持以下职责边界：

```c
Drv_Ibus_UpdateTimeout(App_SchedulerGetTickMs());

if (Drv_Ibus_GetInput(&rc_input) != FC_STATUS_OK)
{
    rc_input.failsafe = true;
    rc_input.link_valid = false;
    rc_input.throttle_low = false;
}

App_SafetyEvaluate(&rc_input, &imu, &attitude, &battery, scheduler_ok);
```

驱动只输出通道、有效性和 failsafe；是否进入 STOP/READY/RUNNING 仍由 `app_safety` 与 `app_flight` 决定。

## 调试打印建议

只在主循环的 10 Hz 或更低频任务打印：

```text
IBUS ok=1250 crc=2 fmt=0 range=0 timeout=1 age=4ms
RC r=12 p=-8 y=3 thr=0 low=1 arm=0 mode=0 fs=0
```

建议打印 `Drv_Ibus_GetStats()`、归一化通道和帧龄。不要在 UART RX 中断/DMA 回调里调用 `printf()`；优先通过后续的非阻塞 `BSP_DebugUart_Write()` 发送固定长度日志。
