# CubeMX Core 集成

`Core/` 最终由 STM32CubeMX 生成。App、BSP、Drivers、Estimator 和 Control 目录应作为用户代码加入 CubeIDE、Keil 或 VS Code 构建系统。

## main.c

在 CubeMX 初始化 GPIO、DMA、ADC、UART、SPI 和 TIM 后初始化飞控，再启动 1 ms 调度定时器：

```c
(void)App_Init();
HAL_TIM_Base_Start_IT(&htim6);

while (1)
{
    App_Loop();
}
```

如果 `App_Init()` 返回错误，App 仍保持 STOP 和 1000 us，禁止解锁。实机调试时应记录初始化失败模块。

## 1 ms 回调

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        App_Scheduler1msTick();
    }
}
```

必须确认 TIM6 的 update event 精确为 1 kHz。若使用其他定时器，替换句柄并避免与 ESC PWM 定时器混淆。

回调中只调用 `App_Scheduler1msTick()`。不要在中断中调用 `App_Loop()`、打印日志、执行 `HAL_Delay()`、读取传感器或运行 PID。
