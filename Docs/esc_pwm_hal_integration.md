# ESC PWM HAL 集成

## 单位与安全边界

- 所有 BSP 接口单位固定为微秒 `us`。
- 停机和最小脉宽：1000 us。
- 最大脉宽：2000 us。
- `FC_ESC_IDLE_US` 默认 1100 us，可在构建配置中调整，但必须处于 1000-2000 us。
- 不支持旧 F103 工程的 0-100 数值范围。

初始化时，驱动先把四路 CCR 写成 1000 us，再调用 `HAL_TIM_PWM_Start()`。任何通道启动失败时，已经启动的通道会停止，驱动保持未初始化状态，App 不得解锁。

## CubeMX 定时器配置

可以使用一个定时器的四个通道，也可以使用多个定时器。所有通道应使用相同 PWM 帧率，但计数频率可以不同，只要对应的 `FC_ESC_Mx_COUNTER_HZ` 填写正确。

1. 在 CubeMX 时钟树中确认每个 TIM 的输入时钟 `TIM_input_clock`。
2. STM32F407 的 APB 预分频不为 1 时，TIM 时钟通常是对应 PCLK 的 2 倍，以 CubeMX 显示为准。
3. 选择计数频率并计算预分频：

```text
timer_counter_hz = TIM_input_clock / (PSC + 1)
PSC = TIM_input_clock / timer_counter_hz - 1
```

4. 根据 PWM 帧率计算周期：

```text
ARR = timer_counter_hz / FC_ESC_PWM_FRAME_HZ - 1
```

5. 使用 PWM Mode 1、Active High、CCR preload。
6. CubeMX 初始 Pulse 设置为 1000 us 对应的 CCR。
7. GPIO 速度选择足够驱动 PWM 信号即可，不需要超高频设置。

默认配置是 400 Hz。若实际 30A ESC 只明确支持传统 50 Hz，应把 `FC_ESC_PWM_FRAME_HZ` 改为 50 后重新计算 ARR。脉宽仍保持 1000-2000 us。

## us 到 CCR

每个通道独立使用：

```text
CCR = round(pulse_us * timer_counter_hz / 1,000,000)
```

示例：

| 计数频率 | 1000 us | 1500 us | 2000 us |
| --- | ---: | ---: | ---: |
| 500 kHz | 500 | 750 | 1000 |
| 1 MHz | 1000 | 1500 | 2000 |
| 2 MHz | 2000 | 3000 | 4000 |
| 4 MHz | 4000 | 6000 | 8000 |

不能直接假设 `CCR=pulse_us`，除非该通道的定时器计数频率恰好为 1 MHz。

## fc_board.h 映射

单定时器四通道示例：

```c
#define FC_ESC_M1_TIM_HANDLE  htim3
#define FC_ESC_M1_TIM_CHANNEL TIM_CHANNEL_1
#define FC_ESC_M2_TIM_HANDLE  htim3
#define FC_ESC_M2_TIM_CHANNEL TIM_CHANNEL_2
#define FC_ESC_M3_TIM_HANDLE  htim3
#define FC_ESC_M3_TIM_CHANNEL TIM_CHANNEL_3
#define FC_ESC_M4_TIM_HANDLE  htim3
#define FC_ESC_M4_TIM_CHANNEL TIM_CHANNEL_4

#define FC_ESC_TIMER_COUNTER_HZ 1000000U
```

多个定时器计数频率不同时，再分别定义：

```c
#define FC_ESC_M1_COUNTER_HZ 1000000U
#define FC_ESC_M2_COUNTER_HZ 1000000U
#define FC_ESC_M3_COUNTER_HZ 2000000U
#define FC_ESC_M4_COUNTER_HZ 2000000U
```

电机 ID 使用零基索引：M1=0、M2=1、M3=2、M4=3。重复映射到同一个 TIM 通道会导致初始化失败；非法 ID 直接返回错误且不写 CCR。

## 输出许可

BSP 不读取 `FC_STATE_STOP/READY/RUNNING`。安全层确认允许输出后调用：

```c
BSP_EscPwm_SetOutputEnabled(true);
```

停机、失控保护或急停调用：

```c
BSP_EscPwm_StopAll();
```

许可关闭时只接受最终限幅结果为 1000 us 的命令。任何高于 1000 us 的输出都会被拒绝并保持停机值。
