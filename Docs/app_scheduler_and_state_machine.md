# App 调度器与飞行状态机

## 1 ms HAL 回调

CubeMX 配置一个 1 kHz 基本定时器。`App_Init()` 完成后启动中断：

```c
(void)App_Init();
HAL_TIM_Base_Start_IT(&htim6);
```

只在选定定时器的回调中释放调度标志：

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        App_Scheduler1msTick();
    }
}
```

`htim6` 必须替换成实际 1 ms 定时器。不要对所有定时器回调都调用 tick。回调中不得读取传感器、运行 PID、更新 ESC、调用 `HAL_Delay()` 或打印串口。

## 调度任务

| 标志 | 周期 | 工作 |
| --- | ---: | --- |
| 500 Hz | 2 ms | BMI088、rate PID、mixer、ESC |
| 250 Hz | 4 ms | 姿态估计、attitude PID |
| 100 Hz | 10 ms | i-BUS、battery、安全评估、状态机 |
| 50 Hz | 20 ms | BMP388、高度估计、ALT_HOLD PID |
| 10 Hz | 100 ms | 有界 housekeeping、日志快照 |

ISR 共享的任务位、tick、分频器和诊断计数均为 `volatile`。`App_SchedulerFetchReadyTasks()` 在短临界区中复制并清零任务位，避免 ISR 恰好在读取和清除之间释放新任务而导致丢标志。

同一任务尚未被主循环取走又再次释放时，任务位保持置位，同时 `missed_deadline_count` 饱和加一。出现 missed deadline 后 scheduler 状态为不健康，安全层禁止电机输出。

## MainLoop 顺序

```text
100 Hz safety/state
50 Hz altitude
250 Hz attitude
500 Hz rate/motor
10 Hz housekeeping/log snapshot
```

100 Hz 安全和状态转换先于同一 tick 的电机控制。STOP/READY 中可以更新估计器，但不执行控制 PID 输出，电机始终保持 1000 us。

## 状态切换表

| 当前状态 | 条件 | 下一状态 | 动作 |
| --- | --- | --- | --- |
| 启动 | `App_FlightInit()` | STOP | 输出许可关闭，四路 1000 us |
| STOP | RC/IMU/battery/scheduler/姿态均安全，arm 有效，油门低 | READY | reset rate、attitude、altitude PID，保持 1000 us |
| STOP | 任意条件不满足 | STOP | 保持 1000 us |
| READY | 安全正常，`throttle_low=false`，油门高于起飞阈值 | RUNNING | reset 全部 PID，先保持 1000 us，再由下一次 500 Hz 输出 |
| READY | 任意安全错误 | STOP | reset PID、关闭许可、1000 us |
| RUNNING | 安全正常 | RUNNING | 执行控制任务 |
| RUNNING | 任意安全错误 | STOP | reset PID、关闭许可、1000 us |

合法上升路径只有 `STOP -> READY -> RUNNING`。任何状态都可进入 STOP。状态转换函数内部会再次验证安全门槛，防止未来调用者绕过 100 Hz 状态机检查。

默认低油门解锁上限为 50，进入 RUNNING 的油门阈值为 100，单位都是归一化油门 0-1000。

## 模式切换

- 默认模式是 `FC_MODE_STABILIZE`。
- CH6 请求 `FC_MODE_ALT_HOLD`。
- 模式变化时 reset rate、attitude、altitude PID。
- 进入 ALT_HOLD 时锁定当前高度。
- ALT_HOLD 在每次模式检查时都要求高度有效，即使 CH6 一直没有变化。
- ALT_HOLD 中高度失效立即 STOP；高度恢复前不能重新进入 READY。

当前高度链仍需实物联调，因此 ALT_HOLD 不能视为已经具备实飞条件。

## 模块连接

```text
drv_ibus ----------> FcRcInput_t ---------+
drv_bmi088 --------> FcImuData_t ---------+--> app_safety --> output permission
bsp_battery_adc ---> FcBatteryStatus_t ----+
est_attitude ------> FcAttitude_t --------+

app_flight RUNNING --> PID/mixer --> FcMotorOutput_t --> bsp_esc_pwm
```

| 模块 | 调用频率 | 失败处理 |
| --- | ---: | --- |
| `drv_ibus` | 100 Hz | timeout/failsafe，STOP |
| `drv_bmi088` | 500 Hz | 本帧无效，立即 STOP |
| `bsp_battery_adc` | 100 Hz | unknown/critical，STOP |
| `est_attitude` | 250 Hz | 姿态无效，STOP |
| `drv_bmp388`、`est_altitude` | 50 Hz | STABILIZE禁用高度修正；ALT_HOLD失效时退回STABILIZE |
| `bsp_esc_pwm` | 状态转换、500 Hz | STOP/READY 1000 us；写入失败 STOP |

## 串口日志建议

ISR、500 Hz 和 250 Hz 任务中不格式化、不发送串口。10 Hz 任务只复制固定大小日志快照，发送交给 UART DMA 或低优先级非阻塞队列。

建议字段：

- `tick_ms`、flight state、flight mode。
- 状态或模式变化的旧值、新值和原因。
- `active_faults` 位图。
- RC link、arm、throttle、failsafe。
- IMU `valid/calibrated`。
- battery 电压、warning、critical。
- scheduler `missed_deadline_count`、`max_main_loop_time_us`。

周期日志限制在 10 Hz 或更低。状态变化和新故障可以立即写入内存队列，但不能在中断里直接打印。
