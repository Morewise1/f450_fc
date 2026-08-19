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
- 只有 `RUNNING + 高度有效` 时才能进入 ALT_HOLD；进入时锁定当前高度。
- 垂直模式切换只 reset altitude PID，不重置 roll/pitch/yaw 控制环。
- 进入时从当前手动油门平滑混合到 `FC_HOVER_FEEDFORWARD_US + 高度PID修正`。
- 飞手必须先把油门放进捕获区；捕获区内保持目标高度，区间上/下方改变目标高度。
- CH6拨低后先等待油门回捕获区，再直接交权或按配置时间混合到手动油门。
- ALT_HOLD 中高度失效时锁存故障并退回 STABILIZE；CH6先拨低确认后才允许重试。
- 当前代码没有独立的自动起飞高度轨迹或自动着陆判定，不能把定高交接当作一键起降。

相关参数位于 `Config/fc_params.h`。首次实飞必须先无桨检查相位和油门连续性，再在受控环境测出实际悬停油门并调整 `FC_HOVER_FEEDFORWARD_US`。

高度估计模式由 `Config/fc_config.h` 的 `FC_ALT_ESTIMATOR_MODE` 选择。默认的
`FC_ALT_ESTIMATOR_MODE_KALMAN` 是三状态卡尔曼（高度、垂直速度、竖直加速度零偏），使用250 Hz惯性预测和50 Hz气压修正；
动态互补模式保留用于A/B回放，但不是当前飞行默认值。修改模式后必须在Keil中Rebuild并重新烧录。Watch查看 `g_est_altitude_debug`：

- `estimator_mode`：0固定互补、1三状态卡尔曼、2动态互补。
- `inertial_predicted_altitude_m`、`inertial_predicted_velocity_mps`：BMI088预测值。
- `barometer_altitude_m`、`barometer_velocity_mps`：BMP388高度及滑窗拟合速度。
- `adaptive_height_weight_n`、`adaptive_velocity_weight_m`：当前动态权重，越接近1越信惯性。
- `barometer_velocity_sample_count`：气压速度窗口已积累的样本数，起飞后达到配置窗口才使用气压速度。

FAST_TELEMETRY v1.2保持原0..41字节不变，在负载偏移42追加4字节小端
`int32`目标高度，缩放为`米×100`，所以完整负载为46字节。有效性位图bit 8仅在
ALT_HOLD且高度估计有效时置1；上位机应在该位为0时把目标高度曲线视为无效。
协议帧还包含12字节固定头和2字节CRC，因此UART实际发送的完整帧长度为60字节。

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
