# F450 STM32F407 Flight Controller

这是 STM32F407 + HAL 的 F450 四旋翼飞控第一版工程。相邻的 `../F103_maincontrol/` 只作为架构和数据流参考，不复制其标准库外设代码，也不沿用 0-100 占空比式电机输出。

## 当前结果

当前具备裸机 cooperative scheduler、安全状态机、i-BUS、BMI088 SPI、4 路标准 PWM ESC、PID、Quad-X mixer、CMake 构建和主机端测试。

- scheduler 由 1 ms tick 释放 500/250/100/50/10 Hz 标志。
- 初始状态为 STOP，安全条件通过后依次进入 READY、RUNNING。
- 进入 READY、RUNNING和切换模式时 reset 控制 PID。
- ESC 公共单位固定为 us，范围 1000-2000 us，怠速默认 1100 us。
- PWM 驱动按每个通道的定时器计数频率计算 CCR，不假设 72 MHz 或 1 tick/us。
- App 通过独立输出许可控制解锁，BSP 不解释飞行状态。
- 未解锁、初始化失败、传感器无效和急停时保持四路 1000 us。
- BMI088 支持双芯片 ID、物理单位换算、轴映射和非阻塞陀螺校准。
- BMP390、VL53L1X、电池 ADC 和调试 UART 仍为安全 stub。
- ALT_HOLD 只有状态机和接口，当前不能进行实机定高飞行。
- 不包含 FreeRTOS、光流、GPS 或磁力计逻辑。

## 编译验证

```powershell
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

测试包括：

- 500/250/100/50/10 Hz scheduler 标志和 missed deadline。
- STOP/READY/RUNNING、模式切换、PID reset 和 failsafe。
- i-BUS 解析及无硬件安全状态。
- BMI088 fake-HAL、双 CS、轴映射、校准和通信错误。
- ESC fake-HAL、多定时器不同计数频率、1000/1500/2000 us 和启动失败回收。

## CubeMX/HAL 集成

1. 用具体 STM32F407 型号生成 HAL 工程。
2. 在 `main.c` 中调用 `App_Init()` 与 `App_Loop()`。
3. 1 ms 定时器的 `HAL_TIM_PeriodElapsedCallback()` 只调用 `App_Scheduler1msTick()`。
4. SPI1 生成 `hspi1`，并分别配置 BMI088 加速度计和陀螺仪 CS。
5. 配置四路 PWM，在 `fc_board.h` 映射 TIM 句柄、通道和计数频率。
6. 启用 `FC_USE_STM32_HAL=1` 后先完成无桨波形测试。

参考：

- [Docs/app_scheduler_and_state_machine.md](Docs/app_scheduler_and_state_machine.md)
- [Docs/esc_pwm_hal_integration.md](Docs/esc_pwm_hal_integration.md)
- [Docs/no_prop_motor_test.md](Docs/no_prop_motor_test.md)
- [Docs/bmi088_hal_integration.md](Docs/bmi088_hal_integration.md)

## 安全不变量

1. 电机输出只能使用 1000-2000 us，不存在旧工程的 0-100 输出单位。
2. STOP 和 READY 状态四路输出均为 1000 us。
3. STOP 不允许直接进入 RUNNING。
4. 输出许可关闭时，大于 1000 us 的命令会被拒绝。
5. 非法 motor ID 不写任何 CCR。
6. `StopAll()` 将四路写回 1000 us，并关闭输出许可。
7. 传感器无效、遥控失联、电池未知、调度异常或 IMU 未校准时禁止解锁。

三人分工与 PR 规则见 [Docs/team_workflow.md](Docs/team_workflow.md)。
