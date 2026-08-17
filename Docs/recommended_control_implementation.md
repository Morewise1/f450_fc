# 推荐控制方案实现说明

本文记录 BMI088、FS-iA6B、ESC、姿态解算和 PID 的第一版软件约定。当前代码已通过主机测试；i-BUS UART、PWM 和 1 ms 定时器已经生成，BMI088 需要按实物模块改为 I2C2 后才能开始无桨验证。

## 已实现的数据链

```text
FS-iA6B i-BUS -> drv_ibus -> FcRcInput_t
                              |
                              v
                         ctl_rc_map -> FcPilotCommand_t
BMI088 I2C2 -> drv_bmi088 -> FcImuData_t -> est_attitude -> FcAttitude_t
                                              |
                                              v
                                      attitude/rate PID
                                              |
                                              v
                                       Quad-X mixer -> ESC PWM
```

Keil Watch 可直接查看：

```text
g_fc_flight_debug
g_ctl_rate_debug
```

`g_fc_flight_debug` 包含 BMI088 原始/物理量、roll/pitch/yaw、i-BUS 原始通道、映射后操纵意图、解锁状态、四路电机脉宽和安全故障。Watch 结构体仅用于观察，业务模块仍通过函数接口交换数据。

## BMI088 与角度

- 上电后执行 2000 个静止样本的陀螺零偏校准；校准期间禁止解锁。
- 仅在 STOP 状态且判定静止时缓慢跟踪温漂；READY/RUNNING 禁止更新零偏，避免把真实旋转误当成漂移。
- 使用六轴 Mahony 得到 roll、pitch 和相对 yaw。
- 无磁力计时 yaw 角会随时间漂移，这是六轴 IMU 的物理限制；yaw 角速度仍可用于偏航闭环。
- 机体系约定为 X 向前、Y 向右、Z 向下；水平静止时加速度 Z 应约为 `-1 g`。实际安装方向只改 `fc_board.h` 的轴来源和符号。

## 遥控与解锁

- CH1 roll，CH2 pitch，CH3 throttle，CH4 yaw，CH5 解锁，CH6 模式。先在 FS-i6 的 AUX CHANNELS 菜单把 SwD 分配给 CH5；接收机不会自动保证 CH5 就是 SwD。
- `FC_IBUS_ARM_ACTIVE_HIGH` 控制 SwD 极性。第一次无桨观察原始通道后确认；若拨杆逻辑相反，将其改为 `0U`。
- 中位摇杆使用连续死区和 expo，油门使用低端死区和非线性曲线。
- STABILIZE 的油门表示绝对推力强度，不表示上升速度。上升/下降意图留给将来的高度保持模式。
- 遥控失联、数据校验失败、SwD 未解锁或油门不在低位时均不能解锁。

## ESC 限制

- ESC 协议范围固定为 1000-2000 us。
- `FC_ESC_COMMAND_MAX_US` 是正常飞行允许的最大脉宽，不是电机真实 RPM。无转速传感器时不能可靠地用软件宏限定 RPM。
- `FC_MOTOR_TEST_MAX_US` 只限制无桨单电机测试，默认 1200 us。
- `FC_ENABLE_MOTOR_TEST` 默认关闭；测试时临时设为 `1U`，完成后必须恢复 `0U`。
- STOP/READY、初始化失败和任意安全故障均输出 1000 us。

## PID 参数

`ctl_pid` 已支持输出偏移、输入死区、积分/输出限幅、积分分离、变速积分、条件抗饱和、微分先行和不完全微分。高级功能均由 `fc_params.h` 的宏控制。

当前已接入 250 Hz 姿态 P 外环和 500 Hz 角速度 PID 内环，初始增益位于 `Config/fc_params.h`。这些数值只用于拆桨和限位架起调，不是经过实机辨识或试飞验证的最终参数。未完成传感器方向、电机顺序、旋向、遥控方向和闭环纠偏方向验证前，不能直接装桨自由试飞。

满横滚杆、机体水平静止时，当前初始参数产生约 `75 us` 的横滚控制量，左右电机组相差约 `150 us`，因此不会再被整数 PWM 舍入为四路同速。积分项独立限幅，并启用了积分分离、条件抗饱和、测量微分和 30 Hz 微分低通。

## 加入 Keil 的源码组

在 `F407_fc.uvprojx` 中建立或补齐以下组，并添加对应 `.c` 文件：

```text
App:       app_main.c app_scheduler.c app_flight.c app_safety.c
BSP:       bsp_esc_pwm.c bsp_battery_adc.c bsp_debug_uart.c
Radio:     drv_ibus.c
Sensors:   drv_bmi088.c drv_bmp388.c drv_mmc5983ma.c
Estimator: est_attitude.c est_altitude.c
Control:   ctl_rc_map.c ctl_pid.c ctl_rate.c ctl_attitude.c ctl_altitude.c ctl_mixer.c
```

Include Paths 至少加入：

```text
..\App
..\BSP
..\Config
..\Control
..\Drivers\Radio
..\Drivers\Sensors
..\Estimator
```

实物 BMI088-V1.0 使用 I2C2 PB10/PB11；原 SPI1 和双 CS 配置应从 CubeMX 删除。CubeMX 外设配置完成前保持 `FC_USE_STM32_HAL=0U` 只能验证安全 stub；外设句柄和 `fc_board.h` 映射全部完成后才改为 `1U`。不要为了通过编译虚构 HAL 句柄。
