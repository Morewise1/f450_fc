# IMU 坐标轴约定与测试

机体坐标系固定为：X 轴指向机头，Y 轴指向机体右侧，Z 轴指向下方。

- roll 正方向：绕 X 轴旋转，右侧机臂下沉。
- pitch 正方向：绕 Y 轴旋转，机头上仰。
- yaw 正方向：绕 Z 轴旋转，从上向下看为顺时针。
- 加速度单位：g。
- 角速度单位：deg/s。

传感器轴交换和符号处理只允许出现在 `Config/fc_board.h` 与 `drv_bmi088.c`。姿态估计和控制算法不得再次交换或取反坐标轴。

## 映射宏

```c
#define FC_IMU_BODY_X_SOURCE FC_AXIS_SOURCE_X
#define FC_IMU_BODY_Y_SOURCE FC_AXIS_SOURCE_Y
#define FC_IMU_BODY_Z_SOURCE FC_AXIS_SOURCE_Z
#define FC_IMU_BODY_X_SIGN   1.0f
#define FC_IMU_BODY_Y_SIGN   1.0f
#define FC_IMU_BODY_Z_SIGN   1.0f
```

模块旋转安装时只修改这些宏。三个 body 轴必须分别对应三个不同的传感器轴，符号只能是 `1.0f` 或 `-1.0f`；配置非法时初始化返回错误并禁止解锁。

自动测试故意使用 `body X=-sensor Y`、`body Y=sensor X`、`body Z=-sensor Z`，用于证明轴交换和符号处理确实集中在驱动层。

## 无桨坐标轴测试

1. 拆下全部桨叶，水平静置机架并完成陀螺仪校准。
2. 静止时 `gyro_dps` 三轴应接近 0，初步目标为绝对值小于 0.5 deg/s。
3. 静止时加速度模长应接近 1 g。水平放置时 `accel_g.z` 的正负取决于模块丝印朝向和映射，但必须与定义的 Z 向下一致。
4. 机头上仰时，主要变化应出现在 body Y，且 `gyro_dps.y` 为正。
5. 右侧机臂下沉时，主要变化应出现在 body X，且 `gyro_dps.x` 为正。
6. 从上向下看顺时针转动机体时，主要变化应出现在 body Z，且 `gyro_dps.z` 为正。
7. 分别让传感器六个方向朝下，对应加速度轴应接近正负 1 g，另外两轴接近 0。
8. 轴或符号错误时只修改 `fc_board.h`，然后重新执行全部测试。

完成坐标轴测试前，不得安装桨叶或进行 PID 飞行测试。
