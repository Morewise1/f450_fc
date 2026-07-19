# FS-iA6B i-BUS Channel Map

| 通道 | 软件索引 | 用途 | 归一化范围 |
| --- | --- | --- | --- |
| CH1 | 0 | Roll | -500..500 |
| CH2 | 1 | Pitch | -500..500 |
| CH3 | 2 | Throttle | 0..1000 |
| CH4 | 3 | Yaw | -500..500 |
| CH5 | 4 | Arm / safety permission | 0/1 |
| CH6 | 5 | Stabilize / Alt Hold mode | 0/1 |

FS-iA6B 只有 6 个物理通道，因此当前工程把 CH5 同时映射为 `arm_switch` 和 `safety_switch`。`emergency_stop` 不从不存在的 CH7/CH8 推断，后续应由独立硬件急停或明确的遥控组合实现。

原始通道一般约为 1000-2000。驱动先检查合理范围，再限幅和归一化；校验失败或范围异常不会覆盖上一帧有效输入。

