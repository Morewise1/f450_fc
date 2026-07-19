# 三人协作工作流

| 角色 | 主要目录 | 第一阶段任务 |
| --- | --- | --- |
| A：平台与安全 | `Core/`、`BSP/`、scheduler、safety | CubeMX、tick、ESC、ADC、故障保护 |
| B：传感器与估计 | `Drivers/Sensors/`、`Estimator/` | QMI8658C、轴映射、校准、姿态估计 |
| C：遥控与控制 | `Drivers/Radio/`、`Control/`、flight | i-BUS、PID、控制环、mixer |

`Config/` 与安全规则属于共享接口，至少一名非作者评审后合并；电机顺序、轴映射和解锁规则建议全员评审。

- `main` 禁止直接推送。
- 功能分支使用 `feature/<module>-<goal>`。
- 一个 Issue 对应一个分支和一个 Pull Request。
- CubeMX 重新生成代码单独提交。
- PR 必须说明输入输出单位、任务频率、错误路径和无桨测试结果。

