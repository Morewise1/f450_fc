# Quad-X Motor Layout

从机体上方向下看，机头朝前：

```text
          Front (+X)
      M4              M1
   Front Left     Front Right

      M3              M2
    Rear Left      Rear Right
          Rear
```

当前暂定 M1/M3 为 CCW，M2/M4 为 CW。实际旋向、桨型和 yaw 混控符号必须在拆桨电机测试后确认。电机索引和混控符号只允许在 `Config/fc_board.h` 修改。

当前 M144Z-M4 排针映射：M1=PA6/TIM3_CH1，M2=PA7/TIM3_CH2，M3=PB0/TIM3_CH3，M4=PB1/TIM3_CH4。PB8/PB9 未引出，不再用于 ESC。
