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

