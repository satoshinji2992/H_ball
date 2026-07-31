# MCU 交接说明

MaixCAM 应用版本：`1.1.3`，安装包：`dist/h_ball_balance_v1.1.3.zip`。

## 功能分工

MaixCAM 识别钢球、计算位置和速度、管理滚球目标，通过 UART 单向发送误差和速度。
MCU 只需解析数据并控制摆杆，不需要知道 MaixCAM 当前模式或目标值。

滚球模式：

- `CENTER`：目标 `0cm`。
- `3-POINT`：进入后保持 `0cm`；触摸 `START` 后依次到 `+5cm`、`-5cm`。
- `FIXED`：触摸 `T-`、`T+` 设置目标，每次变化 `0.5cm`。

三点模式在位置误差不超过 `15mm`、速度不超过 `20mm/s` 并持续 500ms 后自动切换。

## 接线

```text
MaixCAM A16 / TX  ─────>  MCU UART RX
MaixCAM GND       ─────  MCU GND
```

串口配置：`115200 8N1`，无流控。当前为单向通信，不需要连接 MaixCAM RX。IO 不得
接入 5V 电平。

## 通信协议

每帧固定 11 个 ASCII 字节，无换行：

```text
[EEEEVVVV*]
```

| 字段 | 长度 | 含义 |
|---|---:|---|
| `EEEE` | 4 | `target_position_mm - ball_position_mm` |
| `VVVV` | 4 | 球速度，向 `CAL-R` 为正，单位 mm/s |

数值必须是符号加三位数字，范围 `-999..+999`：

```text
[-110+045*]   // error=-110mm, velocity=+45mm/s
[+000-007*]   // error=0mm,    velocity=-7mm/s
```

视觉无效时发送：

```text
[NaN NaN *]
```

两个字段必须同时有效或同时为 `NaN `。协议没有 CRC；解析器应严格检查帧头、帧尾、
符号和数字。收到新的 `[` 时重新同步。

## MCU 接入

直接加入以下文件：

```text
mcu_reference/vision_protocol.h
mcu_reference/vision_protocol.c
mcu_reference/control_core.h
mcu_reference/control_core.c
```

UART 接收数据后逐字节解析：

```c
vision_measurement_t next;

if (vision_parser_push(&parser, rx_byte, HAL_GetTick(), &next)) {
    latest_vision = next;
}
```

滚球控制调用示例：

```c
control_init(&control);
control_enable(&control, true);       // 状态切换时调用一次

control.vision = latest_vision;
output = control_tick(&control, HAL_GetTick(), dt_s);
set_rod_angle(output.servo_deg);
```

参考控制律：

```text
tilt = Kp × error - Kd × velocity + Ki × integral
```

收到 NaN、未使能，或超过 250ms 没有新有效帧时，`output.active=false`，摆杆输出中位。
参考增益只是初值，必须在实物上重新整定并保留机械角度限制。

## 联调检查

- 球在目标点：接近 `[+000+000*]`。
- 目标为 `0cm`、球在 `+5cm`：接近 `[-050+000*]`。
- 球向 `CAL-R` 运动：速度为正。
- 遮住球并经过短时预测窗口后：收到 `[NaN NaN *]`。
- UART 中断超过 250ms：摆杆回中。
- 三点模式按 `START` 后按 `+5 -> -5cm` 自动切换。
