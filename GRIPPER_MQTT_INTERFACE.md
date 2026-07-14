# MQTT gripper interface

本文档只说明当前工程里夹爪和上位机 MQTT 的对接约定。

## 命令入口

上位机通过 `robot/gripper/cmd` 下发夹爪命令。

接收入口在 `Core/Src/freertos.c`：

```c
static void messageArrived(MessageData* data);
```

真正执行夹爪命令的位置：

```c
static void gripper_execute(modbusHandler_t* h,
                            int position,
                            int speed,
                            int torque,
                            uint8_t select_gripper);
```

## 下发格式

```json
{
  "left":  { "position": 50, "speed": 200, "torque": 0 },
  "right": { "position": 50, "speed": 200, "torque": 0 }
}
```

`mode` 字段可带可不带，当前夹爪执行逻辑不依赖它。

字段说明：

- `position`：目标开合百分比，范围 `0~100`。`100` 是标定后的最大张开位置，`0` 是标定后的闭合位置。
- `speed`：位置运动限速，单位 rpm。`speed <= 0` 时使用默认 `GRIPPER_CMD_DEFAULT_SPEED_RPM`，当前为 `200 rpm`。
- `torque`：兼容旧字段，现在作为电流限幅使用。`torque <= 0` 时使用默认 `GRIPPER_CMD_DEFAULT_CURRENT_A`，当前为 `2.0 A`。`torque > 0` 时按 `0.01 A` 换算，例如 `100 = 1.0 A`，并限制到 `GRIPPER_CMD_MAX_CURRENT_A`。

当前 `gripper_execute()` 内部调用：

```c
Gripper_MoveToPercentWithLimits(gripper,
                                target_percent,
                                speed_rpm,
                                current_limit_a,
                                &realtime);
```

所以 MQTT 下发的 `speed` 已经会参与位置控制限速。

## 状态返回

上位机读取实时状态看 `robot/status`，发布入口在：

```c
static bool mqtt_publish_sys_status(SysStatus_t* status);
```

夹爪字段示例：

```json
{
  "left_gripper":  { "exp_pos": 50, "pos": 48, "reached": 1, "warning": 0 },
  "right_gripper": { "exp_pos": 50, "pos": 49, "reached": 1, "warning": 0 }
}
```

字段说明：

- `exp_pos`：最近一次上位机下发的目标百分比。
- `pos`：`Gripper_ReadRealtime()` 读到编码器 count 后，经 `Gripper_CountToPercent()` 映射出来的实时百分比。
- `reached`：当前位置是否进入目标死区。死区大小由 `GRIPPER_DEFAULT_DEADBAND_PERCENT` 控制。
- `warning`：夹爪驱动器回包里的 `fault_code`。

## 关键默认参数

这些宏都在 `Core/Src/freertos.c` 里：

```c
#define GRIPPER_CMD_DEFAULT_SPEED_RPM    200.0f
#define GRIPPER_CMD_MIN_SPEED_RPM        10.0f
#define GRIPPER_CMD_MAX_SPEED_RPM        500.0f
#define GRIPPER_CMD_DEFAULT_CURRENT_A    2.0f
#define GRIPPER_CMD_MAX_CURRENT_A        3.0f
```

位置死区在 `Core/Inc/gripper.h`：

```c
#define GRIPPER_DEFAULT_DEADBAND_PERCENT 2.0f
```

当前含义是目标左右各 `1%` 范围内认为到位。
