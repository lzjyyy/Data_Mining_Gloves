# 自研夹爪驱动接口手册

本文档说明当前 STM32F407 工程中自研夹爪驱动的常用接口、位置映射、死区逻辑、左右夹爪任务接入，以及 MQTT 相关代码位置。

## 文件

- 驱动头文件：`Core/Inc/gripper.h`
- 驱动源文件：`Core/Src/gripper.c`
- FreeRTOS 接入：`Core/Src/freertos.c`
- Keil 工程文件：`MDK-ARM/STM32F407_V2_DW.uvprojx`

驱动使用自研夹爪私有 RS485 协议，不是通用 Modbus 寄存器协议。夹爪串口需要由夹爪驱动独占，不能同时让 ModbusRTU DMA 接收任务占用同一个 UART。

## 初始化

```c
GripperHandle_t leftGripper;

Gripper_Init(&leftGripper, RS485A_CH, GRIPPER_DEFAULT_ADDR, 1000);
Gripper_SetPositionProfile(&leftGripper, 0, GRIPPER_DEFAULT_CLOSE_COUNT);
```

参数说明：

- `RS485A_CH`：左夹爪通道，对应 `USART1`
- `RS485B_CH`：右夹爪通道，对应 `USART3`
- `GRIPPER_DEFAULT_ADDR`：默认地址 `0x01`
- `timeout_ms`：等待回包超时，当前任务中使用 `1000 ms`

## 位置映射

工程标定后使用百分比开合：

```text
100% = 最大张开
0%   = 闭合
```

当前运行时映射：

```text
100% open = 0 count
75% open  = -13250 count
50% open  = -26500 count
25% open  = -39750 count
0% close  = -53000 count
```

换算函数：

```c
int32_t count = Gripper_PercentToCount(&leftGripper, 50.0f);
float percent = Gripper_CountToPercent(&leftGripper, realtime.multi_turn_count);
```

## 2% 目标死区

当前驱动默认启用 `2%` 死区：

```c
#define GRIPPER_DEFAULT_DEADBAND_PERCENT 2.0f
```

含义是目标位置左右各 `1%`。例如目标 `50%`，当前位置在 `49% ~ 51%` 内时，`Gripper_MoveToPercent()` 不再重复下发位置命令，只返回当前实时状态。

修改死区：

```c
Gripper_SetDeadbandPercent(&leftGripper, 2.0f);  // 左右各 1%
Gripper_SetDeadbandPercent(&leftGripper, 0.0f);  // 关闭软件死区
```

判断是否到达死区：

```c
uint8_t reached = Gripper_IsPercentInDeadband(&leftGripper,
                                              50.0f,
                                              realtime.multi_turn_count);
```

## 常用控制接口

位置控制，推荐上位机日常使用：

```c
GripperRealtime_t st;
GripperResult_t ret;

ret = Gripper_MoveToPercent(&leftGripper, 50.0f, &st);
ret = Gripper_MoveAbsolute(&leftGripper, -26500, &st);
```

速度控制：

```c
ret = Gripper_SetSpeed(&leftGripper, 50.0f, 10000, &st);
ret = Gripper_SetSpeed(&leftGripper, 0.0f, 1000, &st);
```

速度加电流限制：

```c
ret = Gripper_SetSpeedWithCurrentLimit(&leftGripper, 50.0f, 10000, 0.5f, &st);
```

电流控制：

```c
ret = Gripper_SetQCurrent(&leftGripper, 0.5f, 1000, &st);
```

停止输出：

```c
ret = Gripper_Disable(&leftGripper, &st);
```

读取实时状态：

```c
ret = Gripper_ReadRealtime(&leftGripper, &st);
```

实时状态里常用字段：

```c
st.multi_turn_count;  // 当前编码器多圈位置 count
st.speed_raw;         // raw / 100 = rpm
st.q_current_raw;     // raw / 1000 = A
st.fault_code;        // 0 表示无故障
```

## 左右夹爪任务

当前宏配置在 `Core/Src/freertos.c`：

```c
#define GRIPPER_LEFT_ENABLE              1
#define GRIPPER_RIGHT_ENABLE             1
#define GRIPPER_LEFT_ADDR                GRIPPER_DEFAULT_ADDR
#define GRIPPER_RIGHT_ADDR               GRIPPER_DEFAULT_ADDR
```

当前已启用左右两个夹爪，左夹爪走 `RS485A_CH / USART1`，右夹爪走 `RS485B_CH / USART3`。如果只接单左夹爪调试，可以临时改成：

```c
#define GRIPPER_RIGHT_ENABLE             0
```

右夹爪任务使用：

```c
Gripper_Init(&rightGripper, RS485B_CH, GRIPPER_RIGHT_ADDR, 1000);
gripper_startup_calibrate(&rightGripper, RIGHT_GRIPPER);
```

也就是右夹爪走 `USART3 / RS485B_CH`，和左夹爪同一套驱动、标定、回包、状态读取逻辑。

## MQTT 接收命令的位置

MQTT 订阅夹爪命令的位置在 `Core/Src/freertos.c`：

```c
MQTTSubscribe(&mqttClient, "robot/gripper/cmd", QOS0, messageArrived);
```

收到 MQTT 消息后的解析入口：

```c
static void messageArrived(MessageData* data)
```

在这个函数里，代码会解析：

```json
{
  "left":  { "position": -26500, "speed": 0, "torque": 0 },
  "right": { "position": -26500, "speed": 0, "torque": 0 }
}
```

然后分别放入：

```c
osMessageQueuePut(leftGripperQueueHandle, &cmd, 0, 0);
osMessageQueuePut(rightGripperQueueHandle, &cmd, 0, 0);
```

真正执行夹爪命令的位置：

```c
LeftGripperTask()
RightGripperTask()
```

任务里从队列取出命令后调用：

```c
gripper_execute(NULL, cmd.left.position, cmd.left.speed, cmd.left.torque, LEFT_GRIPPER);
gripper_execute(NULL, cmd.right.position, cmd.right.speed, cmd.right.torque, RIGHT_GRIPPER);
```

当前 `position` 按百分比使用，范围 `0~100`：

```text
100 = 最大张开
0   = 闭合
```

`gripper_execute()` 内部会调用 `Gripper_MoveToPercent()`，根据每次上电标定得到的 `open_position_count / close_position_count` 自动换算成编码器 count。

## MQTT 返回状态的位置

周期发布系统状态的位置：

```c
static bool mqtt_publish_sys_status(SysStatus_t* status)
```

当前发布 topic：

```c
robot/status
```

当前夹爪字段：

```json
"left_gripper":  {"exp_pos":..., "pos":..., "reached":..., "warning":...}
"right_gripper": {"exp_pos":..., "pos":..., "reached":..., "warning":...}
```

当前 `exp_pos` 和 `pos` 都是百分比，不再直接返回编码器 count：

```text
exp_pos = 上位机下发的目标开度百分比
pos     = 实时编码器 count 映射后的当前开度百分比
```

内部换算使用：

```c
float left_percent = Gripper_CountToPercent(&leftGripper,
                                            realtime.multi_turn_count);
float right_percent = Gripper_CountToPercent(&rightGripper,
                                             realtime.multi_turn_count);
```

MQTT 发布 JSON 的字段名没有改，只是字段含义从 count 改成了百分比。

## 返回值

```c
GRIPPER_OK          =  0
GRIPPER_ERROR       = -1
GRIPPER_INVALID_ARG = -2
GRIPPER_TIMEOUT     = -3
GRIPPER_CRC_ERROR   = -4
GRIPPER_BAD_FRAME   = -5
```

`-3` 一般是没有收到回包，要检查地址、A/B 线、串口通道、供电，以及该 UART 是否被其他 DMA 接收逻辑占用。
