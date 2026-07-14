# 自研夹爪 STM32F407 驱动说明

## 文件位置

- 驱动头文件：`Core/Inc/gripper.h`
- 驱动源文件：`Core/Src/gripper.c`
- FreeRTOS 接入：`Core/Src/freertos.c` 中的 `LeftGripperTask()` / `RightGripperTask()`
- Keil 工程：`MDK-ARM/STM32F407_V2_DW.uvprojx` 已加入 `gripper.c`

驱动参考 `自研夹爪` 文件夹下的 SDK，实现的是夹爪自定义 RS485 协议，不是通用 Modbus 寄存器协议。

```text
TX: 0xAE seq addr cmd len payload crc_l crc_h
RX: 0xAC seq addr cmd len payload crc_l crc_h
CRC: CRC16/MODBUS，低字节在前
串口: 115200, 8N1
```

## 这版工程做了什么

当前工程按“单左夹爪测试”配置：

```c
#define GRIPPER_LEFT_ENABLE               1
#define GRIPPER_RIGHT_ENABLE              0
#define GRIPPER_LEFT_ADDR                 GRIPPER_DEFAULT_ADDR
#define GRIPPER_RIGHT_ADDR                GRIPPER_DEFAULT_ADDR
#define GRIPPER_STARTUP_CALIB_ENABLE      1
#define GRIPPER_STARTUP_OPEN_LOOP_CALIB   1
#define GRIPPER_STATUS_POLL_ENABLE        1
#define GRIPPER_LEFT_POSITION_TEST_ENABLE 1
#define GRIPPER_READ_VERSION_ON_START     1
```

关键点：

- `MX_Modbus_Init()` 仍然保留在工程里，但左夹爪启用时不再启动 `L_ModbusH`/`USART1` 的 Modbus DMA 接收，避免通用 ModbusRTU task 抢走夹爪回包。
- `MX_Modbus_Init()` 放在 `MX_FREERTOS_Init()` 里执行，保证 Modbus task/queue 创建发生在 `osKernelInitialize()` 之后。
- 左夹爪 task 启动时会先接管 `USART1` 接收，停止可能残留的 UART DMA/IDLE 接收状态。
- 当前测试版已经打开回包：启动读版本、启动动作、测试动作和周期状态读取都会等待并解析夹爪返回的实时状态。
- 右夹爪 task、右夹爪定时器和右夹爪 Modbus DMA 默认关闭，避免没有右夹爪时一直报错。

## 上电测试现象

`LeftGripperTask()` 当前会自动执行一段测试：

1. 打印 `[L-Gripper] task start`。
2. 打印 `[L-Gripper] startup open-loop calibrate begin`。
3. 左夹爪向最大张开方向转动约 `GRIPPER_STARTUP_OPEN_MS`，默认 5000 ms。
4. 驱动发送停止命令，并调用 `Gripper_SetZero()`，把当前最大张开位置作为电机零点。
5. 驱动设置映射 `open = 0`，`close = -53000`。
6. 夹爪先移动到 `-53000`，也就是闭合端。
7. 进入位置测试：
   - 移动到 `-12000`
   - 移动到 `-30000`
   - 移动到 `100% open`，也就是 `0`
8. 打印 `[L-Gripper-Test] done` 后进入正常循环。

如果方向反了，先把 `GRIPPER_STARTUP_SEARCH_RPM` 改成负值，或者在 `gripper_startup_calibrate()` 里把搜索方向反过来。

## 常用接口

初始化：

```c
GripperHandle_t leftGripper;
Gripper_Init(&leftGripper, RS485A_CH, GRIPPER_DEFAULT_ADDR, 1000);
```

速度控制：

```c
GripperRealtime_t st;

ret = Gripper_SetSpeed(&leftGripper, 400.0f, 400000, &st);
ret = Gripper_SetSpeed(&leftGripper, 0.0f, 1000, &st);
```

电流控制：

```c
ret = Gripper_SetQCurrent(&leftGripper, 0.5f, 1000, &st);
```

位置控制：

```c
ret = Gripper_MoveAbsolute(&leftGripper, -26500, &st);
ret = Gripper_MoveToPercent(&leftGripper, 50.0f, &st);
ret = Gripper_Open(&leftGripper, &st);
ret = Gripper_Close(&leftGripper, &st);
```

如果只想发命令、不关心回包，也可以把最后一个参数传 `NULL`：

```c
ret = Gripper_MoveToPercent(&leftGripper, 50.0f, NULL);
```

## 参数单位

- 位置 `count`：电机多圈位置，`16384 count = 1 圈 = 360 deg`
- 速度 `rpm`：电机机械转速，协议 raw 值为 `rpm * 100`
- Q 轴电流 `amp`：单位 A，协议 raw 值为 `A * 1000`
- `Gripper_SetSpeed(..., accel_0p01rpm_per_sec, ...)`：加速度单位是 `0.01 rpm/s`，例如 `400000 = 4000 rpm/s`
- `Gripper_SetQCurrent(..., slope_milliamp_per_sec, ...)`：电流斜率单位是 `mA/s`，例如 `1000 = 1 A/s`

## 开合映射

映射逻辑来自 `自研夹爪/Gripper_sdk-x86-release/.../gripper_device.cpp`：

```c
count = close_count + percent / 100.0f * (open_count - close_count);
```

约定：

- `100%` 表示最大张开
- `0%` 表示闭合

SDK 默认值：

```c
GRIPPER_DEFAULT_OPEN_COUNT  = -3000
GRIPPER_DEFAULT_CLOSE_COUNT = -53000
```

本工程启动开环标定后，会把最大张开端设为零点，所以运行时使用：

```c
open_position_count  = 0
close_position_count = -53000
```

对应关系：

| 开度 | count |
| --- | ---: |
| 100% open | 0 |
| 75% open | -13250 |
| 50% open | -26500 |
| 25% open | -39750 |
| 0% open / close | -53000 |

同事只做位置控制时，推荐用：

```c
GripperRealtime_t st;
ret = Gripper_MoveToPercent(&leftGripper, 50.0f, &st);
```

或者直接发 count：

```c
ret = Gripper_MoveAbsolute(&leftGripper, -26500, &st);
```

## 标定说明

驱动里保留了完整闭环标定接口：

```c
GripperCalibrateConfig_t cfg;
GripperCalibrateResult_t result;

Gripper_CalibrateConfigDefault(&cfg);
cfg.search_direction = 1;
cfg.search_speed_rpm = 50.0f;
cfg.current_threshold_a = 0.6f;
cfg.timeout_ms = 8000;

ret = Gripper_CalibrateLimit(&leftGripper, &cfg, &result);
```

闭环标定逻辑是：给速度向最大张开方向运动，周期读取实时状态；当检测到速度接近 0、电流超过阈值、位置基本不变时，认为碰到机械极限，然后停止、置零、更新开合映射。

但这套闭环标定依赖 `Gripper_ReadRealtime()` 回包。当前现场优先把位置控制跑通，所以 `GRIPPER_STARTUP_OPEN_LOOP_CALIB = 1`，使用开环启动标定：固定向最大张开方向运行一段时间，然后把当前位置设为零点。

## 回包和状态

返回值定义：

```c
GRIPPER_OK          =  0
GRIPPER_ERROR       = -1
GRIPPER_INVALID_ARG = -2
GRIPPER_TIMEOUT     = -3
GRIPPER_CRC_ERROR   = -4
GRIPPER_BAD_FRAME   = -5
```

`-3` 表示等待回包超时。之前的主要风险是 USART1 同时被通用 ModbusRTU DMA 接收占用，夹爪驱动用阻塞 `HAL_UART_Receive()` 等回包时可能收不到数据。本版已经在左夹爪启用时跳过 USART1 的 Modbus DMA。

当前 `GRIPPER_STATUS_POLL_ENABLE = 1`，会周期读取实时状态，并把当前位置和故障码写入 `sys_status.left_gripper_status`。

```c
#define GRIPPER_STATUS_POLL_ENABLE 1
```

也可以手动读取：

```c
GripperRealtime_t st;
ret = Gripper_ReadRealtime(&leftGripper, &st);
```
本版收包做了两件事来提高稳定性：

- 每次需要回包的命令发送前，清掉 UART 接收残留。
- 接收时先同步到 `0xAC` 帧头，再校验 `seq/cmd/addr/CRC`，避免旧包或杂字节导致错帧。
