# App/inc/app_data.h 解析

## 文件职责

`app_data.h` 定义手套业务数据模型，是工程内数据流的基础头文件。采集任务、合帧任务、算法任务、存储任务和通信任务都围绕这里的结构体传递数据。

## 主要类型

| 类型 | 作用 |
| --- | --- |
| `GloveStatus_t` | 系统通用返回状态，例如成功、超时、无内存、参数错误、队列满等。 |
| `GloveHandSide_t` | 左右手标识，预留给标定和上报协议使用。 |
| `GloveVector3f_t` | 三维浮点向量，用于加速度、角速度等三轴物理量。 |
| `GloveQuaternion_t` | 四元数，用于表示 IMU 姿态。 |
| `GloveImuSample_t` | 单个 IMU 的原始采样，包含加速度、角速度、温度和状态位。 |
| `GloveTouchSample_t` | 单个触觉点的采样值和基线值。 |
| `GloveImuSensorData_t` | 一次 IMU 采集结果，包含 16 路 IMU 原始值和四元数。 |
| `GloveTouchSensorData_t` | 一次触觉阵列采集结果，包含 81 个触觉点。 |
| `GloveRawFrame_t` | 合帧后的原始帧，绑定一次 IMU 数据和一次 Touch 数据。 |
| `GloveProcessedFrame_t` | 算法处理结果帧，包含姿态和关节角/角速度。 |
| `GloveFullFrame_t` | 完整业务帧，包含 RawFrame 和 ProcessedFrame，供存储和 RS485 上报消费。 |
| `GloveDataStats_t` | DataManager 运行统计，记录发布数量、丢弃数量、内存池失败和队列失败。 |

## 函数声明

| 函数 | 作用 |
| --- | --- |
| `AppData_ClearImuSensorData()` | 清空 IMU Sensor 数据结构。 |
| `AppData_ClearTouchSensorData()` | 清空 Touch Sensor 数据结构。 |
| `AppData_ClearRawFrame()` | 清空 RawFrame。 |
| `AppData_ClearProcessedFrame()` | 清空算法结果帧。 |
| `AppData_ClearFullFrame()` | 清空完整帧。 |
| `AppData_BuildRawFrameFromSensors()` | 将 IMU 数据和 Touch 数据组装为 RawFrame。 |
| `AppData_BuildFullFrame()` | 将 RawFrame 和 ProcessedFrame 组装为 FullFrame。 |

## 数据流位置

`app_data.h` 不关心 RTOS 队列、内存池和任务调度，只定义业务数据本身。它是上层任务和下层服务之间的公共协议。
