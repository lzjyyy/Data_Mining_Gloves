# Task/inc/frameAssemblerTask.h 解析

## 文件职责

`frameAssemblerTask.h` 声明合帧任务入口和合帧统计结构体。合帧任务负责把 IMU Sensor 数据和 Touch Sensor 数据按时间戳组合成 RawFrame。

## 主要结构体

### `FrameAssemblerStats_t`

合帧任务运行统计。

| 字段 | 含义 |
| --- | --- |
| `assembled_frames` | 成功合成 RawFrame 的数量。 |
| `imu_wait_timeouts` | 等待 IMU 数据超时次数。 |
| `touch_wait_timeouts` | 等待 Touch 数据超时次数。 |
| `imu_stale_drops` | 丢弃过期 IMU 数据次数。 |
| `touch_stale_drops` | 丢弃过期 Touch 数据次数。 |
| `timestamp_mismatch_drops` | IMU 与 Touch 时间戳差距过大导致丢弃次数。 |
| `raw_alloc_failures` | RawFrame 分配失败次数。 |
| `raw_publish_failures` | RawFrame 发布失败次数。 |
| `last_frame_id` | 最近成功合帧的帧号。 |
| `last_time_diff_us` | 最近一次 IMU 和 Touch 时间戳差值。 |
| `last_status` | 最近一次合帧状态。 |

## 函数声明

### `FrameAssemblerTask(void *argument)`

合帧线程入口。

- 从 DataManager 获取 IMU 和 Touch 数据块。
- 时间戳满足阈值时组装 RawFrame 并发布给算法消费者。

### `FrameAssemblerTask_GetStats(FrameAssemblerStats_t *stats)`

读取合帧任务统计。

- 调试任务或诊断接口可以调用它观察合帧质量。
