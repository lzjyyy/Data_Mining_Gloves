# App/inc/app_config.h 解析

## 文件职责

`app_config.h` 是应用层的集中配置文件，定义手套系统的数据规模、内存池容量、消息队列深度、消费者数量以及数据有效标志位。它不直接实现函数，但会影响 `App`、`Services`、`Task` 三层的数据结构大小和运行容量。

## 主要配置

| 宏 | 含义 |
| --- | --- |
| `GLOVE_IMU_COUNT` | IMU 数量，当前为 16 路。 |
| `GLOVE_TOUCH_COUNT` | 触觉采样点数量，当前为 81 点。 |
| `GLOVE_JOINT_DOF_COUNT` | 关节自由度数量，当前为 21。 |
| `GLOVE_IMU_SENSOR_POOL_SIZE` | IMU Sensor 数据块内存池容量。 |
| `GLOVE_TOUCH_SENSOR_POOL_SIZE` | Touch Sensor 数据块内存池容量。 |
| `GLOVE_IMU_SENSOR_QUEUE_DEPTH` | IMU 数据送往合帧任务的队列深度。 |
| `GLOVE_TOUCH_SENSOR_QUEUE_DEPTH` | Touch 数据送往合帧任务的队列深度。 |
| `GLOVE_RAW_FRAME_POOL_SIZE` | RawFrame 内存池容量。 |
| `GLOVE_RAW_FRAME_QUEUE_DEPTH` | RawFrame 队列深度。 |
| `GLOVE_FULL_FRAME_POOL_SIZE` | FullFrame 内存池容量。 |
| `GLOVE_FULL_FRAME_QUEUE_DEPTH` | FullFrame 队列深度。 |
| `GLOVE_RAW_FRAME_CONSUMER_COUNT` | RawFrame 消费者数量，当前为 1。 |
| `GLOVE_FULL_FRAME_CONSUMER_COUNT` | FullFrame 消费者数量，当前为 2。 |

## 数据有效标志

| 宏 | 含义 |
| --- | --- |
| `GLOVE_FRAME_FLAG_NONE` | 无有效数据。 |
| `GLOVE_FRAME_FLAG_IMU_VALID` | IMU 原始数据有效。 |
| `GLOVE_FRAME_FLAG_QUAT_VALID` | IMU 四元数姿态有效。 |
| `GLOVE_FRAME_FLAG_TOUCH_VALID` | 触觉数据有效。 |
| `GLOVE_FRAME_FLAG_ALGORITHM_VALID` | 算法输出有效。 |

## 设计影响

- 内存池大小决定系统在高负载下能缓存多少帧数据。
- 队列深度决定生产者和消费者之间的缓冲能力。
- `GLOVE_FULL_FRAME_CONSUMER_COUNT` 当前与 `DataManager_PublishFullFrame()` 的两路发布逻辑一致，即 Storage 和 RS485。
- 修改 IMU、Touch 或关节数量会改变多个结构体尺寸，需要同步关注 RAM 占用。
