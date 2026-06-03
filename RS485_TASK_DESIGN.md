# RS485Task 设计说明

## 任务目标

`RS485Task` 负责把算法生成的 `FullFrame` 通过 RS485 发送出去，同时支持接收上位机指令。它需要完成：

- 从 `DataManager` 获取 `FullFrame`
- 将原始数据和算法结果封装成 RS485 协议帧
- 计算 CRC
- 控制 RS485 方向引脚
- 使用 UART DMA 或非阻塞方式发送
- 接收 UART 空闲中断产生的数据帧
- 区分接收事件、发送完成事件和错误事件
- DMA 发送完成或异常后释放 `FullFrame`

## 推荐整体架构

建议采用“事件驱动 + 待发送队列 + DMA 收发回调”的结构。

```text
AlgorithmTask
    -> DataManager_PublishFullFrame()
    -> DataManager FullFrame Queue
    -> RS485Task

UART RX DMA/Idle Callback
    -> 投递 RS485_EVENT_RX_FRAME
    -> RS485Task 解析命令

UART TX DMA Complete Callback
    -> 投递 RS485_EVENT_TX_DONE
    -> RS485Task 切回接收态并释放发送资源

UART/DMA Error Callback
    -> 投递 RS485_EVENT_UART_ERROR / RS485_EVENT_DMA_ERROR
    -> RS485Task 统计错误并恢复 DMA
```

## 发送模式

建议支持三种模式：

| 模式 | 含义 |
| --- | --- |
| `RS485_MODE_STREAM` | 主动连续发送 `FullFrame`。 |
| `RS485_MODE_PAUSE` | 暂停主动发送，只响应上位机命令。 |
| `RS485_MODE_SINGLE` | 收到单帧指令后只发送一帧。 |

这样既能满足实时上传，也方便上位机控制和调试。

## 消息队列设计

建议使用以下队列：

| 队列 | 内容 | 写入者 | 读取者 |
| --- | --- | --- | --- |
| DataManager FullFrame 队列 | `GloveFullFrameBlock_t *` | AlgorithmTask / DataManager | RS485Task |
| RS485 event queue | `RS485_Event_t` | UART/DMA 回调、软件定时器 | RS485Task |
| RS485 high TX queue | ACK、ERROR、STATUS 响应包 | RS485Task | RS485Task |
| RS485 normal TX queue | FullFrame 数据包 | RS485Task | RS485Task |

推荐队列深度：

```c
#define RS485_EVENT_QUEUE_DEPTH       8U
#define RS485_TX_HIGH_QUEUE_DEPTH     4U
#define RS485_TX_NORMAL_QUEUE_DEPTH   4U
```

发送时先取高优先级 TX 队列，再取普通 TX 队列：

```text
high TX queue 有数据 -> 先发送 ACK/ERROR/STATUS
normal TX queue 有数据 -> 再发送 FullFrame
```

这样上位机命令响应不会被连续数据流堵住。

## 事件类型设计

不要让 `RS485Task` 去猜事件来源。事件来源应该在回调里明确编码。

```c
typedef enum
{
    RS485_EVENT_RX_FRAME = 0,
    RS485_EVENT_TX_DONE,
    RS485_EVENT_UART_ERROR,
    RS485_EVENT_DMA_ERROR,
    RS485_EVENT_TX_TIMEOUT,
    RS485_EVENT_TX_REQUEST
} RS485_EventType_t;

typedef struct
{
    RS485_EventType_t type;
    uint32_t tick;
    uint32_t arg0;
    uint32_t arg1;
} RS485_Event_t;
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `type` | 事件类型，用于区分接收、发送完成、错误等来源。 |
| `tick` | 事件产生时的 RTOS tick，用于调试事件顺序。 |
| `arg0` | 事件参数，例如接收长度或错误码。 |
| `arg1` | 备用参数。 |

事件来源：

| 事件 | 产生位置 | `arg0` 建议内容 |
| --- | --- | --- |
| `RS485_EVENT_RX_FRAME` | UART 接收空闲回调 | 接收长度。 |
| `RS485_EVENT_TX_DONE` | UART DMA 发送完成回调 | 0。 |
| `RS485_EVENT_UART_ERROR` | UART 错误回调 | `huart->ErrorCode`。 |
| `RS485_EVENT_DMA_ERROR` | DMA 错误回调 | DMA 错误码。 |
| `RS485_EVENT_TX_TIMEOUT` | 软件定时器或任务超时检测 | 超时时长或当前状态。 |
| `RS485_EVENT_TX_REQUEST` | 新发送消息入队后 | 0。 |

### `RS485_EVENT_TX_REQUEST` 的含义

`RS485_EVENT_TX_REQUEST` 只是一个“提醒任务检查是否可以发送”的事件，不代表可以立刻启动 DMA。

收到该事件后必须进入 `RS485_TryStartNextTx()`，由当前状态决定是否发送：

```text
当前正在 TX_BUSY
    -> 不启动新 DMA

当前正在 RECOVERING
    -> 不启动新 DMA

当前处于 RX_IDLE
    -> 检查 high TX queue
    -> 检查 normal TX queue
    -> 根据模式决定是否主动取 FullFrame
```

这样可以避免：

- 上一次 DMA 未完成时重复调用 `HAL_UART_Transmit_DMA()`
- 当前发送上下文 `s_current_tx` 被覆盖
- FullFrame 释放错对象
- RS485 DE 引脚方向混乱
- HAL 返回 `HAL_BUSY` 后状态机仍误认为发送已启动

## 发送消息结构

```c
typedef enum
{
    RS485_TX_MSG_FULL_FRAME = 0,
    RS485_TX_MSG_ACK,
    RS485_TX_MSG_STATUS,
    RS485_TX_MSG_ERROR
} RS485_TxMsgType_t;

typedef enum
{
    RS485_TX_REASON_STREAM_FRAME = 0,
    RS485_TX_REASON_CMD_ACK,
    RS485_TX_REASON_CMD_STATUS,
    RS485_TX_REASON_CMD_ERROR,
    RS485_TX_REASON_SINGLE_FRAME,
    RS485_TX_REASON_INTERNAL_ERROR
} RS485_TxReason_t;

typedef struct
{
    RS485_TxMsgType_t type;
    RS485_TxReason_t reason;
    GloveFullFrameBlock_t *full;
    uint8_t data[RS485_TX_BUFFER_SIZE];
    uint16_t len;
} RS485_TxMsg_t;
```

设计要点：

- `data` 保存已经封装好的待发送字节流。
- `len` 是实际发送长度。
- `full` 只在 `RS485_TX_MSG_FULL_FRAME` 时有效。
- `full` 的作用是让 DMA 发送完成后能释放对应的 `FullFrame`。
- `type` 表示“发的是什么包”。
- `reason` 表示“为什么要发这个包”。

发送原因建议这样区分：

| reason | 含义 | 典型队列 | 是否关联 FullFrame |
| --- | --- | --- | --- |
| `RS485_TX_REASON_STREAM_FRAME` | 主动连续上报 FullFrame。 | normal TX queue | 是 |
| `RS485_TX_REASON_CMD_ACK` | 收到合法指令后的 ACK。 | high TX queue | 否 |
| `RS485_TX_REASON_CMD_STATUS` | 状态查询回应。 | high TX queue | 否 |
| `RS485_TX_REASON_CMD_ERROR` | 指令格式、CRC 或命令错误回应。 | high TX queue | 否 |
| `RS485_TX_REASON_SINGLE_FRAME` | 上位机请求单帧数据。 | high 或 normal TX queue | 是 |
| `RS485_TX_REASON_INTERNAL_ERROR` | 内部错误主动上报。 | high TX queue | 否 |

`type` 和 `reason` 需要同时存在。例如：

```text
type = RS485_TX_MSG_FULL_FRAME
reason = RS485_TX_REASON_STREAM_FRAME
    -> 主动上报数据帧

type = RS485_TX_MSG_FULL_FRAME
reason = RS485_TX_REASON_SINGLE_FRAME
    -> 上位机请求的一帧数据

type = RS485_TX_MSG_ACK
reason = RS485_TX_REASON_CMD_ACK
    -> 指令 ACK 回应
```

发送完成释放资源时，应优先根据 `reason` 判断是否需要释放 `FullFrame`：

```c
static void RS485_ReleaseTxMsg(RS485_TxMsg_t *msg)
{
    if (msg == NULL)
    {
        return;
    }

    if (((msg->reason == RS485_TX_REASON_STREAM_FRAME) ||
         (msg->reason == RS485_TX_REASON_SINGLE_FRAME)) &&
        (msg->full != NULL))
    {
        (void)DataManager_ReleaseFullFrame(msg->full);
        msg->full = NULL;
    }
}
```

## FullFrame 生命周期

这是 RS485Task 最关键的规则。

```text
DataManager_GetFullFrame() 成功
    -> RS485_PackFullFrame() 成功
    -> 入 normal TX queue 成功
    -> 暂时不能释放 FullFrame
    -> UART DMA 发送完成
    -> DataManager_ReleaseFullFrame()
```

失败路径必须释放：

```text
打包失败
    -> DataManager_ReleaseFullFrame()

TX 队列满
    -> DataManager_ReleaseFullFrame()

DMA 启动失败
    -> 切回接收态
    -> DataManager_ReleaseFullFrame()

DMA 发送超时或异常
    -> Abort DMA
    -> 切回接收态
    -> DataManager_ReleaseFullFrame()
```

如果 FullFrame 不释放，`DataManager` 的 FullFrame 内存池会逐渐耗尽。

## 主要辅助函数

### 帧解析函数

```c
static RS485_Status_t RS485_ParseFrame(const uint8_t *buf,
                                       uint16_t len,
                                       RS485_CmdFrame_t *cmd);
```

职责：

- 检查帧头
- 检查长度
- 校验 CRC
- 解析命令字
- 解析 payload
- 输出标准化命令结构

建议先支持以下命令：

| 命令 | 作用 |
| --- | --- |
| `START_STREAM` | 开始连续发送 FullFrame。 |
| `STOP_STREAM` | 停止连续发送。 |
| `SEND_ONE_FRAME` | 请求发送一帧。 |
| `GET_STATUS` | 查询 RS485Task 状态和错误统计。 |
| `RESET_STATS` | 清空错误统计。 |
| `SET_RATE` | 设置主动上报频率。 |

### 帧封装函数

```c
static RS485_Status_t RS485_PackFullFrame(uint8_t *buf,
                                          uint16_t buf_size,
                                          const GloveFullFrame_t *frame,
                                          uint16_t *out_len);
```

职责：

- 写入帧头
- 写入协议版本
- 写入帧类型
- 写入 payload 长度
- 写入 `frame_id`
- 写入 `timestamp_us`
- 写入 RawFrame 和 ProcessedFrame 中需要上报的字段
- 计算 CRC
- 写入 CRC
- 输出最终包长度

建议逐字段写入，不建议直接发送整个结构体。原因是结构体可能存在 padding，且上位机解析时还会遇到大小端、float 格式和对齐问题。

### 帧错误统计处理函数

```c
static void RS485_RecordError(RS485_Error_t error);
```

职责：

- 按错误类型递增计数
- 记录最近一次错误
- 严重错误时触发 UART/DMA 恢复
- 必要时切换 RS485 回接收态

建议错误类型：

```c
typedef enum
{
    RS485_ERR_NONE = 0,
    RS485_ERR_RX_CRC,
    RS485_ERR_RX_LENGTH,
    RS485_ERR_RX_HEADER,
    RS485_ERR_PACK_FAILED,
    RS485_ERR_TX_QUEUE_FULL,
    RS485_ERR_DMA_START_FAILED,
    RS485_ERR_DMA_TIMEOUT,
    RS485_ERR_UART_OVERRUN,
    RS485_ERR_UART_DMA_ERROR
} RS485_Error_t;
```

### RS485 DMA 发送函数

```c
static RS485_Status_t RS485_StartDmaSend(RS485_TxMsg_t *msg);
```

职责：

- 判断当前是否已经处于发送中
- 保存当前发送消息上下文
- 控制 DE 引脚切到发送态
- 调用 `HAL_UART_Transmit_DMA()`
- 记录发送开始 tick
- 启动发送超时检测

典型流程：

```text
检查参数
    -> 检查 s_tx_busy
    -> s_current_tx = msg
    -> s_tx_busy = 1
    -> DE = TX
    -> HAL_UART_Transmit_DMA()
    -> 成功后等待 TX_DONE 事件
```

注意：`HAL_UART_Transmit_DMA()` 返回成功只代表 DMA 已启动，不代表总线上的最后一位已经发完。DE 引脚必须在发送完成事件之后再拉低。

### 串口 DMA 接收处理函数

```c
static void RS485_RxDmaHandler(uint16_t rx_len);
```

职责：

- 处理 UART 空闲中断或 ReceiveToIdle 回调给出的接收长度
- 调用 `RS485_ParseFrame()`
- 根据命令改变发送模式或产生应答包
- 重新启动 DMA 接收
- 发现异常时进行 UART/DMA 反初始化和重新初始化

建议使用：

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huartx,
                             s_rx_dma_buffer,
                             RS485_RX_BUFFER_SIZE);
```

## 回调和事件投递

### 接收空闲回调

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huartx)
    {
        RS485_PostEvent(RS485_EVENT_RX_FRAME, Size, 0U);
    }
}
```

### DMA 发送完成回调

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huartx)
    {
        RS485_PostEvent(RS485_EVENT_TX_DONE, 0U, 0U);
    }
}
```

### UART 错误回调

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huartx)
    {
        RS485_PostEvent(RS485_EVENT_UART_ERROR, huart->ErrorCode, 0U);
    }
}
```

## RS485Task 主循环

建议主循环分成三件事：

```text
1. 主动模式下，从 DataManager 取 FullFrame，封装后放入 normal TX queue
2. 如果当前没有 DMA 发送，尝试从 TX 队列启动下一包
3. 从 event queue 取事件，分发处理 RX、TX_DONE、ERROR、TIMEOUT
```

伪代码：

```c
void RS485Task(void *argument)
{
    RS485_Event_t evt;

    RS485_CreateQueues();
    RS485_InitDmaReceive();

    for (;;)
    {
        if (s_rs485_mode == RS485_MODE_STREAM)
        {
            RS485_TryFetchFullFrameToTxQueue();
        }

        RS485_TryStartNextTx();

        if (osMessageQueueGet(s_rs485_event_queue, &evt, NULL, 10U) == osOK)
        {
            RS485_HandleEvent(&evt);
        }
    }
}
```

## 半双工状态判断

RS485 是半双工，总线同一时间只能发送或接收。建议维护一个明确状态：

```c
typedef enum
{
    RS485_STATE_RX_IDLE = 0,
    RS485_STATE_TX_PENDING,
    RS485_STATE_TX_BUSY,
    RS485_STATE_WAIT_TURNAROUND,
    RS485_STATE_RECOVERING
} RS485_State_t;
```

状态含义：

| 状态 | 含义 |
| --- | --- |
| `RS485_STATE_RX_IDLE` | 接收态或空闲态，可以启动发送。 |
| `RS485_STATE_TX_PENDING` | 已有待发送消息，但还未启动 DMA。 |
| `RS485_STATE_TX_BUSY` | UART DMA 正在发送。 |
| `RS485_STATE_WAIT_TURNAROUND` | 发送完成后等待总线方向稳定。 |
| `RS485_STATE_RECOVERING` | UART/DMA 异常恢复中。 |

`RS485_TryStartNextTx()` 必须先判断状态：

```c
static void RS485_TryStartNextTx(void)
{
    RS485_TxMsg_t msg;

    if (s_rs485_state != RS485_STATE_RX_IDLE)
    {
        return;
    }

    if (osMessageQueueGet(s_rs485_tx_high_queue, &msg, NULL, 0U) == osOK)
    {
        (void)RS485_StartDmaSend(&msg);
        return;
    }

    if (osMessageQueueGet(s_rs485_tx_normal_queue, &msg, NULL, 0U) == osOK)
    {
        (void)RS485_StartDmaSend(&msg);
        return;
    }

    if (s_rs485_mode == RS485_MODE_STREAM)
    {
        RS485_TryFetchFullFrameToTxQueue();
    }

    if (osMessageQueueGet(s_rs485_tx_normal_queue, &msg, NULL, 0U) == osOK)
    {
        (void)RS485_StartDmaSend(&msg);
    }
}
```

优先级顺序：

```text
1. 正在发送或恢复时，不启动新发送
2. 先发送 high TX queue 中的指令回应、状态回应、错误回应
3. 再发送 normal TX queue 中已经排队的 FullFrame
4. 主动发送模式下，再尝试从 DataManager 获取新的 FullFrame
5. 没有任何待发送数据时，保持接收态
```

## 接收指令与主动发送关系

`RS485Task` 同时处理两类发送来源：

```text
1. 接收上位机指令后产生回应
2. 主动上报 FullFrame 数据
```

推荐规则：

```text
RX_IDLE 状态
    -> 收到上位机命令
        -> 解析命令
        -> 生成 ACK / STATUS / ERROR
        -> 放入 high TX queue
        -> 投递 RS485_EVENT_TX_REQUEST

RX_IDLE 状态
    -> 没有命令回应待发
    -> 当前模式允许主动发送
        -> 从 DataManager 获取 FullFrame
        -> 封装后放入 normal TX queue
        -> 投递 RS485_EVENT_TX_REQUEST

TX_DONE 事件
    -> 确认 UART 真正发送完成
    -> DE 切回 RX
    -> 释放当前发送消息关联资源
    -> 状态回到 RX_IDLE
    -> 再次尝试发送下一包
```

收到命令后的处理示例：

```c
static void RS485_HandleCommand(const RS485_CmdFrame_t *cmd)
{
    switch (cmd->cmd)
    {
        case RS485_CMD_GET_STATUS:
            RS485_QueueStatusResponse();
            RS485_PostEvent(RS485_EVENT_TX_REQUEST, 0U, 0U);
            break;

        case RS485_CMD_START_STREAM:
            s_rs485_mode = RS485_MODE_STREAM;
            RS485_QueueAckResponse(cmd->cmd);
            RS485_PostEvent(RS485_EVENT_TX_REQUEST, 0U, 0U);
            break;

        case RS485_CMD_STOP_STREAM:
            s_rs485_mode = RS485_MODE_PAUSE;
            RS485_QueueAckResponse(cmd->cmd);
            RS485_PostEvent(RS485_EVENT_TX_REQUEST, 0U, 0U);
            break;

        case RS485_CMD_SEND_ONE_FRAME:
            RS485_QueueAckResponse(cmd->cmd);
            RS485_RequestSingleFrame();
            RS485_PostEvent(RS485_EVENT_TX_REQUEST, 0U, 0U);
            break;

        default:
            RS485_QueueErrorResponse(RS485_ERR_RX_HEADER);
            RS485_PostEvent(RS485_EVENT_TX_REQUEST, 0U, 0U);
            break;
    }
}
```

事件处理：

```c
static void RS485_HandleEvent(const RS485_Event_t *evt)
{
    switch (evt->type)
    {
        case RS485_EVENT_RX_FRAME:
            RS485_HandleRxFrame((uint16_t)evt->arg0);
            break;

        case RS485_EVENT_TX_DONE:
            RS485_HandleTxDone();
            break;

        case RS485_EVENT_UART_ERROR:
            RS485_HandleUartError(evt->arg0);
            break;

        case RS485_EVENT_DMA_ERROR:
            RS485_HandleDmaError(evt->arg0);
            break;

        case RS485_EVENT_TX_TIMEOUT:
            RS485_HandleTxTimeout();
            break;

        case RS485_EVENT_TX_REQUEST:
            RS485_TryStartNextTx();
            break;

        default:
            break;
    }
}
```

## 总线状态机

RS485 是半双工，总线上同一时刻只能有一方发送。发送 DMA 完成后，不等于可以马上发下一条；如果当前协议是主从问答模式，发送完成后应该先等待对端响应，直到响应完成或超时后才回到空闲态。

建议增加总线状态：

```c
typedef enum
{
    RS485_BUS_IDLE = 0,
    RS485_BUS_TX_BUSY,
    RS485_BUS_WAIT_REPLY,
    RS485_BUS_RX_BUSY,
    RS485_BUS_RX_DONE,
    RS485_BUS_TIMEOUT,
    RS485_BUS_ERROR
} RS485_BusState_t;
```

状态含义：

| 状态 | 含义 | 是否允许发送下一包 |
| --- | --- | --- |
| `RS485_BUS_IDLE` | 总线在本设备视角下空闲。 | 允许 |
| `RS485_BUS_TX_BUSY` | 本设备 UART DMA 正在发送。 | 不允许 |
| `RS485_BUS_WAIT_REPLY` | 本设备已发完，正在等待对端响应或接收窗口结束。 | 不允许 |
| `RS485_BUS_RX_BUSY` | 正在接收对端响应。 | 不允许 |
| `RS485_BUS_RX_DONE` | 一帧响应接收完成，等待任务解析。 | 不允许 |
| `RS485_BUS_TIMEOUT` | 等待响应超时，等待任务处理。 | 不允许 |
| `RS485_BUS_ERROR` | UART/DMA 或协议状态异常。 | 不允许 |

真正允许发送下一条的条件应是：

```c
static uint8_t RS485_CanSend(void)
{
    if (s_bus_state != RS485_BUS_IDLE)
    {
        return 0U;
    }

    if (HAL_UART_GetState(&huartx) == HAL_UART_STATE_BUSY_TX)
    {
        return 0U;
    }

    return 1U;
}
```

因此，`RS485_EVENT_TX_REQUEST` 和 `RS485_EVENT_TX_DONE` 都不能直接启动下一次 DMA。它们只能驱动状态机，最终由 `RS485_CanSend()` 和 `RS485_TryStartNextTx()` 决定。

推荐状态流：

```text
IDLE
    -> TX_BUSY
    -> WAIT_REPLY
    -> RX_DONE
    -> IDLE

WAIT_REPLY
    -> TIMEOUT
    -> IDLE

任意状态
    -> ERROR
    -> RECOVER
    -> IDLE
```

如果当前发送的是“主动上报 FullFrame”，且协议不要求对端回复，也建议在发送完成后进入一个很短的接收窗口，再回到 `IDLE`，避免连续发送时上位机没有机会插入控制命令。

## 发送完成处理

`RS485_HandleTxDone()` 应该完成：

```text
确认 UART 发送完成
    -> DE = RX
    -> 重新启动 ReceiveToIdle DMA
    -> 释放当前发送消息关联资源
    -> 根据当前消息类型进入 WAIT_REPLY 或短接收窗口
    -> 记录等待响应开始 tick
```

如果芯片/HAL 的 DMA 完成回调早于 UART TC 标志，需要在拉低 DE 前确认 TC：

```c
while (__HAL_UART_GET_FLAG(&huartx, UART_FLAG_TC) == RESET)
{
}
```

更好的方式是使用 UART 发送完成中断或在任务中短时间等待 TC，避免在中断里做阻塞等待。

示例流程：

```c
static void RS485_HandleTxDone(void)
{
    if (s_bus_state != RS485_BUS_TX_BUSY)
    {
        RS485_RecordError(RS485_ERR_UNEXPECTED_TX_DONE);
        return;
    }

    RS485_WaitUartTc();
    RS485_TX_DISABLE();
    RS485_RX_ENABLE();
    RS485_StartReceiveToIdleDma();

    RS485_ReleaseTxMsg(&s_current_tx);
    memset(&s_current_tx, 0, sizeof(s_current_tx));

    s_reply_start_tick = osKernelGetTickCount();
    s_bus_state = RS485_BUS_WAIT_REPLY;
}
```

注意：`TX_DONE` 后不要直接调用 `RS485_TryStartNextTx()`。如果刚发完请求，对端可能马上响应；此时继续发下一包会和对端响应冲突。

## 接收完成与响应超时

接收 DMA + IDLE 回调只负责记录接收长度并投递事件：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huartx)
    {
        if (Size > 0U)
        {
            s_rx_len = Size;
            RS485_PostEvent(RS485_EVENT_RX_FRAME, Size, 0U);
        }

        RS485_StartReceiveToIdleDma();
    }
}
```

任务中处理接收帧：

```c
static void RS485_HandleRxFrame(uint16_t rx_len)
{
    s_bus_state = RS485_BUS_RX_DONE;

    RS485_ProcessFrame(s_rx_dma_buffer, rx_len);

    s_rx_len = 0U;
    s_bus_state = RS485_BUS_IDLE;

    RS485_TryStartNextTx();
}
```

等待响应时需要超时判断：

```c
#define RS485_REPLY_TIMEOUT_MS  20U

static void RS485_CheckReplyTimeout(void)
{
    if (s_bus_state != RS485_BUS_WAIT_REPLY)
    {
        return;
    }

    if ((osKernelGetTickCount() - s_reply_start_tick) >= RS485_MsToTicks(RS485_REPLY_TIMEOUT_MS))
    {
        s_bus_state = RS485_BUS_TIMEOUT;
        RS485_PostEvent(RS485_EVENT_TX_TIMEOUT, 0U, 0U);
    }
}
```

超时事件处理：

```c
static void RS485_HandleTxTimeout(void)
{
    if (s_bus_state == RS485_BUS_TIMEOUT)
    {
        RS485_RecordError(RS485_ERR_DMA_TIMEOUT);
        s_bus_state = RS485_BUS_IDLE;
        RS485_TryStartNextTx();
    }
}
```

完整判断是：

```text
发送完成
    -> 切回接收
    -> WAIT_REPLY
    -> 收到 IDLE + DMA 接收事件
        -> 处理响应帧
        -> IDLE
        -> 再判断是否发送下一包
    -> 等待超时
        -> 记录无响应
        -> IDLE
        -> 再判断是否发送下一包
```

## 接收异常恢复

当出现 UART overrun、frame error、noise error、DMA error 时，可以执行恢复流程：

```text
记录错误
    -> HAL_UART_DMAStop()
    -> HAL_UART_DeInit()
    -> MX_USARTx_UART_Init()
    -> 重新启动 HAL_UARTEx_ReceiveToIdle_DMA()
    -> DE = RX
```

示例流程：

```c
static void RS485_RecoverRxDma(void)
{
    HAL_UART_DMAStop(&huartx);
    HAL_UART_DeInit(&huartx);
    MX_USARTx_UART_Init();

    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

    HAL_UARTEx_ReceiveToIdle_DMA(&huartx,
                                 s_rx_dma_buffer,
                                 RS485_RX_BUFFER_SIZE);
}
```

## 开发顺序建议

1. 先实现 `RS485Task`、事件队列、TX 队列。
2. 用固定字符串测试 DMA 发送。
3. 加 RS485 DE 引脚控制，确认发送完成后再切回接收态。
4. 加 `HAL_UARTEx_ReceiveToIdle_DMA()`，确认能收到上位机命令。
5. 实现 `RS485_ParseFrame()`，先支持 `START_STREAM`、`STOP_STREAM`、`GET_STATUS`。
6. 实现 `RS485_PackFullFrame()`，先只发送 `frame_id`、`timestamp_us`、`valid_flags`。
7. 接入 `DataManager_GetFullFrame(DATA_CONSUMER_RS485, ...)`。
8. 确认 DMA 发送完成后释放 `FullFrame`。
9. 扩展 payload 到 RawFrame 和 ProcessedFrame。
10. 加错误统计、发送超时和 DMA 异常恢复。
11. 加总线状态机，确保 `TX_DONE` 后等待响应完成或超时再发下一包。

## 最重要的原则

- 事件来源在回调里明确标记，`RS485Task` 只根据 `event.type` 分发处理。
- 同一时间只允许一个 UART DMA 发送。
- DE 引脚必须在真正发送完成后再切回接收态。
- DMA 发送完成不等于总线空闲，`TX_DONE` 后应进入 `WAIT_REPLY` 或短接收窗口。
- 只有 `RS485_BUS_IDLE` 才允许启动下一次发送。
- `FullFrame` 必须在发送完成或失败处理后释放。
- ACK/STATUS/ERROR 使用高优先级发送队列，避免被连续 FullFrame 堵住。
