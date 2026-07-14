# Gripper DMA receive notes

本文档说明这次为自研夹爪驱动增加 DMA 接收后，工程里改了什么、为什么改、当前数据流怎么走，以及后续调试要注意什么。

## 背景问题

原来的夹爪驱动在 `Gripper_ReadFrame()` 中使用阻塞式单字节接收：

```c
RS485_Receive(handle->rs485_ch, &b, 1U, 1U);
```

左右夹爪分别在两个 FreeRTOS 任务中运行：

- 左夹爪：`RS485A_CH` / `USART1`
- 右夹爪：`RS485B_CH` / `USART3`

虽然硬件串口是两个，但 STM32F407 是单核 MCU，任意时刻 CPU 只能运行一个任务。如果上位机高频同时下发左右夹爪开合命令，优先级高的任务可能打断优先级低的任务。低优先级任务如果正在收一帧回包，就可能不能及时从 UART 中取走后续字节，导致半包、错包、CRC 错误，最终返回：

```c
GRIPPER_BAD_FRAME = -5
```

因此这次改造的目标是：UART 收到的字节先由 DMA/中断放入内存 ring buffer，任务后面再慢慢解析。这样任务被抢占时，UART 数据也不会依赖任务实时读取。

## 修改文件

本次 DMA 接收相关改动涉及 3 个文件：

```text
Core/Src/gripper.c
Core/Inc/gripper.h
MODBUS-LIB/Src/UARTCallback.c
```

没有修改 `usart.c` 的 DMA 初始化。工程原本已经生成了 USART1/USART3 的 DMA 句柄和中断：

- `USART1 RX` 使用 `hdma_usart1_rx`
- `USART3 RX` 使用 `hdma_usart3_rx`
- DMA IRQ 已经在 `stm32f4xx_it.c` 中调用 HAL handler

## gripper.c 改动

### 1. 新增 DMA 接收参数

位置：`Core/Src/gripper.c`

```c
#define GRIPPER_DMA_RX_CHUNK_SIZE 64U
#define GRIPPER_DMA_RX_RING_SIZE 512U
```

含义：

- `GRIPPER_DMA_RX_CHUNK_SIZE`：每次 `HAL_UARTEx_ReceiveToIdle_DMA()` 使用的 DMA 临时缓冲区大小。
- `GRIPPER_DMA_RX_RING_SIZE`：夹爪驱动自己的环形缓冲区大小。

当前每个夹爪串口都有独立缓冲：

```text
USART1 / RS485A -> left ring buffer
USART3 / RS485B -> right ring buffer
```

### 2. 新增 `GripperDmaRx_t`

新增结构体用于管理每路 DMA 接收：

```c
typedef struct {
  uint8_t rs485_ch;
  UART_HandleTypeDef* huart;
  uint8_t dma_buf[GRIPPER_DMA_RX_CHUNK_SIZE];
  uint8_t ring_buf[GRIPPER_DMA_RX_RING_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint8_t active;
} GripperDmaRx_t;
```

当前实例：

```c
static GripperDmaRx_t gripper_dma_rx_a = { RS485A_CH, &huart1, ... };
static GripperDmaRx_t gripper_dma_rx_b = { RS485B_CH, &huart3, ... };
```

### 3. 新增 ring buffer 操作

新增函数：

```c
static void Gripper_DmaRxReset(GripperDmaRx_t* rx);
static void Gripper_DmaRxPush(GripperDmaRx_t* rx, const uint8_t* data, uint16_t len);
static HAL_StatusTypeDef Gripper_DmaRxReadByte(uint8_t rs485_ch, uint8_t* data, uint32_t timeout_ms);
```

作用：

- `Gripper_DmaRxReset()`：清空 ring buffer。
- `Gripper_DmaRxPush()`：DMA 回调中把收到的数据放入 ring buffer。
- `Gripper_DmaRxReadByte()`：协议解析时从 ring buffer 读取一个字节。

如果 ring buffer 满了，当前策略是丢弃最旧字节，保留最新数据。

### 4. 新增 DMA 启动函数

新增：

```c
static HAL_StatusTypeDef Gripper_DmaRxStart(uint8_t rs485_ch);
```

它内部调用：

```c
HAL_UARTEx_ReceiveToIdle_DMA(rx->huart,
                             rx->dma_buf,
                             GRIPPER_DMA_RX_CHUNK_SIZE);
```

并关闭半传输中断：

```c
__HAL_DMA_DISABLE_IT(rx->huart->hdmarx, DMA_IT_HT);
```

半传输中断对本场景没必要，关闭后可以减少中断次数。

### 5. `Gripper_Init()` 自动启动 DMA

在 `Gripper_Init()` 最后新增：

```c
(void)Gripper_DmaRxStart(rs485_ch);
```

因此左/右夹爪初始化后会自动启动对应 UART 的 DMA 接收。

### 6. 新增 `Gripper_ReadByte()`

新增：

```c
static HAL_StatusTypeDef Gripper_ReadByte(uint8_t rs485_ch,
                                          uint8_t* data,
                                          uint32_t timeout_ms);
```

逻辑：

- 如果该通道 DMA ring buffer 已激活，则从 ring buffer 读字节。
- 如果 DMA 没激活，则回退到原来的阻塞接收：

```c
RS485_Receive(rs485_ch, data, 1U, timeout_ms);
```

这样做的好处是：如果 DMA 启动失败，驱动仍然有兜底路径。

### 7. `Gripper_ReadFrame()` 改为按完整帧读取

`Gripper_ReadFrame()` 现在不再直接调用 `RS485_Receive()`，而是调用：

```c
Gripper_ReadByte()
```

并且按协议格式精确读完整帧：

```text
0xAC + seq + addr + cmd + len + payload + crc16
```

处理流程：

```text
1. 等待 0xAC 帧头
2. 读取 5 字节固定头
3. 根据 len 计算完整帧长度
4. 继续读取 payload + crc
5. 校验 CRC
6. 校验 seq / cmd / addr
7. 输出 payload
```

这样即使低优先级任务中途被高优先级任务抢占，只要总超时时间没到，它恢复运行后会继续从 ring buffer 取剩余字节，而不是把半包直接判成 `-5`。

### 8. `Gripper_FlushRx()` 适配 DMA

原来 `Gripper_FlushRx()` 会调用：

```c
HAL_UART_AbortReceive(huart);
```

这会把 DMA 接收停掉。

现在逻辑改为：

- 如果该串口 DMA ring buffer 已激活，只清 ring buffer。
- 如果 DMA 未激活，才走原来的 `HAL_UART_AbortReceive()`。

这样每次事务前 flush 不会把夹爪 DMA 接收停掉。

## gripper.h 改动

新增一个函数声明：

```c
uint8_t Gripper_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);
```

这个函数不是 HAL 的弱回调，而是夹爪驱动自己的回调处理函数。

返回值含义：

```text
1 = 该 UART 属于夹爪 DMA，数据已被夹爪驱动处理
0 = 该 UART 不属于夹爪 DMA，交给其他模块处理
```

## UARTCallback.c 改动

工程中原本已经有 HAL 回调：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
```

位置：

```text
MODBUS-LIB/Src/UARTCallback.c
```

一开始我直接在 `gripper.c` 里也写了同名函数，Keil 链接时报：

```text
Error: L6200E: Symbol HAL_UARTEx_RxEventCallback multiply defined
```

因此最终做法是保留工程原来的唯一 HAL 回调，在它开头增加转发：

```c
if (Gripper_UARTEx_RxEventCallback(huart, Size) != 0U)
{
    return;
}
```

含义：

- 如果是夹爪正在使用的 USART1/USART3，数据进入夹爪 ring buffer，然后直接返回，不再走 Modbus。
- 如果不是夹爪处理的 UART，继续走原来的 Modbus DMA 逻辑。

## 当前接收数据流

以左夹爪为例：

```text
夹爪回包
  -> USART1 RX
  -> DMA 写入 gripper_dma_rx_a.dma_buf
  -> HAL_UARTEx_RxEventCallback()
  -> Gripper_UARTEx_RxEventCallback()
  -> Gripper_DmaRxPush()
  -> gripper_dma_rx_a.ring_buf
  -> Gripper_ReadFrame()
  -> Gripper_ReadByte()
  -> 校验完整协议帧
  -> payload 给 Gripper_ParseRealtime()
```

右夹爪同理，只是通道变为：

```text
USART3 / RS485B / gripper_dma_rx_b
```

## 和 Modbus DMA 的关系

你现在说 Modbus DMA 已经不用了，但工程中 `MODBUS-LIB/Src/UARTCallback.c` 仍然参与编译，所以必须保留唯一的 HAL 回调。

当前设计不会删除 Modbus 回调，只是在回调最前面给夹爪优先处理机会：

```text
夹爪 UART -> 夹爪处理 -> return
非夹爪 UART -> 原 Modbus 逻辑
```

这样对其他仍可能使用 Modbus 的串口影响最小。

## 需要注意的点

### 1. 不要再在别的文件定义 `HAL_UARTEx_RxEventCallback`

整个工程只能有一个：

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
```

现在这个函数在：

```text
MODBUS-LIB/Src/UARTCallback.c
```

夹爪驱动使用：

```c
Gripper_UARTEx_RxEventCallback()
```

作为转发函数，不要再改回 HAL 同名函数。

### 2. `Gripper_Init()` 会启动 DMA

如果调试中发现某个夹爪没有回包，要先确认 `Gripper_Init()` 是否执行，以及 `Gripper_DmaRxStart()` 是否返回 `HAL_OK`。

当前代码没有打印 DMA 启动结果。如果后续需要调试，可以临时加打印。

### 3. `Gripper_FlushRx()` 不会停 DMA

DMA active 时，flush 只清 ring buffer。这样避免每次发命令前把 DMA 接收停掉。

### 4. 如果仍然出现大量 `-5`

DMA 已经解决“任务被抢占导致来不及从 UART 取字节”的问题。

如果仍大量 `-5`，优先怀疑：

- 回包确实 CRC 错
- 收到了旧包，seq 不匹配
- 收到了其他地址或其他命令的包
- 上位机下发太快，底层事务交叠，读到上一条命令的迟到回包
- 发送前 flush 清掉了尚未处理完的数据

这时建议在 `Gripper_ReadFrame()` 中临时打印失败原因，例如 CRC 错、seq 错、cmd 错、addr 错，进一步定位。

### 5. 如果出现 `-3`

`-3 = GRIPPER_TIMEOUT`，表示 ring buffer 中没有等到合法帧头或完整帧。

优先检查：

- A/B 线
- 地址
- 供电
- 波特率
- DMA 是否成功启动
- 该 UART 是否被其他模块重新 `HAL_UART_DMAStop()` 或 `HAL_UART_AbortReceive()`

## 本次改动目的总结

这次 DMA 改造不是为了改变夹爪协议，也不是改变 MQTT 接口，而是改变底层接收方式：

```text
原来：任务阻塞式逐字节读 UART
现在：DMA/中断先收进 ring buffer，任务再从 buffer 解析完整帧
```

它主要解决的问题是：

```text
左右夹爪同时高频动作时，低优先级任务被抢占，导致收半包/坏帧，返回 -5。
```

改完后，任务即使被抢占，UART 数据也会先进入内存缓冲，理论上能显著减少低优先级夹爪的 `GRIPPER_BAD_FRAME = -5`。
