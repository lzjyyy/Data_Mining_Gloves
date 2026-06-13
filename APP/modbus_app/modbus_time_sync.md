# Modbus 时间同步说明

本文档整理 `modbus_time_sync.c/.h` 以及 `main.c`、`modbus_frame.c` 中与时间同步相关的调用关系，方便后续维护和主机端对接。

## 1. 目标

时间同步模块的目标是把外部同步脉冲 `Time_tongbu_Pin` 和主机通过 Modbus 写入的 UTC 时间戳绑定起来，形成设备本地可查询的 UTC 微秒时间。

模块内部维护两类时间：

- 本地时间：TIM5 计数值，按微秒语义使用，溢出次数由软件扩展到 64 位。
- UTC 时间：主机写入的 64 位 UTC 微秒时间戳，配合本地计时推算当前 UTC。

## 2. 硬件和入口

相关初始化在 `main.c`：

- `MX_GPIO_Init()` 配置 `Time_tongbu_Pin` 为上升沿 EXTI，带上拉。
- `MX_TIM5_Init()` 配置 TIM5 为 32 位向上计数器，`Prescaler = 250 - 1`，`Period = 0xFFFFFFFF`。
- `ModbusTimeSync_Init()` 清空同步状态和 TIM5 计数。

中断入口：

- `HAL_GPIO_EXTI_Rising_Callback()` 调用 `ModbusTimeSync_OnGpioSyncEdge(GPIO_Pin)`。
- `HAL_TIM_PeriodElapsedCallback()` 调用 `ModbusTimeSync_OnTimPeriodElapsed(htim)`，用于统计 TIM5 溢出次数。

## 3. Modbus 寄存器接口

时间同步相关寄存器定义在 `modbus_registers.h`：

| 寄存器 | 宽度 | 方向 | 含义 |
| --- | --- | --- | --- |
| `REG_UTC_TIMESTAMP_US` (`0x0002`) | 4 个 16 位寄存器 | 读 | 当前推算出的 UTC 微秒时间。尚未建立 UTC 基准时返回 0。 |
| `REG_LOCAL_UPTIME_US` (`0x0006`) | 4 个 16 位寄存器 | 读 | 当前本地 TIM5 扩展计数，单位按 us 使用。 |
| `REG_TIME_SYNC_UTC_US` (`0x000A`) | 4 个 16 位寄存器 | 读/写 | 读为最近一次主机写入的 UTC；写用于执行一次校时。 |

64 位值的寄存器顺序为低 16 位在前：

```text
reg + 0: bits 15..0
reg + 1: bits 31..16
reg + 2: bits 47..32
reg + 3: bits 63..48
```

写时间同步时，`modbus_frame.c` 只接受从 `REG_TIME_SYNC_UTC_US` 开始、长度为 `MODBUS_REGS_U64` 的连续写入。收到后调用：

```c
ModbusTimeSync_SetUtcFromMaster(utc_us);
```

## 4. 同步流程

典型流程如下：

1. 外部同步信号在 `Time_tongbu_Pin` 产生上升沿。
2. `ModbusTimeSync_OnGpioSyncEdge()` 捕获当前本地计数，记录为 `time_sync_last_edge_local_us`。
3. 如果此前已经有 UTC 基准，则用当前本地间隔和频率修正值预测该边沿对应的 UTC，保存到 `time_sync_predicted_edge_utc_us`。
4. 模块置位 `time_sync_wait_utc_frame = 1`，表示正在等待主机发来该同步边沿对应的 UTC；已有 UTC 基准时，当前 UTC 仍继续递增。
5. 主机通过 Modbus 写 `REG_TIME_SYNC_UTC_US`，传入 64 位 UTC 微秒时间。
6. `ModbusTimeSync_SetUtcFromMaster()` 根据当前状态更新 UTC 基准、误差和频率修正。
7. 校时完成后置位 `time_sync_synced = 1`，清除 `time_sync_wait_utc_frame`。

## 5. 关键函数

### `ModbusTimeSync_Init`

清空所有同步状态变量，清零 TIM5 计数器，清除 TIM5 更新标志。该函数不主动启动 TIM5，TIM5 会在第一次同步边沿或第一次主机写 UTC 时启动。

### `ModbusTimeSync_OnTimPeriodElapsed`

只处理 TIM5 的更新中断：

```c
if ((htim != NULL) && (htim->Instance == TIM5))
{
  time_sync_local_timer_overflow++;
}
```

`time_sync_local_timer_overflow` 和 TIM5 当前计数拼成 64 位本地时间。

### `ModbusTimeSync_OnGpioSyncEdge`

只响应 `Time_tongbu_Pin`：

- 如果 TIM5 尚未运行，先清零并启动 TIM5，本次边沿本地时间记为 0。
- 如果 TIM5 已运行，读取当前本地时间。
- 记录本次边沿时间和本地间隔。
- 如果已有 UTC 基准，则预测本次边沿 UTC。
- 标记正在等待主机 UTC 帧。已有 UTC 基准时，不清除当前同步状态，`REG_UTC_TIMESTAMP_US` 仍会继续递增。

### `ModbusTimeSync_SetUtcFromMaster`

主机写入 UTC 后进入该函数。它有三种处理分支：

| 状态 | 处理 |
| --- | --- |
| 没有等待同步边沿 | 直接把主机 UTC 作为新的 UTC 基准，清零并重启本地计时。 |
| 等待边沿，但没有预测值 | 通常是第一次同步。清零频率修正，根据“边沿到收到帧”的本地耗时修正 UTC 基准。 |
| 等待边沿，且已有预测值 | 计算 `utc_us - predicted_edge_utc_us` 作为同步误差，按误差调整频率修正，再更新 UTC 基准。 |

函数最后会：

- `time_sync_wait_utc_frame = 0`
- `time_sync_has_prediction = 0`
- `time_sync_synced = 1`

### `ModbusTimeSync_GetUtcTimestampUs`

只有 `time_sync_synced != 0` 时才返回有效 UTC：

```text
UTC now = time_sync_utc_base_us + ApplyFreqCorr(local_now_us)
```

未同步时返回 0。

## 6. 频率修正逻辑

频率修正单位是 ppb，内部变量为 `time_sync_freq_corr_ppb`。

相关限制：

| 宏 | 值 | 含义 |
| --- | --- | --- |
| `TIME_SYNC_ERROR_LIMIT_US` | `500000` | 单次误差超过 500 ms 时，不继续累加修正，直接清零频率修正。 |
| `TIME_SYNC_MAX_CORR_STEP_PPB` | `100000` | 单次修正步进最大 100000 ppb。 |
| `TIME_SYNC_MAX_CORR_PPB` | `1000000` | 总修正最大 1000000 ppb。 |

修正计算：

```text
corr_step_ppb = error_us * 1_000_000_000 / last_local_interval_us
next_corr_ppb = old_corr_ppb + clamp(corr_step_ppb)
```

推算经过时间时使用：

```text
corrected_elapsed_us = elapsed_us + elapsed_us * corr_ppb / 1_000_000_000
```

## 7. 对接注意事项

- 主机写入的 UTC 单位必须是微秒，且 64 位寄存器低字在前。
- 推荐流程是先给 `Time_tongbu_Pin` 一个上升沿同步脉冲，再尽快写入该脉冲对应的 UTC 到 `REG_TIME_SYNC_UTC_US`。
- 如果主机没有等待同步边沿就直接写 UTC，模块会直接以收到写入时刻作为 UTC 基准。
- `REG_UTC_TIMESTAMP_US` 在尚未建立 UTC 基准时返回 0；建立基准后，即使正在等待下一帧同步 UTC，也会继续返回递增时间。
- 读取多寄存器 64 位时间时，建议一次性读取完整 4 个寄存器，避免跨越计数更新造成高低字不一致。
