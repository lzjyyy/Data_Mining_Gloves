# Services/src/frame_pool.c 解析

## 文件职责

`frame_pool.c` 实现固定块内存池。它用一个空闲索引栈管理静态数组中的数据块，适合嵌入式 RTOS 中对实时性和内存确定性要求较高的数据帧流转。

## 函数解析

### `FramePool_IsAligned(const void *ptr)`

检查地址是否按 32 bit 对齐。

- 返回 `1` 表示已对齐，返回 `0` 表示未对齐。
- `FramePool_Init()` 用它检查 `storage` 起始地址，避免结构体访问时出现非对齐风险。

### `FramePool_Init(FramePool_t *pool, void *storage, uint16_t block_size, uint16_t capacity, uint16_t *free_stack)`

初始化内存池。

- 检查 `pool`、`storage`、`free_stack` 是否为空。
- 检查 `block_size`、`capacity` 是否为 0。
- 检查 `storage` 是否 32 bit 对齐。
- 设置池的存储区、块大小、容量、空闲数量和历史最低空闲数量。
- 初始化 `free_stack`，分配时从栈顶弹出索引。
- 成功返回 `GLOVE_STATUS_OK`，参数不合法返回 `GLOVE_STATUS_INVALID_PARAM`。

### `FramePool_Alloc(FramePool_t *pool)`

从内存池分配一个数据块。

- `pool` 为空时返回 `NULL`。
- 在临界区中操作 `free_count` 和 `free_stack`，避免多任务并发破坏栈状态。
- 空闲块不足时返回 `NULL`。
- 分配成功后更新 `min_free_count`，用于后续观察池容量压力。

### `FramePool_Free(FramePool_t *pool, void *block)`

归还一个数据块。

- 检查参数和地址是否属于该内存池。
- 计算 `block` 相对 `storage` 的偏移。
- 要求释放地址正好是某个块的起始地址。
- 在临界区中将块索引压回 `free_stack`。
- 如果 `free_count >= capacity`，说明可能重复释放，返回 `GLOVE_STATUS_ERROR`。

### `FramePool_Owns(const FramePool_t *pool, const void *block)`

判断指针是否落在内存池管理的连续地址范围内。

- 不判断是否为块起始地址，只判断地址范围。
- `FramePool_Free()` 会进一步检查块对齐到 `block_size`。

### `FramePool_GetStats(const FramePool_t *pool, FramePoolStats_t *stats)`

读取内存池统计。

- 参数为空时直接返回。
- 在临界区中复制容量、空闲数量、历史最低空闲数量。
- `used_count` 由 `capacity - free_count` 计算得到。

## 注意点

- 该内存池只负责块级分配，不清零块内容；清零由 `DataManager_Alloc*()` 调用 `AppData_Clear*()` 完成。
- `FramePool_Owns()` 只能证明地址在范围内，不能证明这块当前处于已分配状态。
- 当前没有记录每个块的已分配标记，因此重复释放只能通过 `free_count >= capacity` 捕获一部分场景。
