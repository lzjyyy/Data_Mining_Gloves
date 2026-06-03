# Services/inc/frame_pool.h 解析

## 文件职责

`frame_pool.h` 声明固定大小内存池接口。工程运行时不使用 `malloc/free`，而是在启动阶段准备好固定数量的数据块，任务之间只传递指针。

## 主要结构体

### `FramePool_t`

描述一个固定块内存池。

| 字段 | 含义 |
| --- | --- |
| `storage` | 连续数据块存储区起始地址。 |
| `free_stack` | 空闲块索引栈。 |
| `block_size` | 单个数据块大小，单位 byte。 |
| `capacity` | 数据块总数量。 |
| `free_count` | 当前空闲块数量。 |
| `min_free_count` | 历史最低空闲数量，用于评估池容量是否足够。 |

### `FramePoolStats_t`

内存池运行统计。

| 字段 | 含义 |
| --- | --- |
| `capacity` | 总块数。 |
| `free_count` | 当前空闲块数。 |
| `min_free_count` | 历史最低空闲块数。 |
| `used_count` | 当前已使用块数。 |

## 函数声明

| 函数 | 作用 |
| --- | --- |
| `FramePool_Init()` | 初始化内存池，将存储区和空闲索引栈绑定到池对象。 |
| `FramePool_Alloc()` | 从池中取出一个空闲块。 |
| `FramePool_Free()` | 将一个块归还到池。 |
| `FramePool_Owns()` | 判断指针是否位于该池管理的地址范围内。 |
| `FramePool_GetStats()` | 获取内存池运行统计。 |

## 使用约定

- `storage` 和 `free_stack` 由上层静态分配，内存池不拥有动态分配能力。
- 释放时必须传回块起始地址，不能传块内部地址。
- `FramePool_Free()` 会检查地址归属，能拦截部分错误释放。
