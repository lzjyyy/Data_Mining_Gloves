#include "data_manager.h"

#include <stddef.h>
#include <string.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"


typedef struct
{
    osMessageQueueId_t raw_for_algorithm;
    osMessageQueueId_t full_for_storage;
    osMessageQueueId_t full_for_rs485;
} DataManagerQueues_t;

static FramePool_t s_raw_pool;
static FramePool_t s_full_pool;

static DataManagerQueues_t s_queues;
static GloveDataStats_t s_stats;
static uint8_t s_initialized;

static GloveRawFrameBlock_t s_raw_blocks[GLOVE_RAW_FRAME_POOL_SIZE];
static GloveFullFrameBlock_t s_full_blocks[GLOVE_FULL_FRAME_POOL_SIZE];
static uint16_t s_raw_free_stack[GLOVE_RAW_FRAME_POOL_SIZE];
static uint16_t s_full_free_stack[GLOVE_FULL_FRAME_POOL_SIZE];

static osMessageQueueId_t CreatePointerQueue(uint32_t depth, const char *name)
{
    const osMessageQueueAttr_t attr = {
        .name = name,
        .attr_bits = 0U,
        .cb_mem = NULL,
        .cb_size = 0U,
        .mq_mem = NULL,
        .mq_size = 0U
    };

    return osMessageQueueNew(depth, sizeof(void *), &attr);
}

static uint32_t TimeoutMsToTicks(uint32_t timeout_ms)
{
    uint64_t ticks;

    if (timeout_ms == osWaitForever)
    {
        return osWaitForever;
    }

    /* 向上取整，确保非 0 毫秒超时时间不会被换算成 0 tick */
    ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;
    if ((timeout_ms > 0U) && (ticks == 0ULL))
    {
        ticks = 1ULL;
    }

    return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static void StatsIncrement(uint32_t *value)
{
    /* 统计量可能被多个任务更新，因此自增操作放在临界区 */
    taskENTER_CRITICAL();
    (*value)++;
    taskEXIT_CRITICAL();
}

static GloveStatus_t SendPointer(osMessageQueueId_t queue, void *ptr, uint32_t timeout_ms)
{
    /* 所有发布路径统一走这个函数，保证队列发送失败统计一致 */
    if ((queue == NULL) || (ptr == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (osMessageQueuePut(queue, &ptr, 0U, TimeoutMsToTicks(timeout_ms)) != osOK)
    {
        StatsIncrement(&s_stats.queue_send_failures);
        return GLOVE_STATUS_QUEUE_FULL;
    }

    return GLOVE_STATUS_OK;
}

static GloveStatus_t ReceivePointer(osMessageQueueId_t queue, void **ptr, uint32_t timeout_ms)
{
    void *local_ptr = NULL;
    osStatus_t status;

    if ((queue == NULL) || (ptr == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = osMessageQueueGet(queue, &local_ptr, NULL, TimeoutMsToTicks(timeout_ms));
    if (status == osOK)
    {
        *ptr = local_ptr;
        return GLOVE_STATUS_OK;
    }

    return (status == osErrorTimeout) ? GLOVE_STATUS_TIMEOUT : GLOVE_STATUS_QUEUE_EMPTY;
}

/*
 * 引用计数模型：
 * 1. 发布者在发布过程中临时持有 1 个引用；
 * 2. 每成功投递给一个消费者队列，就给该消费者增加 1 个引用；
 * 3. 消费者 Get* 成功后，处理完必须调用一次 Release*；
 * 4. 最后一个引用释放时，帧块才会归还到对应内存池
 */
static void RawFrame_AddRef(GloveRawFrameBlock_t *block)
{
    taskENTER_CRITICAL();
    block->ref_count++;
    taskEXIT_CRITICAL();
}

static void FullFrame_AddRef(GloveFullFrameBlock_t *block)
{
    taskENTER_CRITICAL();
    block->ref_count++;
    taskEXIT_CRITICAL();
}

GloveStatus_t DataManager_Init(void)
{
    GloveStatus_t status;

    /* 创建内存池和队列前，先清空运行状态 */
    (void)memset(&s_queues, 0, sizeof(s_queues));
    (void)memset(&s_stats, 0, sizeof(s_stats));

    /* RawFrame 由采集任务产生，只给算法任务消费 */
    status = FramePool_Init(&s_raw_pool,
                            s_raw_blocks,
                            sizeof(s_raw_blocks[0]),
                            GLOVE_RAW_FRAME_POOL_SIZE,
                            s_raw_free_stack);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    /* FullFrame 包含 raw + processed，由 SD 和 RS485 消费 */
    status = FramePool_Init(&s_full_pool,
                            s_full_blocks,
                            sizeof(s_full_blocks[0]),
                            GLOVE_FULL_FRAME_POOL_SIZE,
                            s_full_free_stack);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    /* 每个消费者一个独立队列，避免慢消费者阻塞其他消费者 */
    s_queues.raw_for_algorithm = CreatePointerQueue(GLOVE_RAW_FRAME_QUEUE_DEPTH, "raw.alg");
    s_queues.full_for_storage = CreatePointerQueue(GLOVE_FULL_FRAME_QUEUE_DEPTH, "full.store");
    s_queues.full_for_rs485 = CreatePointerQueue(GLOVE_FULL_FRAME_QUEUE_DEPTH, "full.rs485");

    if ((s_queues.raw_for_algorithm == NULL) ||
        (s_queues.full_for_storage == NULL) ||
        (s_queues.full_for_rs485 == NULL))
    {
        return GLOVE_STATUS_NO_MEMORY;
    }

    s_initialized = 1U;
    return GLOVE_STATUS_OK;
}

GloveRawFrameBlock_t *DataManager_AllocRawFrame(void)
{
    GloveRawFrameBlock_t *block;

    /* 未初始化前申请帧，视为调用顺序错误 */
    if (s_initialized == 0U)
    {
        return NULL;
    }

    block = (GloveRawFrameBlock_t *)FramePool_Alloc(&s_raw_pool);
    if (block == NULL)
    {
        StatsIncrement(&s_stats.pool_alloc_failures);
        return NULL;
    }

    /* 给生产者一帧干净数据，并重置引用计数 */
    AppData_ClearRawFrame(&block->frame);
    block->ref_count = 0U;
    return block;
}

GloveFullFrameBlock_t *DataManager_AllocFullFrame(void)
{
    GloveFullFrameBlock_t *block;

    /* 未初始化前申请帧，视为调用顺序错误 */
    if (s_initialized == 0U)
    {
        return NULL;
    }

    block = (GloveFullFrameBlock_t *)FramePool_Alloc(&s_full_pool);
    if (block == NULL)
    {
        StatsIncrement(&s_stats.pool_alloc_failures);
        return NULL;
    }

    AppData_ClearFullFrame(&block->frame);
    block->ref_count = 0U;
    return block;
}

GloveStatus_t DataManager_PublishRawFrame(GloveRawFrameBlock_t *block, uint32_t timeout_ms)
{
    GloveStatus_t status;
    GloveStatus_t final_status = GLOVE_STATUS_OK;
    uint8_t delivered_count = 0U;

    if ((s_initialized == 0U) || (block == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    /*
     * 发布者引用用于保证发布过程中的帧始终有效
     * 即使消费者任务很快收到并释放了帧，也不会导致发布过程访问已归还内存
     */
    block->ref_count = 1U;

    /* 原始数据只投递给算法任务 */
    RawFrame_AddRef(block);
    status = SendPointer(s_queues.raw_for_algorithm, block, timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        (void)DataManager_ReleaseRawFrame(block);
        final_status = status;
    }
    else
    {
        delivered_count++;
    }

    /* 发布完成后释放发布者临时引用 */
    (void)DataManager_ReleaseRawFrame(block);

    if (delivered_count == 0U)
    {
        StatsIncrement(&s_stats.raw_frames_dropped);
        return GLOVE_STATUS_QUEUE_FULL;
    }

    StatsIncrement(&s_stats.raw_frames_published);
    return final_status;
}

GloveStatus_t DataManager_PublishFullFrame(GloveFullFrameBlock_t *block, uint32_t timeout_ms)
{
    GloveStatus_t status;
    GloveStatus_t final_status = GLOVE_STATUS_OK;
    uint8_t delivered_count = 0U;

    if ((s_initialized == 0U) || (block == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    /*
     * FullFrame 是 SD 和 RS485 唯一可见的数据帧
     * 这样可以保证原始采样和对应算法结果始终绑定在同一个 frame_id 上
     */
    block->ref_count = 1U;

    /* SD 存储任务获得一个独立引用 */
    FullFrame_AddRef(block);
    status = SendPointer(s_queues.full_for_storage, block, timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        (void)DataManager_ReleaseFullFrame(block);
        final_status = status;
    }
    else
    {
        delivered_count++;
    }

    /* RS485 通讯任务获得另一个独立引用 */
    FullFrame_AddRef(block);
    status = SendPointer(s_queues.full_for_rs485, block, timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        (void)DataManager_ReleaseFullFrame(block);
        final_status = status;
    }
    else
    {
        delivered_count++;
    }

    /* 发布完成后释放发布者临时引用 */
    (void)DataManager_ReleaseFullFrame(block);

    if (delivered_count == 0U)
    {
        StatsIncrement(&s_stats.full_frames_dropped);
        return GLOVE_STATUS_QUEUE_FULL;
    }

    StatsIncrement(&s_stats.full_frames_published);
    return final_status;
}

GloveStatus_t DataManager_GetRawFrame(DataConsumer_t consumer,
                                      GloveRawFrameBlock_t **block,
                                      uint32_t timeout_ms)
{
    osMessageQueueId_t queue = NULL;

    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    switch (consumer)
    {
        case DATA_CONSUMER_ALGORITHM:
            /* RawFrame 只暴露给算法任务，不暴露给 SD/RS485 */
            queue = s_queues.raw_for_algorithm;
            break;
        default:
            return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReceivePointer(queue, (void **)block, timeout_ms);
}

GloveStatus_t DataManager_GetFullFrame(DataConsumer_t consumer,
                                       GloveFullFrameBlock_t **block,
                                       uint32_t timeout_ms)
{
    osMessageQueueId_t queue = NULL;

    if (block == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    switch (consumer)
    {
        case DATA_CONSUMER_STORAGE:
            /* SD 存储任务记录完整的 raw + processed 数据 */
            queue = s_queues.full_for_storage;
            break;
        case DATA_CONSUMER_RS485:
            /* RS485 发送同样的完整数据帧格式 */
            queue = s_queues.full_for_rs485;
            break;
        default:
            return GLOVE_STATUS_INVALID_PARAM;
    }

    return ReceivePointer(queue, (void **)block, timeout_ms);
}

GloveStatus_t DataManager_ReleaseRawFrame(GloveRawFrameBlock_t *block)
{
    uint8_t should_free = 0U;

    /* 拒绝外来指针，确保只能释放 raw_pool 管理的块 */
    if ((s_initialized == 0U) || (block == NULL) || (FramePool_Owns(&s_raw_pool, block) == 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    /* 最后一个引用释放时，将块归还到 raw_pool */
    taskENTER_CRITICAL();
    if (block->ref_count > 0U)
    {
        block->ref_count--;
        should_free = (block->ref_count == 0U) ? 1U : 0U;
    }
    taskEXIT_CRITICAL();

    return (should_free != 0U) ? FramePool_Free(&s_raw_pool, block) : GLOVE_STATUS_OK;
}

GloveStatus_t DataManager_ReleaseFullFrame(GloveFullFrameBlock_t *block)
{
    uint8_t should_free = 0U;

    /* 确保只释放 full_pool 管理的块 */
    if ((s_initialized == 0U) || (block == NULL) || (FramePool_Owns(&s_full_pool, block) == 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    /* 最后一个引用释放时，将块归还到 full_pool */
    taskENTER_CRITICAL();
    if (block->ref_count > 0U)
    {
        block->ref_count--;
        should_free = (block->ref_count == 0U) ? 1U : 0U;
    }
    taskEXIT_CRITICAL();

    return (should_free != 0U) ? FramePool_Free(&s_full_pool, block) : GLOVE_STATUS_OK;
}

void DataManager_GetStats(DataManagerStats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    /* 统计计数先在临界区内快照，然后再读取各内存池状态 */
    taskENTER_CRITICAL();
    stats->data = s_stats;
    taskEXIT_CRITICAL();

    FramePool_GetStats(&s_raw_pool, &stats->raw_pool);
    FramePool_GetStats(&s_full_pool, &stats->full_pool);
}
