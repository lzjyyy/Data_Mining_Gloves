#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"
#include "frame_pool.h"

/* 数据消费者类型，用于选择对应的订阅队列 */
typedef enum
{
    DATA_CONSUMER_ALGORITHM = 0,
    DATA_CONSUMER_STORAGE = 1,
    DATA_CONSUMER_RS485 = 2
} DataConsumer_t;

/* ref_count 是引用计数，用于保证消费者全部释放后再归还内存池 */
typedef struct
{
    GloveRawFrame_t frame;
    volatile uint8_t ref_count;
    uint8_t reserved[3];
} GloveRawFrameBlock_t;

typedef struct
{
    GloveFullFrame_t frame;
    volatile uint8_t ref_count;
    uint8_t reserved[3];
} GloveFullFrameBlock_t;

typedef struct
{
    GloveDataStats_t data;
    FramePoolStats_t raw_pool;
    FramePoolStats_t full_pool;
} DataManagerStats_t;

GloveStatus_t DataManager_Init(void);

/*
 * 数据流：
 *   采集任务 -> RawFrame -> 算法任务
 *   算法任务 -> FullFrame(raw + processed) -> SD 存储 / RS485 通讯
 *
 * Alloc* 返回一个由生产者临时拥有的帧块
 * 生产者填充 block->frame 后调用 Publish* 发布
 * Publish* 返回后，生产者不再拥有该 block，不能继续访问
 */
GloveRawFrameBlock_t *DataManager_AllocRawFrame(void);
GloveFullFrameBlock_t *DataManager_AllocFullFrame(void);

GloveStatus_t DataManager_PublishRawFrame(GloveRawFrameBlock_t *block, uint32_t timeout_ms);
GloveStatus_t DataManager_PublishFullFrame(GloveFullFrameBlock_t *block, uint32_t timeout_ms);

/*
 * 每次 Get* 成功后，消费者在使用完数据后必须调用一次对应的 Release*
 * 确保引用计数能够正确归还内存池
 */
GloveStatus_t DataManager_GetRawFrame(DataConsumer_t consumer,
                                      GloveRawFrameBlock_t **block,
                                      uint32_t timeout_ms);
GloveStatus_t DataManager_GetFullFrame(DataConsumer_t consumer,
                                       GloveFullFrameBlock_t **block,
                                       uint32_t timeout_ms);

GloveStatus_t DataManager_ReleaseRawFrame(GloveRawFrameBlock_t *block);
GloveStatus_t DataManager_ReleaseFullFrame(GloveFullFrameBlock_t *block);

void DataManager_GetStats(DataManagerStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* DATA_MANAGER_H */
