#include "app_data.h"

#include <stddef.h>
#include <string.h>

/* 清空原始帧，避免复用内存池中的旧数据 */
void AppData_ClearRawFrame(GloveRawFrame_t *frame)
{
    if (frame != NULL)
    {
        (void)memset(frame, 0, sizeof(*frame));
    }
}

/* 清空算法结果帧 */
void AppData_ClearProcessedFrame(GloveProcessedFrame_t *frame)
{
    if (frame != NULL)
    {
        (void)memset(frame, 0, sizeof(*frame));
    }
}

/* 清空完整帧 */
void AppData_ClearFullFrame(GloveFullFrame_t *frame)
{
    if (frame != NULL)
    {
        (void)memset(frame, 0, sizeof(*frame));
    }
}

/*
 * 组装完整帧
 *
 * 算法任务先消费 RawFrame，计算出 ProcessedFrame，
 * 然后通过这个函数合成为 FullFrame，最后再发布给 SD 和 RS485
 */
void AppData_BuildFullFrame(GloveFullFrame_t *full,
                            const GloveRawFrame_t *raw,
                            const GloveProcessedFrame_t *processed)
{
    if ((full == NULL) || (raw == NULL) || (processed == NULL))
    {
        return;
    }

    /* FullFrame 的公共帧头以原始采样帧为准，保证采样时间对齐 */
    full->frame_id = raw->frame_id;
    full->timestamp_us = raw->timestamp_us;
    full->valid_flags = raw->valid_flags | processed->valid_flags;
    full->raw = *raw;
    full->processed = *processed;
}
