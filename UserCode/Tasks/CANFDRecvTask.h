#ifndef __CANFD_RECV_TASK_H__
#define __CANFD_RECV_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CANFD接收处理任务
 * @param argument 任务入口参数，未使用
 * @retval 无
 *
 * @note
 * 该任务完成以下功能：
 * 1. 初始化 CANFD 协议上下文；
 * 2. 轮询 FDCAN1 接收缓存；
 * 3. 当收到指定 ID（如 0x100）的命令帧时，调用 CANFD 协议分发器；
 * 4. 若有应答数据，则通过 FDCAN1 回发固定应答 ID（如 0x180）。
 */
void StartCANFDRecvTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __CANFD_RECV_TASK_H__ */