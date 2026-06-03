# Task/inc/test_task.h 解析

## 文件职责

`test_task.h` 声明测试任务入口函数。该任务用于验证 DataManager、合帧、算法模拟和 FullFrame 多消费者发布链路。

## 函数声明

### `StartTestTask(void *argument)`

测试线程入口。

- 实现在 `test_task.c`。
- 当前 `.c` 文件中主要测试流程被注释，任务实际只周期延时。
- 如果恢复注释代码，可用于持续跑一帧完整数据流自测。
