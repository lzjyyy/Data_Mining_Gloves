#ifndef __APP_TASK_H__
#define __APP_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void AppTask_Init(void);
void AppTask_GetCreateStatus(uint8_t *rs485_created, uint8_t *lcd_created);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H__ */
