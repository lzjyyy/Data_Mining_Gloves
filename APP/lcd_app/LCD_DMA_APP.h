#ifndef __LCD_DMA_APP_H__
#define __LCD_DMA_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LCD_DMA_QUEUE_DEPTH 8U
#define LCD_DMA_MAX_CHUNK   0xFFFFU

HAL_StatusTypeDef LCD_DMA_APP_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data, uint32_t len);
void LCD_DMA_APP_TxCpltCallback(SPI_HandleTypeDef *hspi);
void LCD_DMA_APP_ErrorCallback(SPI_HandleTypeDef *hspi);
uint8_t LCD_DMA_APP_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif
