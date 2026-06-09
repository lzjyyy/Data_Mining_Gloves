#include "LCD_DMA_APP.h"

typedef struct
{
  SPI_HandleTypeDef *hspi;
  uint8_t *data;
  uint16_t len;
} LCD_DMA_QueueItemTypeDef;

static LCD_DMA_QueueItemTypeDef lcd_dma_queue[LCD_DMA_QUEUE_DEPTH];
static volatile uint8_t lcd_dma_head = 0U;
static volatile uint8_t lcd_dma_tail = 0U;
static volatile uint8_t lcd_dma_count = 0U;
static volatile uint8_t lcd_dma_busy = 0U;
static volatile HAL_StatusTypeDef lcd_dma_error = HAL_OK;

static HAL_StatusTypeDef LCD_DMA_APP_StartNext(void)
{
  LCD_DMA_QueueItemTypeDef item;
  HAL_StatusTypeDef status;

  if ((lcd_dma_busy != 0U) || (lcd_dma_count == 0U))
  {
    return HAL_OK;
  }

  item = lcd_dma_queue[lcd_dma_tail];
  lcd_dma_tail = (uint8_t)((lcd_dma_tail + 1U) % LCD_DMA_QUEUE_DEPTH);
  lcd_dma_count--;
  lcd_dma_busy = 1U;

  status = HAL_SPI_Transmit_DMA(item.hspi, item.data, item.len);
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(item.hspi->hdmatx, DMA_IT_HT);
  }
  else
  {
    lcd_dma_busy = 0U;
    lcd_dma_error = status;
  }

  return status;
}

static HAL_StatusTypeDef LCD_DMA_APP_Enqueue(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t len)
{
  if ((hspi == NULL) || (data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  while (lcd_dma_count >= LCD_DMA_QUEUE_DEPTH)
  {
    if (lcd_dma_error != HAL_OK)
    {
      return lcd_dma_error;
    }
  }

  __disable_irq();
  lcd_dma_queue[lcd_dma_head].hspi = hspi;
  lcd_dma_queue[lcd_dma_head].data = data;
  lcd_dma_queue[lcd_dma_head].len = len;
  lcd_dma_head = (uint8_t)((lcd_dma_head + 1U) % LCD_DMA_QUEUE_DEPTH);
  lcd_dma_count++;
  __enable_irq();

  return LCD_DMA_APP_StartNext();
}

HAL_StatusTypeDef LCD_DMA_APP_Transmit(SPI_HandleTypeDef *hspi, uint8_t *data, uint32_t len)
{
  HAL_StatusTypeDef status = HAL_OK;

  if ((hspi == NULL) || (data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  lcd_dma_error = HAL_OK;

  while (len > 0U)
  {
    uint16_t chunk = (len > LCD_DMA_MAX_CHUNK) ? LCD_DMA_MAX_CHUNK : (uint16_t)len;

    status = LCD_DMA_APP_Enqueue(hspi, data, chunk);
    if (status != HAL_OK)
    {
      return status;
    }

    data += chunk;
    len -= chunk;
  }

  while ((lcd_dma_busy != 0U) || (lcd_dma_count != 0U))
  {
    if (lcd_dma_error != HAL_OK)
    {
      return lcd_dma_error;
    }
  }

  return lcd_dma_error;
}

void LCD_DMA_APP_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI1))
  {
    lcd_dma_busy = 0U;
    (void)LCD_DMA_APP_StartNext();
  }
}

void LCD_DMA_APP_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi != NULL) && (hspi->Instance == SPI1))
  {
    lcd_dma_error = HAL_ERROR;
    lcd_dma_busy = 0U;
  }
}

uint8_t LCD_DMA_APP_IsBusy(void)
{
  return (uint8_t)((lcd_dma_busy != 0U) || (lcd_dma_count != 0U));
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  LCD_DMA_APP_TxCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  LCD_DMA_APP_ErrorCallback(hspi);
}
