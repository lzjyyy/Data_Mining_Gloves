#include "LedTask.h"
#include "ws2816a_spi_test.h"  
#include "main.h"


static volatile LedMode_t led_mode = LED_MODE_OFF;
int led_brightness = 15;

/* ========== 内部函数 ========== */

static void LedTask_ApplyColor(uint16_t r, uint16_t g, uint16_t b)
{   
		WS2816A_SetPixel(0, r, g, b);                
		WS2816A_SetPixel(1, r, g, b);
    WS2816A_Show();
}

static void LedTask_ApplyModeOnce(LedMode_t mode)
{
    switch (mode)
    {
        case LED_MODE_OFF:
            LedTask_ApplyColor(0, 0, 0);
            break;

        case LED_MODE_BOOT:
            LedTask_ApplyColor(0, 400, 1000);
            break;

        case LED_MODE_RUN:
            LedTask_ApplyColor(0, 1000, 200);
            break;

        case LED_MODE_ERROR:
            LedTask_ApplyColor(1000, 0, 0);
            break;

        case LED_MODE_IDLE:
						LedTask_ApplyColor(1000, 1000, 1000);
            break;
				
        case LED_MODE_RESET:
						LedTask_ApplyColor(600, 140, 1000);
            break;
				
        default:
            LedTask_ApplyColor(0, 0, 0);
            break;
    }
}

/* ========== 对外接口 ========== */

void LedTask_SetMode(LedMode_t mode)
{
    if (led_mode == mode)
    {
        return;
    }

    led_mode = mode;
}

LedMode_t LedTask_GetMode(void)
{
    return led_mode;
}

void LedTask_SetModeFromISR(LedMode_t mode)
{
    if (led_mode == mode)
    {
        return;
    }

    led_mode = mode;
}
/* ========== FreeRTOS 任务函数 ========== */

void StartLedTask(void *argument)
{
    (void)argument;
		LedMode_t last_mode;
    WS2816A_SPI_Init();
		WS2816A_SetPixelGain(0, led_brightness, led_brightness, led_brightness);  
		WS2816A_SetPixel(0, 0, 1000, 200);                
		WS2816A_SetPixel(1, 0, 1000, 200);
    LedTask_ApplyModeOnce(led_mode);
    last_mode = led_mode;


    for (;;)
    {

        LedMode_t current_mode = led_mode;

        if (current_mode != last_mode)
        {
            LedTask_ApplyModeOnce(current_mode);
            last_mode = current_mode;
        }

        osDelay(50);
    }
}

