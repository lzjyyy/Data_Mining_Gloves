#include "timers_APP.h"
#include "tim.h"

static volatile uint8_t lcd_refresh_event = 0U;

void Timers_APP_Init(void)
{
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t Timers_APP_TakeLcdRefreshEvent(void)
{
    uint8_t event;

    __disable_irq();
    event = lcd_refresh_event;
    lcd_refresh_event = 0U;
    __enable_irq();

    return event;
}

void Timers_APP_OnPeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        lcd_refresh_event = 1U;
    }
}
