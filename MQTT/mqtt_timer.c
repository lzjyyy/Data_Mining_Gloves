#include "mqtt_timer.h"
#include "FreeRTOS.h"

void TimerInit(Timer* t)
{
    t->start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    t->timeout_ms = 0;
}

char TimerIsExpired(Timer* t)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return ((now - t->start_ms) >= t->timeout_ms) ? 1 : 0;
}

void TimerCountdownMS(Timer* t, uint32_t ms)
{
    t->start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    t->timeout_ms = ms;
}

void TimerCountdown(Timer* t, uint32_t sec)
{
    TimerCountdownMS(t, sec * 1000);
}

int TimerLeftMS(Timer* t)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - t->start_ms) >= t->timeout_ms) return 0;
    return (int)(t->timeout_ms - (now - t->start_ms));
}
