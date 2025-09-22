#ifndef MQTT_TIMER_H
#define MQTT_TIMER_H

#include "stdint.h"

typedef struct {
    uint32_t start_ms;
    uint32_t timeout_ms;
} Timer;

void TimerInit(Timer* t);
char TimerIsExpired(Timer* t);
void TimerCountdownMS(Timer* t, uint32_t ms);
void TimerCountdown(Timer* t, uint32_t sec);
int TimerLeftMS(Timer* t);

#endif
