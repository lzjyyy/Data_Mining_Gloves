#ifndef __MQTT_NETWORK_H__
#define __MQTT_NETWORK_H__

#include <stdint.h>

typedef struct Network Network;

struct Network {
    int (*mqttread)(Network*, uint8_t*, int, int);
    int (*mqttwrite)(Network*, uint8_t*, int, int);
    void (*disconnect)(Network*);
    int sock;
};

void NetworkInit(Network* n);
int NetworkConnect(Network* n, const char* ip, uint16_t port);
void NetworkDisconnect(Network* n);

#endif
