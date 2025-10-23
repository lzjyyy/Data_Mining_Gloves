#include "mqtt_network.h"
#include "w5500_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define CONNECT_RETRY_MAX 3
#define CONNECT_RETRY_DELAY_MS 200

static int parse_ip(const char* ip_str, uint8_t ip[4])
{
    int n[4];
    if (sscanf(ip_str, "%d.%d.%d.%d", &n[0], &n[1], &n[2], &n[3]) != 4)
        return -1;
    for (int i = 0; i < 4; i++)
        ip[i] = (uint8_t)n[i];
    return 0;
}

// recv 封装
int W5500_recv(Network* n, uint8_t* buffer, int len, int timeout_ms)
{
    int32_t available = getSn_RX_RSR(n->sock);
    if (available == 0) return 0;

    if (available > len) available = len;
    return recv(n->sock, buffer, available);
}

// send 封装
int W5500_send(Network* n, uint8_t* buffer, int len, int timeout_ms)
{
    // printf("W5500_send\r\n");
    int sent = 0;
    while (sent < len)
    {
        int rc = send(n->sock, buffer + sent, len - sent);
        if (rc <= 0) break;
        sent += rc;
    }
    return sent;
}

// close 封装
void W5500_close(Network* n)
{
    disconnect(n->sock);
}


void NetworkInit(Network* n) {
    n->sock = 0;
    n->mqttread = W5500_recv;
    n->mqttwrite = W5500_send;
    n->disconnect = W5500_close;
}

// int NetworkConnect(Network* n, const char* ip_str, uint16_t port) {
//     uint8_t ip[4];
//     if (parse_ip(ip_str, ip) != 0)
//         return -1; // IP 转换失败

//     close(n->sock);
//     if (socket(n->sock, Sn_MR_TCP, port, 0) != n->sock)
//         return -2;

//     int8_t conn_result = 0;
//     conn_result = connect(n->sock, ip, port);
//     if (conn_result != SOCK_OK)
//         return conn_result;

//     return 0;
// }

void NetworkDisconnect(Network* n) {
    disconnect(n->sock);
}

int NetworkConnect(Network* n, const char* ip_str, uint16_t port)
{
    uint8_t ip[4];
    // 将以字符串形式（例如"192.168.1.100"）提供的IP地址，解析并转换为W5500库函数所需的uint8_t数组格式
    if (parse_ip(ip_str, ip) != 0)
        return -1; // IP 转换失败

    int max_attempts = 5;
    int attempt;
    for (attempt = 1; attempt <= max_attempts; attempt++)
    {
        printf("[NetworkConnect] attempt %d: opening socket...\r\n", attempt);
        close(n->sock);

        // 等待 socket 真的关闭
        TickType_t start = xTaskGetTickCount();
        while (getSn_SR(n->sock) != SOCK_CLOSED) {
            if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(500)) {
                printf("Warning: socket %d close timeout, SR=0x%02X\r\n",
                    n->sock, getSn_SR(n->sock));
                break;
            }
            osDelay(10);
        }

        if (socket(n->sock, Sn_MR_TCP, port, 0) != n->sock)
        {
            printf("[NetworkConnect] socket() failed\r\n");
            continue;
        }

        printf("[NetworkConnect] connecting to %s:%d...\r\n", ip_str, port);
        int8_t conn_result = connect(n->sock, ip, port);

        uint8_t sr = getSn_SR(n->sock); // 获取socket状态寄存器 (Socket Status Register)
        uint8_t ir = getSn_IR(n->sock); // 获取socket中断寄存器 (Socket Interrupt Register)
        printf("[NetworkConnect] connect() result=%d, Sn_SR=0x%02X, Sn_IR=0x%02X\r\n",
            conn_result, sr, ir);

        if (conn_result == SOCK_OK && sr == SOCK_ESTABLISHED)
        {
            printf("[NetworkConnect] Socket %d connected to %s:%d\r\n", n->sock, ip_str, port);
            return 0; // 成功
        }
        else
        {
            printf("[NetworkConnect] Socket %d connection failed, closing...\r\n", n->sock);
            close(n->sock);
            osDelay(200); // 等待重试
        }
    }

    printf("[NetworkConnect] Failed to connect after %d attempts\r\n", max_attempts);
    return -2;
}
