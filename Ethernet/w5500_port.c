#include "w5500_port.h"
#include "wizchip_conf.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

#define TCP_RETRY_TIME 2000   // SYN重试时间，单位：ms
#define TCP_RETRY_COUNT 3     // SYN重试次数
/* Socket 寄存器基地址偏移 */
#define SOCK_BASE(sock)   (0x100 * (sock))
#define Sn_TX_RTR(sock)   (SOCK_BASE(sock) + 0x001D)  // TCP SYN 重试时间高字节
#define Sn_TX_RTR_L(sock) (SOCK_BASE(sock) + 0x001E)  // TCP SYN 重试时间低字节
#define Sn_TX_RCR(sock)   (SOCK_BASE(sock) + 0x001F)  // TCP SYN 重试次数
#define Sn_TX_RTR 0x001D   // 8-bit寄存器，TCP重试时间单位 100μs
#define Sn_TX_RCR 0x001E   // 8-bit寄存器，SYN重试次数

/* SPI 互斥量 */
SemaphoreHandle_t spiMutex;

/* ========== 片选/复位辅助 ========== */
static inline void W5500_Select(void)
{
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

static inline void W5500_DeSelect(void)
{
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

static inline void W5500_HwReset(void)
{
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
}

/* ========== SPI 回调 ========== */
static uint8_t W5500_ReadByte(void)
{
    uint8_t tx = 0xFF, rx = 0;
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, HAL_MAX_DELAY);
    xSemaphoreGive(spiMutex);
    return rx;
}

static void W5500_WriteByte(uint8_t byte)
{
    uint8_t rx;
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi2, &byte, &rx, 1, HAL_MAX_DELAY);
    xSemaphoreGive(spiMutex);
}

/* ========== 临界区（可选） ========== */
static void wizchip_critical_enter(void)
{
    __disable_irq();
}

static void wizchip_critical_exit(void)
{
    __enable_irq();
}

/* ========== ioLibrary 适配 ========== */
static void wizchip_select(void)
{
    W5500_Select();
}

static void wizchip_deselect(void)
{
    W5500_DeSelect();
}

static uint8_t wizchip_read(void)
{
    return W5500_ReadByte();
}

static void wizchip_write(uint8_t wb)
{
    W5500_WriteByte(wb);
}

static void wizchip_write_reg(uint8_t sock, uint16_t addr, uint8_t val)
{
    uint8_t ctrl[3];
    ctrl[0] = (addr >> 8) & 0xFF;
    ctrl[1] = addr & 0xFF;
    ctrl[2] = (sock << 5) | 0x04; // Socket n + write + common
    wizchip_select();
    for (int i = 0;i < 3;i++)
        W5500_WriteByte(ctrl[i]);
    W5500_WriteByte(val);
    wizchip_deselect();
}

static void setSn_TX_RTR(uint8_t sock, uint8_t value)
{
    wizchip_write_reg(sock, Sn_TX_RTR, value);
}

static void setSn_TX_RCR(uint8_t sock, uint8_t value)
{
    wizchip_write_reg(sock, Sn_TX_RCR, value);
}

/* ========== 初始化 ========== */
int W5500_DriverInit(void)
{
    /* 创建 SPI 互斥量 */
    spiMutex = xSemaphoreCreateMutex();

    /* 注册回调 */
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
    reg_wizchip_cris_cbfunc(wizchip_critical_enter, wizchip_critical_exit);

    /* 复位硬件 */
    W5500_HwReset();

    /* 缓冲分配（TX/RX）：8个 socket */
    uint8_t memsize[2][8] = {
        {8,0,0,0,0,0,0,0},  // TX buffer
        {8,0,0,0,0,0,0,0}   // RX buffer
    };

    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1) {
        printf("WIZCHIP Init failed.\r\n");
        return -1; // 缓冲分配失败
    }

    uint8_t version = getVERSIONR();
    printf("W5500 Version: 0x%02X\r\n", version);

    /* ====== PHY 配置 ====== */
    wiz_PhyConf phyconf;
    phyconf.by = PHY_CONFBY_SW;   // 软件配置
    phyconf.mode = PHY_MODE_MANUAL; // 手动
    // phyconf.mode = PHY_MODE_AUTONEGO; // 自动协商
    phyconf.speed = PHY_SPEED_10;   // 10M
    phyconf.duplex = PHY_DUPLEX_FULL; // 全双工
    int8_t result = 0;
    result = ctlwizchip(CW_SET_PHYCONF, (void*)&phyconf);
    if (result != 0)
    {
        printf("Set PHY config failed!\r\n");
        return -2;
    }

    /* ====== 设置 TCP SYN 超时参数 ====== */
    // for (uint8_t sock = 0; sock < 8; sock++) {
    //     setSn_TX_RTR(sock, TCP_RETRY_TIME / 100); // 100μs单位
    //     setSn_TX_RCR(sock, TCP_RETRY_COUNT);
    // }


    /* 检测物理层,等待 PHY 链路 */
    uint8_t link;
    uint32_t link_chk_cnt = 0;
    do {
        HAL_Delay(200);
        result = 0;
        result = ctlwizchip(CW_GET_PHYLINK, (void*)&link);
        link_chk_cnt++;
        printf("Link: %d,chk_cnt:%d,result:%d\r\n", link, link_chk_cnt, result);
    } while (link == PHY_LINK_OFF && (link_chk_cnt < MAX_LINK_CHK_CNT));

    if (result != 0 || (link == 0))
    {
        printf("Link on failed!\r\n");
        return -3;
    }
    return 0;
}

/* ========== 网络参数（静态IP） ========== */
void W5500_NetInfo_SetStatic(void)
{
    wiz_NetInfo netinfo = {
        .mac = {0x00,0x08,0xDC,0x11,0x22,0x33},
        .ip = {192,168,1,123},
        .sn = {255,255,255,0},
        .gw = {192,168,1,1},
        .dns = {8,8,8,8},
        .dhcp = NETINFO_STATIC
    };
    ctlnetwork(CN_SET_NETINFO, (void*)&netinfo);
}

/* ========== 打印网络参数 ========== */
void W5500_PrintNetInfo(void)
{
    wiz_NetInfo net;
    ctlnetwork(CN_GET_NETINFO, &net);
    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
        net.mac[0], net.mac[1], net.mac[2], net.mac[3], net.mac[4], net.mac[5]);
    printf("IP : %d.%d.%d.%d\r\n", net.ip[0], net.ip[1], net.ip[2], net.ip[3]);
    printf("SN : %d.%d.%d.%d\r\n", net.sn[0], net.sn[1], net.sn[2], net.sn[3]);
    printf("GW : %d.%d.%d.%d\r\n", net.gw[0], net.gw[1], net.gw[2], net.gw[3]);
    printf("DNS: %d.%d.%d.%d\r\n", net.dns[0], net.dns[1], net.dns[2], net.dns[3]);
}

/* ========== 初始化后提升 SPI 速率（可选） ========== */
/* 例如把 SPI Prescaler 从 64 提升到 8/4，视板子走线质量与长度而定 */
void W5500_RaiseSpiSpeed(void)
{
    // 以 HAL 为例：先禁用 SPI，改 CR1 分频，再启用（不同 HAL 版本略有差异）
    // 注意：在 CubeMX 自动生成的 hspi2.Init.BaudRatePrescaler 基础上修改
    __HAL_SPI_DISABLE(&hspi2);
    MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_2);
    __HAL_SPI_ENABLE(&hspi2);
}

/* ========== TCP Echo Server 示例（Socket 0, Port 5000） ========== */
void W5500_TCP_EchoServer_Loop(void)
{
    static uint8_t sock = 0;
    static uint8_t state = 0;
    uint8_t buf[1024];

    if (state == 0) {
        close(sock);
        if (socket(sock, Sn_MR_TCP, 5000, 0) != sock) return;
        if (listen(sock) != SOCK_OK) return;
        state = 1;
        return;
    }

    switch (getSn_SR(sock)) {
    case SOCK_ESTABLISHED: {
        int32_t len = getSn_RX_RSR(sock);
        if (len > 0) {
            if (len > sizeof(buf)) len = sizeof(buf);
            len = recv(sock, buf, len);
            if (len > 0) {
                // 回显
                send(sock, buf, len);
            }
        }
        break;
    }
    case SOCK_CLOSE_WAIT:
        // 对端关闭，主动关
        disconnect(sock);
        state = 0;
        break;
    case SOCK_CLOSED:
    case SOCK_INIT:
    case SOCK_LISTEN:
    default:
        // 维持监听
        break;
    }
}

int W5500_TCP_Connect_Debug(uint8_t sock, uint8_t* ip, uint16_t port, uint32_t timeout_ms)
{
    close(sock);

    if (socket(sock, Sn_MR_TCP, port, 0) != sock) {
        printf("Socket %d open failed\r\n", sock);
        return -1;
    }

    printf("Socket %d opened, connecting to %d.%d.%d.%d:%d ...\r\n",
        sock, ip[0], ip[1], ip[2], ip[3], port);

    setSn_DIPR(sock, ip);
    setSn_DPORT(sock, port);
    setSn_CR(sock, Sn_CR_CONNECT);

    uint32_t start = HAL_GetTick();
    while (getSn_CR(sock)); // 等待命令完成

    while (1) {
        uint8_t status = getSn_SR(sock);
        uint8_t ir = getSn_IR(sock);

        printf("Sn_SR=%02X, Sn_IR=%02X\r\n", status, ir);

        if (status == SOCK_ESTABLISHED) {
            printf("Socket %d connected!\r\n", sock);
            return 0;
        }

        if (status == SOCK_CLOSED) {
            printf("Socket %d closed! IR=0x%02X\r\n", sock, ir);
            return -2;
        }

        if (ir & Sn_IR_TIMEOUT) {
            setSn_IR(sock, Sn_IR_TIMEOUT);
            printf("Socket %d timeout!\r\n", sock);
            return -3;
        }

        if ((HAL_GetTick() - start) > timeout_ms) {
            printf("Socket %d connect timeout after %lu ms\r\n", sock, timeout_ms);
            return -4;
        }
    }
}

uint8_t W5500_Get_PHYCFGR(void)
{
    uint8_t val = 0;
    uint16_t addr = W5500_PHYCFGR;
    uint8_t ctrl[3];

    ctrl[0] = (addr >> 8) & 0xFF;
    ctrl[1] = addr & 0xFF;
    ctrl[2] = 0x00;

    wizchip_select();
    for (int i = 0; i < 3; i++)
        W5500_WriteByte(ctrl[i]);

    val = W5500_ReadByte();
    wizchip_deselect();

    return val;
}