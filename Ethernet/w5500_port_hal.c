#include "w5500_port_hal.h"
#include "socket.h"

#define W5500_CS_L HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET);
#define W5500_CS_H HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET);
wiz_NetInfo gWIZNETINFO = { .mac = {0x78, 0x83, 0x68, 0x88, 0x56, 0x72},
                            .ip = {192, 168, 1,136},
                            .sn = {255,255,255,0},
                            .gw = {192, 168,1,1},
                            .dns = {180,76,76,76},
                            .dhcp = NETINFO_DHCP };


uint16_t Net_Status; 									//SOCKET 返回值
uint8_t destip[4] = { 192, 168, 1, 108 };  //tcp模式 服务器ip
uint16_t destport = 5000;									//服务器端口
uint8_t buffer[256];											//数据缓冲区


void SPI_WriteByte(uint8_t TxData)										//SPI写一字节
{
    uint8_t data;
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &data, 1, 100);
}


uint8_t SPI_ReadByte(void)														//SPI 读一字节
{
    uint8_t data[2] = { 0xFF,0xFF };
    HAL_SPI_TransmitReceive(&hspi1, data, data + 1, 1, 100);
    return data[1];
}


void SPI_CrisEnter(void)																// 
{
    __set_PRIMASK(1);
}


void SPI_CrisExit(void)
{
    __set_PRIMASK(0);
}


void SPI_CS_Select(void)
{
    W5500_CS_L;
}


void SPI_CS_Deselect(void)
{
    W5500_CS_H;
}
//W5500复位
void W5500_RESET(void)
{
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}


//初始化芯片参数
void ChipParametersConfiguration(void)
{
    uint8_t tmp;
    uint8_t memsize[2][8] = { {2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2} };
    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1)
    {
        while (1);
    }


    do {
        if (ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1)
        {
            while (1);
        }
    } while (tmp == PHY_LINK_OFF);
}


//初始化网络参数 mac ip等
void NetworkParameterConfiguration(void)  //Intialize the network information to be used in WIZCHIP
{
    uint8_t tmpstr[6];


    ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
    ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO);
    ctlwizchip(CW_GET_ID, (void*)tmpstr);
}


//W5500初始化
void W5500_ChipInit(void)
{
    W5500_RESET();
    reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit); //注册临界函数
    reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect); //注册SPI片选函数
    reg_wizchip_spi_cbfunc(SPI_ReadByte, SPI_WriteByte); //注册SPI读写函数
    ChipParametersConfiguration();//初始化芯片
    NetworkParameterConfiguration();//初始化网络参数配置 MAC IP等
}

void do_tcpc(void)
{
    uint16_t Len;


    Net_Status = getSn_SR(0);


    switch (Net_Status)                  /*获取socket的状态*/
    {
    case SOCK_CLOSED:                   /*socket处于关闭状态*/
        socket(0, Sn_MR_TCP, 8123, Sn_MR_ND);
        break;
    case SOCK_INIT:                      /*socket处于初始化状态*/
        connect(0, destip, destport);/*socket连接服务器*/
        break;
    case SOCK_ESTABLISHED:               /*socket处于连接建立状态*/
        if (getSn_IR(0) & Sn_IR_CON)
        {
            setSn_IR(0, Sn_IR_CON);      /*清除接收中断标志位*/
        }
        Len = getSn_RX_RSR(0);            /*获取接收的数据长度*/
        if (Len > 0)  //接收到数据
        {
            recv(0, buffer, Len);     /*接收来自Server的数据*/
            send(0, buffer, Len);    //返回给服务器
        }


        break;
    case SOCK_CLOSE_WAIT:        /*socket处于等待关闭状态*/
        close(0);
        break;
    }
}