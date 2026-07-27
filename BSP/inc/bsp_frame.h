#ifndef __FRAME_H
#define __FRAME_H

//include file
#include "main.h"
#include "bsp_status.h"
typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[FRAME_MAX_DATA];
} Frame_t;

extern uint8_t FRAME_ACK_DATA[4];
extern uint8_t FILE_ACK_FRAME[10];
extern Frame_t ft;


void bsp_frame_init(void);
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length);
ProtocolStatus_t ParseFrame(const uint8_t* frame_buf, uint16_t frame_len, Frame_t* out_frame);
void Modbus_ProcessReadCommConfig(const Frame_t *frame);
uint16_t PackFrame(uint8_t* out_frame, uint8_t cmd, const uint8_t* data, uint8_t len);
void frame_process(Frame_t *frame);
void Append_CRC16_To_Frame(uint8_t *frame);
void frame_update_request_ack(Frame_t *frame);
void frame_FILE_INFO_ack(Frame_t *frame);
void frame_data_ack(Frame_t *frame);
void frame_VERIFY_CRC32_ack(Frame_t *frame);
void send_ack(uint8_t cmd, const uint8_t *data, uint8_t len, const char *tag);
void send_error_ack(uint8_t cmd_code, uint8_t state, uint8_t cmd, const char *tag);
void send_frameID_error_ack(uint8_t cmd_code, uint8_t state, uint8_t cmd,uint16_t *frameID, const char *tag);
void update_error_count(ProtocolStatus_t error_code);
void Stop_UpgradeAndReset(void);
#endif

