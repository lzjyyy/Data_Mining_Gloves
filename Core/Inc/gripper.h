#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rs485.h"
#include <stdbool.h>
#include <stdint.h>

#define GRIPPER_DEFAULT_ADDR        0x01U
#define GRIPPER_BROADCAST_ADDR     0x00U
#define GRIPPER_PUBLIC_ADDR        0xFFU

#define GRIPPER_FRAME_TX_HEAD       0xAEU
#define GRIPPER_FRAME_RX_HEAD       0xACU
#define GRIPPER_MAX_PAYLOAD_LEN     248U
#define GRIPPER_MAX_FRAME_LEN       (5U + GRIPPER_MAX_PAYLOAD_LEN + 2U)
#define GRIPPER_DEFAULT_TIMEOUT_MS  200U
#define GRIPPER_DEFAULT_OPEN_COUNT  (-3000)
#define GRIPPER_DEFAULT_CLOSE_COUNT (-53000)
#define GRIPPER_DEFAULT_DEADBAND_PERCENT 2.0f

/* Legacy ZE300 Modbus register map kept so old debug code can still compile. */
#define REG_ENABLE                  0x0100
#define REG_POS_HIGH                0x0102
#define REG_POS_LOW                 0x0103
#define REG_SPEED                   0x0104
#define REG_TORQUE                  0x0105
#define REG_TRIGGER                 0x0108
#define REG_REALTIME_POS_HIGH       0x0609
#define REG_POS_REACHED             0x0602
#define REG_TORQUE_REACHED          0x0601
#define REG_SPEED_REACHED           0x0603
#define REG_WARNING_INFO            0x0612

typedef enum {
  GRIPPER_OK = 0,
  GRIPPER_ERROR = -1,
  GRIPPER_INVALID_ARG = -2,
  GRIPPER_TIMEOUT = -3,
  GRIPPER_CRC_ERROR = -4,
  GRIPPER_BAD_FRAME = -5,
} GripperResult_t;

typedef enum {
  GRIPPER_CMD_REBOOT = 0x00,
  GRIPPER_CMD_READ_VERSION = 0x0A,
  GRIPPER_CMD_READ_REALTIME = 0x0B,
  GRIPPER_CMD_CLEAR_FAULT = 0x0F,
  GRIPPER_CMD_READ_USER_PARAMS = 0x10,
  GRIPPER_CMD_WRITE_USER_PARAMS = 0x11,
  GRIPPER_CMD_READ_MOTOR_PARAMS = 0x12,
  GRIPPER_CMD_WRITE_MOTOR_PARAMS = 0x13,
  GRIPPER_CMD_READ_MOTION_PARAMS = 0x14,
  GRIPPER_CMD_WRITE_MOTION_PARAMS_TEMP = 0x15,
  GRIPPER_CMD_WRITE_MOTION_PARAMS_SAVE = 0x16,
  GRIPPER_CMD_SET_ZERO = 0x1D,
  GRIPPER_CMD_ENCODER_CALIB = 0x1E,
  GRIPPER_CMD_RESTORE_DEFAULT = 0x1F,
  GRIPPER_CMD_Q_CURRENT = 0x20,
  GRIPPER_CMD_SPEED = 0x21,
  GRIPPER_CMD_MOVE_ABSOLUTE = 0x22,
  GRIPPER_CMD_MOVE_RELATIVE = 0x23,
  GRIPPER_CMD_HOME_SHORTEST = 0x24,
  GRIPPER_CMD_BRAKE = 0x2E,
  GRIPPER_CMD_DISABLE = 0x2F,
} GripperCommand_t;

typedef struct {
  uint16_t single_turn_raw;
  int32_t multi_turn_count;
  int32_t speed_raw;
  int32_t q_current_raw;
  uint16_t bus_voltage_raw;
  uint16_t bus_current_raw;
  uint8_t temperature_c;
  uint8_t run_state;
  uint8_t motor_enabled;
  uint8_t fault_code;
} GripperRealtime_t;

typedef struct {
  uint8_t rs485_ch;
  uint8_t device_addr;
  uint16_t timeout_ms;
  uint8_t next_seq;
  GripperRealtime_t last_status;
  GripperResult_t last_error;
  int32_t open_position_count;
  int32_t close_position_count;
  float deadband_percent;
  uint8_t calibrated;
} GripperHandle_t;

typedef struct {
  float position_kp;
  float position_ki;
  uint32_t position_output_limit_raw;
  float velocity_kp;
  float velocity_ki;
  uint32_t velocity_output_limit_raw;
} GripperMotionParams_t;

typedef struct {
  uint16_t boot_ver;
  uint16_t software_ver;
  uint16_t hardware_ver;
  uint8_t aux_rs485_ver;
  uint8_t modbus_rs485_ver;
  uint8_t aux_can_ver;
  uint8_t canopen_ver;
  uint8_t uid[12];
} GripperVersion_t;

typedef struct {
  float search_speed_rpm;
  int8_t search_direction;
  uint16_t poll_interval_ms;
  uint16_t timeout_ms;
  float speed_epsilon_rpm;
  float current_threshold_a;
  int32_t position_epsilon_count;
  uint8_t detect_consecutive_samples;
  uint8_t clear_fault_before_start;
  uint8_t set_zero_after_detect;
  int32_t backoff_count_after_zero;
} GripperCalibrateConfig_t;

typedef struct {
  uint8_t limit_detected;
  uint8_t zero_set;
  uint8_t backoff_done;
  uint8_t detect_samples;
  int32_t limit_count_before_zero;
  uint16_t mechanical_offset;
  GripperRealtime_t final_status;
} GripperCalibrateResult_t;

typedef struct {
  GripperCalibrateConfig_t calibrate;
  int32_t close_position_count;
  float close_speed_limit_rpm;
  float close_current_limit_a;
  uint8_t close_after_calibrate;
} GripperStartupConfig_t;

void Gripper_Init(GripperHandle_t* handle,
                  uint8_t rs485_ch,
                  uint8_t device_addr,
                  uint16_t timeout_ms);
uint8_t Gripper_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size);

GripperResult_t Gripper_ReadRealtime(GripperHandle_t* handle,
                                      GripperRealtime_t* status);
GripperResult_t Gripper_ReadVersion(GripperHandle_t* handle,
                                    GripperVersion_t* version);
GripperResult_t Gripper_ClearFault(GripperHandle_t* handle, uint8_t* fault_code);
GripperResult_t Gripper_MoveAbsolute(GripperHandle_t* handle,
                                      int32_t target_count,
                                      GripperRealtime_t* status);
GripperResult_t Gripper_MoveAbsoluteWithLimits(GripperHandle_t* handle,
                                               int32_t target_count,
                                               float max_speed_rpm,
                                               float max_current_amp,
                                               GripperRealtime_t* status);
GripperResult_t Gripper_MoveAbsoluteWithLimitsRaw(GripperHandle_t* handle,
                                                  int32_t target_count,
                                                  uint32_t speed_limit_raw,
                                                  uint32_t current_limit_raw,
                                                  GripperRealtime_t* status);
GripperResult_t Gripper_MoveRelative(GripperHandle_t* handle,
                                      int32_t delta_count,
                                      GripperRealtime_t* status);
GripperResult_t Gripper_GoHomeShortest(GripperHandle_t* handle,
                                       GripperRealtime_t* status);
GripperResult_t Gripper_SetSpeed(GripperHandle_t* handle,
                                  float rpm,
                                  uint32_t accel_0p01rpm_per_sec,
                                  GripperRealtime_t* status);
GripperResult_t Gripper_SetSpeedWithCurrentLimit(GripperHandle_t* handle,
                                                 float rpm,
                                                 uint32_t accel_0p01rpm_per_sec,
                                                 float current_limit_a,
                                                 GripperRealtime_t* status);
GripperResult_t Gripper_SetQCurrent(GripperHandle_t* handle,
                                     float amp,
                                     uint32_t slope_milliamp_per_sec,
                                     GripperRealtime_t* status);
GripperResult_t Gripper_Disable(GripperHandle_t* handle,
                                 GripperRealtime_t* status);
GripperResult_t Gripper_SetZero(GripperHandle_t* handle, uint16_t* mechanical_offset);
GripperResult_t Gripper_ReadMotionParams(GripperHandle_t* handle,
                                         GripperMotionParams_t* params);
GripperResult_t Gripper_WriteMotionParamsTemp(GripperHandle_t* handle,
                                              const GripperMotionParams_t* params,
                                              GripperMotionParams_t* echo);
GripperResult_t Gripper_WriteMotionParamsSave(GripperHandle_t* handle,
                                              const GripperMotionParams_t* params,
                                              GripperMotionParams_t* echo);
void Gripper_CalibrateConfigDefault(GripperCalibrateConfig_t* config);
void Gripper_StartupConfigDefault(GripperStartupConfig_t* config);
GripperResult_t Gripper_CalibrateLimit(GripperHandle_t* handle,
                                       const GripperCalibrateConfig_t* config,
                                       GripperCalibrateResult_t* result);
GripperResult_t Gripper_StartupCalibrateOpenClose(GripperHandle_t* handle,
                                                  const GripperStartupConfig_t* config,
                                                  GripperCalibrateResult_t* result);
GripperResult_t Gripper_Reboot(GripperHandle_t* handle);

int32_t Gripper_RpmToRaw(float rpm);
int32_t Gripper_AmpToRaw(float amp);
float Gripper_RawToRpm(int32_t raw);
float Gripper_RawToAmp(int32_t raw);
float Gripper_CountToDeg(int32_t count);
void Gripper_SetPositionProfile(GripperHandle_t* handle,
                                int32_t open_count,
                                int32_t close_count);
void Gripper_SetDeadbandPercent(GripperHandle_t* handle, float deadband_percent);
int32_t Gripper_PercentToCount(const GripperHandle_t* handle, float percent);
float Gripper_CountToPercent(const GripperHandle_t* handle, int32_t count);
uint8_t Gripper_IsPercentInDeadband(const GripperHandle_t* handle,
                                    float target_percent,
                                    int32_t current_count);
GripperResult_t Gripper_MoveToPercent(GripperHandle_t* handle,
                                      float percent,
                                      GripperRealtime_t* status);
GripperResult_t Gripper_MoveToPercentWithLimits(GripperHandle_t* handle,
                                                float percent,
                                                float max_speed_rpm,
                                                float max_current_amp,
                                                GripperRealtime_t* status);
GripperResult_t Gripper_Open(GripperHandle_t* handle,
                             GripperRealtime_t* status);
GripperResult_t Gripper_Close(GripperHandle_t* handle,
                              GripperRealtime_t* status);

#ifdef __cplusplus
}
#endif

#endif
