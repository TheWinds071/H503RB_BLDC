#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"
#include "motor_encoder_drive.h"

#define MOTOR_CAN_CMD_ID 0x201U
#define MOTOR_CAN_STATUS_ID 0x181U
#define MOTOR_CAN_STATUS_PERIOD_MS 50U

typedef enum {
  MOTOR_CAN_MODE_IDLE = 0U,
  MOTOR_CAN_MODE_POSITION = 1U,
  MOTOR_CAN_MODE_SPEED = 2U,
} motor_can_mode_t;

HAL_StatusTypeDef motor_can_init(FDCAN_HandleTypeDef *hfdcan);
void motor_can_process(motor_encoder_drive_t *drive);
uint8_t motor_can_is_active(void);

#ifdef __cplusplus
}
#endif

#endif
