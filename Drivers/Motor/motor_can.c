#include "motor_can.h"

#include <string.h>

typedef struct {
  uint8_t mode;
  uint8_t enable;
  int32_t value;
  uint32_t seq;
} motor_can_command_t;

static FDCAN_HandleTypeDef *motor_can_hfdcan = NULL;
static volatile motor_can_command_t motor_can_pending_cmd = {0U, 0U, 0, 0U};
static volatile uint8_t motor_can_cmd_pending = 0U;
static uint8_t motor_can_active = 0U;
static uint8_t motor_can_mode = MOTOR_CAN_MODE_IDLE;
static int32_t motor_can_target_value = 0;
static uint32_t motor_can_rx_count = 0U;
static uint32_t motor_can_tx_count = 0U;
static uint32_t motor_can_last_status_ms = 0U;

static int32_t motor_can_read_i32_le(const uint8_t *data) {
  return (int32_t)((uint32_t)data[0] |
                   ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24));
}

static void motor_can_write_i32_le(uint8_t *data, int32_t value) {
  data[0] = (uint8_t)((uint32_t)value & 0xFFU);
  data[1] = (uint8_t)(((uint32_t)value >> 8) & 0xFFU);
  data[2] = (uint8_t)(((uint32_t)value >> 16) & 0xFFU);
  data[3] = (uint8_t)(((uint32_t)value >> 24) & 0xFFU);
}

static void motor_can_write_u16_le(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

HAL_StatusTypeDef motor_can_init(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_FilterTypeDef filter = {0};

  if (hfdcan == NULL) {
    return HAL_ERROR;
  }

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = MOTOR_CAN_CMD_ID;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ConfigInterruptLines(hfdcan,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     FDCAN_INTERRUPT_LINE0) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ActivateNotification(hfdcan,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0U) != HAL_OK) {
    return HAL_ERROR;
  }

  motor_can_hfdcan = hfdcan;
  return HAL_OK;
}

uint8_t motor_can_is_active(void) {
  return motor_can_active;
}

void motor_can_process(motor_encoder_drive_t *drive) {
  motor_can_command_t cmd;
  uint32_t now_ms;
  uint8_t tx_data[16] = {0U};
  FDCAN_TxHeaderTypeDef tx_header = {0};
  int32_t position_mdeg;
  int32_t speed_mhz = 0;

  if ((drive == NULL) || (motor_can_hfdcan == NULL)) {
    return;
  }

  if (motor_can_cmd_pending != 0U) {
    __disable_irq();
    cmd = motor_can_pending_cmd;
    motor_can_cmd_pending = 0U;
    __enable_irq();

    if (cmd.enable == 0U) {
      (void)motor_encoder_drive_stop(drive);
      motor_can_active = 0U;
      motor_can_mode = MOTOR_CAN_MODE_IDLE;
    } else if (cmd.mode == MOTOR_CAN_MODE_POSITION) {
      if (drive->started == 0U) {
        if (motor_encoder_drive_start(drive) != HAL_OK) {
          return;
        }
      }
      motor_encoder_drive_set_mode(drive, MOTOR_ENCODER_DRIVE_MODE_POSITION);
      if (motor_encoder_drive_set_position_target_mdeg(drive, cmd.value) ==
          HAL_OK) {
        motor_can_active = 1U;
        motor_can_mode = MOTOR_CAN_MODE_POSITION;
        motor_can_target_value = cmd.value;
      }
    } else if (cmd.mode == MOTOR_CAN_MODE_SPEED) {
      if (drive->started == 0U) {
        if (motor_encoder_drive_start(drive) != HAL_OK) {
          return;
        }
      }
      motor_encoder_drive_set_mode(drive, MOTOR_ENCODER_DRIVE_MODE_SPEED);
      if (motor_encoder_drive_set_mechanical_speed_target_mhz(drive,
                                                              cmd.value) ==
          HAL_OK) {
        motor_can_active = 1U;
        motor_can_mode = MOTOR_CAN_MODE_SPEED;
        motor_can_target_value = cmd.value;
      }
    }
  }

  now_ms = HAL_GetTick();
  if ((now_ms - motor_can_last_status_ms) < MOTOR_CAN_STATUS_PERIOD_MS) {
    return;
  }
  motor_can_last_status_ms = now_ms;

  position_mdeg = motor_encoder_drive_get_position_mdeg(drive);
  if ((drive->align != NULL) && (drive->align->pole_pairs != 0U)) {
    speed_mhz = (int32_t)((motor_encoder_drive_get_measured_speed_hz(drive) *
                           1000.0f) /
                          (float)drive->align->pole_pairs);
  }

  tx_data[0] = motor_can_mode;
  tx_data[1] = motor_can_active;
  motor_can_write_u16_le(&tx_data[2], (uint16_t)(motor_can_rx_count & 0xFFFFU));
  motor_can_write_i32_le(&tx_data[4], position_mdeg);
  motor_can_write_i32_le(&tx_data[8], motor_can_target_value);
  motor_can_write_i32_le(&tx_data[12], speed_mhz);

  tx_header.Identifier = MOTOR_CAN_STATUS_ID;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_16;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_ON;
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = (uint32_t)(motor_can_tx_count & 0xFFU);

  if (HAL_FDCAN_GetTxFifoFreeLevel(motor_can_hfdcan) > 0U) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(motor_can_hfdcan,
                                      &tx_header,
                                      tx_data) == HAL_OK) {
      motor_can_tx_count++;
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rx_header = {0};
  uint8_t rx_data[64] = {0U};
  motor_can_command_t cmd;

  if ((hfdcan != motor_can_hfdcan) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) !=
        HAL_OK) {
      return;
    }
    if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
        (rx_header.Identifier != MOTOR_CAN_CMD_ID) ||
        (rx_header.RxFrameType != FDCAN_DATA_FRAME) ||
        (rx_header.DataLength < FDCAN_DLC_BYTES_8)) {
      continue;
    }

    cmd.mode = rx_data[0];
    cmd.enable = rx_data[1] & 0x01U;
    cmd.value = motor_can_read_i32_le(&rx_data[4]);
    cmd.seq = motor_can_rx_count + 1U;
    motor_can_pending_cmd = cmd;
    motor_can_cmd_pending = 1U;
    motor_can_rx_count++;
  }
}
