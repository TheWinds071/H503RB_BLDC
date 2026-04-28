#ifndef MOTOR_ENCODER_DRIVE_H
#define MOTOR_ENCODER_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_encoder_align.h"

typedef struct {
    motor_encoder_align_t *align;
    float voltage_q;
    float electrical_speed_hz;
    float bus_voltage;
    float modulation_limit;
    int8_t sensor_direction;
} motor_encoder_drive_config_t;

typedef struct {
    motor_encoder_align_t *align;
    float voltage_q;
    float electrical_speed_rad_s;
    float bus_voltage;
    float modulation_limit;
    int8_t sensor_direction;
    float electrical_angle_rad;
    uint16_t electrical_angle_raw;
    uint32_t last_tick_ms;
    uint8_t started;
} motor_encoder_drive_t;

HAL_StatusTypeDef motor_encoder_drive_init(
    motor_encoder_drive_t *drive,
    const motor_encoder_drive_config_t *config);
HAL_StatusTypeDef motor_encoder_drive_start(motor_encoder_drive_t *drive);
HAL_StatusTypeDef motor_encoder_drive_stop(motor_encoder_drive_t *drive);
HAL_StatusTypeDef motor_encoder_drive_update(motor_encoder_drive_t *drive);
uint32_t motor_encoder_drive_get_electrical_mdeg(
    const motor_encoder_drive_t *drive);

#ifdef __cplusplus
}
#endif

#endif
