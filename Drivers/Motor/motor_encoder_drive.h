#ifndef MOTOR_ENCODER_DRIVE_H
#define MOTOR_ENCODER_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_encoder_align.h"
#include "motor_foc.h"

typedef struct {
    motor_encoder_align_t *align;
    float voltage_q;
    float electrical_speed_hz;
    float bus_voltage;
    float modulation_limit;
    int8_t sensor_direction;
} motor_encoder_drive_config_t;

typedef enum {
    MOTOR_ENCODER_DRIVE_MODE_SPEED = 0,
    MOTOR_ENCODER_DRIVE_MODE_POSITION = 1,
} motor_encoder_drive_mode_t;

typedef struct {
    motor_encoder_align_t *align;
    float voltage_q;
    float electrical_speed_rad_s;
    float bus_voltage;
    float modulation_limit;
    int8_t sensor_direction;
    float electrical_angle_rad;
    uint16_t electrical_angle_raw;
    uint16_t last_electrical_angle_raw;
    uint16_t mechanical_angle_raw;
    uint16_t last_mechanical_angle_raw;
    int32_t mechanical_position_counts;
    int32_t mechanical_zero_counts;
    int32_t position_target_counts;
    float measured_speed_rad_s;
    int32_t speed_delta_accum_raw;
    float speed_dt_accum_s;
    float speed_iq_integrator;
    float voltage_d_integrator;
    float voltage_q_integrator;
    motor_foc_dq_t current_dq;
    motor_foc_dq_t measured_current_dq;
    float current_q_ref;
    float voltage_d_applied;
    float voltage_q_applied;
    uint32_t start_tick_ms;
    uint32_t last_tick_ms;
    uint8_t current_observer_ready;
    uint8_t speed_ready;
    uint8_t position_ready;
    motor_encoder_drive_mode_t mode;
    uint8_t started;
} motor_encoder_drive_t;

HAL_StatusTypeDef motor_encoder_drive_init(
    motor_encoder_drive_t *drive,
    const motor_encoder_drive_config_t *config);
HAL_StatusTypeDef motor_encoder_drive_start(motor_encoder_drive_t *drive);
HAL_StatusTypeDef motor_encoder_drive_stop(motor_encoder_drive_t *drive);
HAL_StatusTypeDef motor_encoder_drive_update(motor_encoder_drive_t *drive);
HAL_StatusTypeDef motor_encoder_drive_update_with_current(
    motor_encoder_drive_t *drive,
    const motor_foc_abc_t *phase_current_abc);
HAL_StatusTypeDef motor_encoder_drive_update_with_current_dt(
    motor_encoder_drive_t *drive,
    const motor_foc_abc_t *phase_current_abc,
    float dt_s);
void motor_encoder_drive_set_mode(motor_encoder_drive_t *drive,
                                  motor_encoder_drive_mode_t mode);
HAL_StatusTypeDef motor_encoder_drive_set_position_target_mdeg(
    motor_encoder_drive_t *drive,
    int32_t target_mdeg);
HAL_StatusTypeDef motor_encoder_drive_set_mechanical_speed_target_mhz(
    motor_encoder_drive_t *drive,
    int32_t target_mhz);
uint32_t motor_encoder_drive_get_electrical_mdeg(
    const motor_encoder_drive_t *drive);
int32_t motor_encoder_drive_get_position_mdeg(
    const motor_encoder_drive_t *drive);
int32_t motor_encoder_drive_get_position_target_mdeg(
    const motor_encoder_drive_t *drive);
float motor_encoder_drive_get_measured_speed_hz(
    const motor_encoder_drive_t *drive);
float motor_encoder_drive_get_voltage_q(const motor_encoder_drive_t *drive);
float motor_encoder_drive_get_current_q_ref(const motor_encoder_drive_t *drive);
motor_foc_dq_t motor_encoder_drive_get_current_dq(
    const motor_encoder_drive_t *drive);

#ifdef __cplusplus
}
#endif

#endif
