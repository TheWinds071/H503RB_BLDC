#ifndef MOTOR_OPEN_LOOP_H
#define MOTOR_OPEN_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_pwm.h"

typedef struct {
    motor_pwm_t *motor_pwm;
    float bus_voltage;
    float align_voltage;
    uint32_t align_time_ms;
    float voltage_q;
    float startup_speed_hz;
    uint32_t ramp_time_ms;
    float electrical_speed_hz;
    float modulation_limit;
} motor_open_loop_config_t;

typedef enum {
    MOTOR_OPEN_LOOP_STATE_ALIGN = 0,
    MOTOR_OPEN_LOOP_STATE_RAMP,
    MOTOR_OPEN_LOOP_STATE_RUN
} motor_open_loop_state_t;

typedef struct {
    motor_pwm_t *motor_pwm;
    float bus_voltage;
    float align_voltage;
    uint32_t align_time_ms;
    float voltage_q;
    float startup_speed_rad_s;
    uint32_t ramp_time_ms;
    float target_speed_rad_s;
    float electrical_speed_rad_s;
    float electrical_angle_rad;
    float modulation_limit;
    uint32_t last_tick_ms;
    uint32_t phase_tick_ms;
    motor_open_loop_state_t state;
} motor_open_loop_t;

HAL_StatusTypeDef motor_open_loop_init(
    motor_open_loop_t *open_loop,
    const motor_open_loop_config_t *config);
HAL_StatusTypeDef motor_open_loop_start(motor_open_loop_t *open_loop);
HAL_StatusTypeDef motor_open_loop_stop(motor_open_loop_t *open_loop);
void motor_open_loop_update(motor_open_loop_t *open_loop);
float motor_open_loop_get_electrical_angle_deg(
    const motor_open_loop_t *open_loop);

#ifdef __cplusplus
}
#endif

#endif
