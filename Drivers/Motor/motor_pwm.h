#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    float bus_voltage;
} motor_pwm_config_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    float bus_voltage;
    uint32_t period_ticks;
    uint8_t started;
} motor_pwm_t;

HAL_StatusTypeDef motor_pwm_init(motor_pwm_t *motor_pwm,
                                 const motor_pwm_config_t *config);
HAL_StatusTypeDef motor_pwm_start(motor_pwm_t *motor_pwm);
HAL_StatusTypeDef motor_pwm_stop(motor_pwm_t *motor_pwm);
void motor_pwm_set_duty(motor_pwm_t *motor_pwm, float duty_a, float duty_b,
                        float duty_c);

#ifdef __cplusplus
}
#endif

#endif
