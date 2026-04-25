#include "motor_pwm.h"

static float motor_pwm_clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t motor_pwm_duty_to_compare(const motor_pwm_t *motor_pwm,
                                          float duty) {
    const float clamped_duty = motor_pwm_clampf(duty, 0.0f, 1.0f);
    const float scaled = clamped_duty * (float)motor_pwm->period_ticks;
    uint32_t compare = (uint32_t)(scaled + 0.5f);

    if (compare > motor_pwm->period_ticks) {
        compare = motor_pwm->period_ticks;
    }

    return compare;
}

HAL_StatusTypeDef motor_pwm_init(motor_pwm_t *motor_pwm,
                                 const motor_pwm_config_t *config) {
    if ((motor_pwm == NULL) || (config == NULL) || (config->htim == NULL)) {
        return HAL_ERROR;
    }

    motor_pwm->htim = config->htim;
    motor_pwm->enable_port = config->enable_port;
    motor_pwm->enable_pin = config->enable_pin;
    motor_pwm->bus_voltage = config->bus_voltage;
    motor_pwm->period_ticks = __HAL_TIM_GET_AUTORELOAD(config->htim);
    motor_pwm->started = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef motor_pwm_start(motor_pwm_t *motor_pwm) {
    HAL_StatusTypeDef status = HAL_OK;

    if ((motor_pwm == NULL) || (motor_pwm->htim == NULL)) {
        return HAL_ERROR;
    }

    motor_pwm_set_duty(motor_pwm, 0.5f, 0.5f, 0.5f);

    status = HAL_TIM_PWM_Start(motor_pwm->htim, TIM_CHANNEL_1);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_TIM_PWM_Start(motor_pwm->htim, TIM_CHANNEL_2);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_TIM_PWM_Start(motor_pwm->htim, TIM_CHANNEL_3);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_TIMEx_PWMN_Start(motor_pwm->htim, TIM_CHANNEL_1);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_TIMEx_PWMN_Start(motor_pwm->htim, TIM_CHANNEL_2);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_TIMEx_PWMN_Start(motor_pwm->htim, TIM_CHANNEL_3);
    if (status != HAL_OK) {
        return status;
    }

    if (motor_pwm->enable_port != NULL) {
        HAL_GPIO_WritePin(motor_pwm->enable_port, motor_pwm->enable_pin,
                          GPIO_PIN_SET);
    }

    motor_pwm->started = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef motor_pwm_stop(motor_pwm_t *motor_pwm) {
    if ((motor_pwm == NULL) || (motor_pwm->htim == NULL)) {
        return HAL_ERROR;
    }

    if (motor_pwm->enable_port != NULL) {
        HAL_GPIO_WritePin(motor_pwm->enable_port, motor_pwm->enable_pin,
                          GPIO_PIN_RESET);
    }

    (void)HAL_TIMEx_PWMN_Stop(motor_pwm->htim, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(motor_pwm->htim, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(motor_pwm->htim, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(motor_pwm->htim, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(motor_pwm->htim, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(motor_pwm->htim, TIM_CHANNEL_3);

    motor_pwm_set_duty(motor_pwm, 0.5f, 0.5f, 0.5f);
    motor_pwm->started = 0U;

    return HAL_OK;
}

void motor_pwm_set_duty(motor_pwm_t *motor_pwm, float duty_a, float duty_b,
                        float duty_c) {
    if ((motor_pwm == NULL) || (motor_pwm->htim == NULL)) {
        return;
    }

    __HAL_TIM_SET_COMPARE(motor_pwm->htim, TIM_CHANNEL_1,
                          motor_pwm_duty_to_compare(motor_pwm, duty_a));
    __HAL_TIM_SET_COMPARE(motor_pwm->htim, TIM_CHANNEL_2,
                          motor_pwm_duty_to_compare(motor_pwm, duty_b));
    __HAL_TIM_SET_COMPARE(motor_pwm->htim, TIM_CHANNEL_3,
                          motor_pwm_duty_to_compare(motor_pwm, duty_c));
}
