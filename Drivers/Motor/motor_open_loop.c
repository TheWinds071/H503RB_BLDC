#include "motor_open_loop.h"

#include "svpwm.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float motor_open_loop_wrap_angle(float angle_rad) {
    const float two_pi = 2.0f * (float)M_PI;

    while (angle_rad >= two_pi) {
        angle_rad -= two_pi;
    }

    while (angle_rad < 0.0f) {
        angle_rad += two_pi;
    }

    return angle_rad;
}

HAL_StatusTypeDef motor_open_loop_init(
    motor_open_loop_t *open_loop,
    const motor_open_loop_config_t *config) {
    if ((open_loop == NULL) || (config == NULL) || (config->motor_pwm == NULL)) {
        return HAL_ERROR;
    }

    open_loop->motor_pwm = config->motor_pwm;
    open_loop->bus_voltage = config->bus_voltage;
    open_loop->align_voltage = config->align_voltage;
    open_loop->align_time_ms = config->align_time_ms;
    open_loop->voltage_q = config->voltage_q;
    open_loop->startup_speed_rad_s =
        2.0f * (float)M_PI * config->startup_speed_hz;
    open_loop->ramp_time_ms = config->ramp_time_ms;
    open_loop->target_speed_rad_s =
        2.0f * (float)M_PI * config->electrical_speed_hz;
    open_loop->electrical_speed_rad_s = open_loop->startup_speed_rad_s;
    open_loop->electrical_angle_rad = 0.0f;
    open_loop->modulation_limit = config->modulation_limit;
    open_loop->last_tick_ms = HAL_GetTick();
    open_loop->phase_tick_ms = open_loop->last_tick_ms;
    open_loop->state = MOTOR_OPEN_LOOP_STATE_ALIGN;

    return HAL_OK;
}

HAL_StatusTypeDef motor_open_loop_start(motor_open_loop_t *open_loop) {
    if (open_loop == NULL) {
        return HAL_ERROR;
    }

    open_loop->last_tick_ms = HAL_GetTick();
    open_loop->phase_tick_ms = open_loop->last_tick_ms;
    open_loop->electrical_angle_rad = 0.0f;
    open_loop->electrical_speed_rad_s = open_loop->startup_speed_rad_s;
    open_loop->state = MOTOR_OPEN_LOOP_STATE_ALIGN;
    return motor_pwm_start(open_loop->motor_pwm);
}

HAL_StatusTypeDef motor_open_loop_stop(motor_open_loop_t *open_loop) {
    if (open_loop == NULL) {
        return HAL_ERROR;
    }

    return motor_pwm_stop(open_loop->motor_pwm);
}

void motor_open_loop_update(motor_open_loop_t *open_loop) {
    const float half_pi = 0.5f * (float)M_PI;
    const uint32_t now_ms = HAL_GetTick();
    uint32_t delta_ms;
    float dt_s;
    float v_alpha;
    float v_beta;
    float command_voltage;
    uint32_t phase_elapsed_ms;
    svpwm_duty_t duty = {0.5f, 0.5f, 0.5f};

    if ((open_loop == NULL) || (open_loop->motor_pwm == NULL)) {
        return;
    }

    delta_ms = now_ms - open_loop->last_tick_ms;
    if (delta_ms == 0U) {
        return;
    }

    open_loop->last_tick_ms = now_ms;
    dt_s = (float)delta_ms * 0.001f;
    phase_elapsed_ms = now_ms - open_loop->phase_tick_ms;

    switch (open_loop->state) {
    case MOTOR_OPEN_LOOP_STATE_ALIGN:
        command_voltage = open_loop->align_voltage;
        open_loop->electrical_angle_rad = 0.0f;
        open_loop->electrical_speed_rad_s = 0.0f;
        if (phase_elapsed_ms >= open_loop->align_time_ms) {
            open_loop->state = MOTOR_OPEN_LOOP_STATE_RAMP;
            open_loop->phase_tick_ms = now_ms;
        }
        break;
    case MOTOR_OPEN_LOOP_STATE_RAMP:
        command_voltage = open_loop->voltage_q;
        if (open_loop->ramp_time_ms == 0U) {
            open_loop->electrical_speed_rad_s = open_loop->target_speed_rad_s;
            open_loop->state = MOTOR_OPEN_LOOP_STATE_RUN;
        } else {
            const float ratio =
                (float)phase_elapsed_ms / (float)open_loop->ramp_time_ms;
            const float clamped_ratio = (ratio > 1.0f) ? 1.0f : ratio;
            open_loop->electrical_speed_rad_s =
                open_loop->startup_speed_rad_s +
                ((open_loop->target_speed_rad_s - open_loop->startup_speed_rad_s) *
                 clamped_ratio);
            if (phase_elapsed_ms >= open_loop->ramp_time_ms) {
                open_loop->state = MOTOR_OPEN_LOOP_STATE_RUN;
            }
        }
        open_loop->electrical_angle_rad =
            motor_open_loop_wrap_angle(open_loop->electrical_angle_rad +
                                       (open_loop->electrical_speed_rad_s * dt_s));
        break;
    case MOTOR_OPEN_LOOP_STATE_RUN:
    default:
        command_voltage = open_loop->voltage_q;
        open_loop->electrical_speed_rad_s = open_loop->target_speed_rad_s;
        open_loop->electrical_angle_rad =
            motor_open_loop_wrap_angle(open_loop->electrical_angle_rad +
                                       (open_loop->electrical_speed_rad_s * dt_s));
        break;
    }

    v_alpha = command_voltage * cosf(open_loop->electrical_angle_rad + half_pi);
    v_beta = command_voltage * sinf(open_loop->electrical_angle_rad + half_pi);

    svpwm_generate(v_alpha, v_beta, open_loop->bus_voltage,
                   open_loop->modulation_limit, &duty);
    motor_pwm_set_duty(open_loop->motor_pwm, duty.duty_a, duty.duty_b,
                       duty.duty_c);
}

float motor_open_loop_get_electrical_angle_deg(
    const motor_open_loop_t *open_loop) {
    if (open_loop == NULL) {
        return 0.0f;
    }

    return open_loop->electrical_angle_rad * (180.0f / (float)M_PI);
}
