#include "motor_encoder_drive.h"

#include "as5047p.h"
#include "motor_foc.h"
#include "svpwm.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float motor_encoder_drive_raw_to_rad(uint16_t raw) {
    return ((float)(raw & 0x3FFFU) * (2.0f * (float)M_PI)) / 16384.0f;
}

HAL_StatusTypeDef motor_encoder_drive_init(
    motor_encoder_drive_t *drive,
    const motor_encoder_drive_config_t *config) {
    if ((drive == NULL) || (config == NULL) || (config->align == NULL) ||
        (config->align->encoder == NULL) || (config->align->motor_pwm == NULL) ||
        (config->bus_voltage <= 0.0f)) {
        return HAL_ERROR;
    }

    drive->align = config->align;
    drive->voltage_q = config->voltage_q;
    drive->electrical_speed_rad_s =
        2.0f * (float)M_PI * config->electrical_speed_hz;
    drive->bus_voltage = config->bus_voltage;
    drive->modulation_limit = config->modulation_limit;
    drive->sensor_direction =
        (config->sensor_direction < 0) ? (int8_t)-1 : (int8_t)1;
    drive->electrical_angle_rad = 0.0f;
    drive->electrical_angle_raw = 0U;
    drive->last_tick_ms = HAL_GetTick();
    drive->started = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef motor_encoder_drive_start(motor_encoder_drive_t *drive) {
    if ((drive == NULL) || (drive->align == NULL)) {
        return HAL_ERROR;
    }

    if (motor_pwm_start(drive->align->motor_pwm) != HAL_OK) {
        drive->started = 0U;
        return HAL_ERROR;
    }

    drive->started = 1U;
    drive->electrical_angle_rad = 0.0f;
    drive->electrical_angle_raw = 0U;
    drive->last_tick_ms = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef motor_encoder_drive_stop(motor_encoder_drive_t *drive) {
    if ((drive == NULL) || (drive->align == NULL)) {
        return HAL_ERROR;
    }

    drive->started = 0U;
    return motor_pwm_stop(drive->align->motor_pwm);
}

HAL_StatusTypeDef motor_encoder_drive_update(motor_encoder_drive_t *drive) {
    uint16_t mech_raw = 0U;
    uint16_t electrical_raw;
    float theta;
    motor_foc_dq_t voltage_dq = {0.0f, 0.0f};
    motor_foc_alphabeta_t voltage_alphabeta;
    svpwm_duty_t duty = {0.5f, 0.5f, 0.5f};

    if ((drive == NULL) || (drive->align == NULL) ||
        (drive->align->encoder == NULL) || (drive->align->motor_pwm == NULL) ||
        (drive->started == 0U)) {
        return HAL_ERROR;
    }

    if (as5047p_get_position(drive->align->encoder, with_daec, &mech_raw) != 0) {
        return HAL_ERROR;
    }

    electrical_raw = motor_encoder_align_get_elec_raw(drive->align, mech_raw);
    if (drive->sensor_direction < 0) {
        electrical_raw = (uint16_t)((0x4000U - electrical_raw) & 0x3FFFU);
    }

    drive->electrical_angle_raw = electrical_raw;
    drive->electrical_angle_rad = motor_encoder_drive_raw_to_rad(electrical_raw);
    drive->last_tick_ms = HAL_GetTick();

    theta = drive->electrical_angle_rad;
    voltage_dq.q = drive->voltage_q;
    voltage_alphabeta = motor_foc_inverse_park(voltage_dq, theta);

    svpwm_generate(voltage_alphabeta.alpha, voltage_alphabeta.beta,
                   drive->bus_voltage,
                   drive->modulation_limit, &duty);
    motor_pwm_set_duty(drive->align->motor_pwm, duty.duty_a, duty.duty_b,
                       duty.duty_c);

    return HAL_OK;
}

uint32_t motor_encoder_drive_get_electrical_mdeg(
    const motor_encoder_drive_t *drive) {
    if (drive == NULL) {
        return 0U;
    }

    return motor_encoder_align_raw_to_mdeg(drive->electrical_angle_raw);
}
