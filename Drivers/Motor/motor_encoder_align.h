#ifndef MOTOR_ENCODER_ALIGN_H
#define MOTOR_ENCODER_ALIGN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "as5047p.h"
#include "motor_pwm.h"
#include "stm32h5xx_hal.h"

typedef struct {
    as5047p_handle_t *encoder;
    motor_pwm_t *motor_pwm;
    uint8_t pole_pairs;
    float bus_voltage;
    float align_voltage;
    float modulation_limit;
    uint32_t align_time_ms;
    uint32_t sample_count;
} motor_encoder_align_config_t;

typedef struct {
    as5047p_handle_t *encoder;
    motor_pwm_t *motor_pwm;
    uint8_t pole_pairs;
    float bus_voltage;
    float align_voltage;
    float modulation_limit;
    uint32_t align_time_ms;
    uint32_t sample_count;
    uint16_t electrical_offset_raw;
} motor_encoder_align_t;

HAL_StatusTypeDef motor_encoder_align_init(
    motor_encoder_align_t *align,
    const motor_encoder_align_config_t *config);
HAL_StatusTypeDef motor_encoder_align_run(motor_encoder_align_t *align,
                                          uint16_t *aligned_mech_raw);
uint16_t motor_encoder_align_mech_to_elec_raw(
    const motor_encoder_align_t *align,
    uint16_t mech_raw);
uint16_t motor_encoder_align_get_elec_raw(
    const motor_encoder_align_t *align,
    uint16_t mech_raw);
uint16_t motor_encoder_align_get_offset_raw(
    const motor_encoder_align_t *align);
uint32_t motor_encoder_align_raw_to_mdeg(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif
