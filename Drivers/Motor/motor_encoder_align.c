#include "motor_encoder_align.h"

#include "svpwm.h"

uint32_t motor_encoder_align_raw_to_mdeg(uint16_t raw) {
    return (uint32_t)((((uint64_t)(raw & 0x3FFFU)) * 360000ULL + 8192ULL) /
                      16384ULL);
}

HAL_StatusTypeDef motor_encoder_align_init(
    motor_encoder_align_t *align,
    const motor_encoder_align_config_t *config) {
    if ((align == NULL) || (config == NULL) || (config->encoder == NULL) ||
        (config->motor_pwm == NULL) || (config->pole_pairs == 0U) ||
        (config->bus_voltage <= 0.0f) || (config->sample_count == 0U)) {
        return HAL_ERROR;
    }

    align->encoder = config->encoder;
    align->motor_pwm = config->motor_pwm;
    align->pole_pairs = config->pole_pairs;
    align->bus_voltage = config->bus_voltage;
    align->align_voltage = config->align_voltage;
    align->modulation_limit = config->modulation_limit;
    align->align_time_ms = config->align_time_ms;
    align->sample_count = config->sample_count;
    align->electrical_offset_raw = 0U;

    return HAL_OK;
}

uint16_t motor_encoder_align_mech_to_elec_raw(
    const motor_encoder_align_t *align,
    uint16_t mech_raw) {
    if (align == NULL) {
        return 0U;
    }

    return (uint16_t)(((uint32_t)(mech_raw & 0x3FFFU) *
                       (uint32_t)align->pole_pairs) &
                      0x3FFFU);
}

uint16_t motor_encoder_align_get_elec_raw(
    const motor_encoder_align_t *align,
    uint16_t mech_raw) {
    if (align == NULL) {
        return 0U;
    }

    return (uint16_t)((motor_encoder_align_mech_to_elec_raw(align, mech_raw) -
                       align->electrical_offset_raw) &
                      0x3FFFU);
}

uint16_t motor_encoder_align_get_offset_raw(
    const motor_encoder_align_t *align) {
    if (align == NULL) {
        return 0U;
    }

    return align->electrical_offset_raw;
}

static HAL_StatusTypeDef motor_encoder_align_apply_vector(
    motor_encoder_align_t *align) {
    svpwm_duty_t duty = {0.5f, 0.5f, 0.5f};

    if (motor_pwm_start(align->motor_pwm) != HAL_OK) {
        return HAL_ERROR;
    }

    svpwm_generate(align->align_voltage, 0.0f, align->bus_voltage,
                   align->modulation_limit, &duty);
    motor_pwm_set_duty(align->motor_pwm, duty.duty_a, duty.duty_b,
                       duty.duty_c);

    return HAL_OK;
}

static HAL_StatusTypeDef motor_encoder_align_read_average_raw(
    motor_encoder_align_t *align,
    uint16_t *raw) {
    uint16_t first_raw = 0U;
    int32_t sum_raw;

    if ((align == NULL) || (raw == NULL)) {
        return HAL_ERROR;
    }

    if (as5047p_get_position(align->encoder, with_daec, &first_raw) != 0) {
        return HAL_ERROR;
    }

    sum_raw = (int32_t)(first_raw & 0x3FFFU);

    for (uint32_t i = 1U; i < align->sample_count; i++) {
        uint16_t sample_raw = 0U;
        int32_t delta;

        HAL_Delay(2U);
        if (as5047p_get_position(align->encoder, with_daec, &sample_raw) != 0) {
            return HAL_ERROR;
        }

        delta = (int32_t)(sample_raw & 0x3FFFU) -
                (int32_t)(first_raw & 0x3FFFU);
        if (delta > 8191) {
            delta -= 16384;
        } else if (delta < -8192) {
            delta += 16384;
        }

        sum_raw += (int32_t)(first_raw & 0x3FFFU) + delta;
    }

    {
        int32_t average_raw = sum_raw / (int32_t)align->sample_count;

        while (average_raw < 0) {
            average_raw += 16384;
        }
        while (average_raw >= 16384) {
            average_raw -= 16384;
        }

        *raw = (uint16_t)average_raw;
    }

    return HAL_OK;
}

HAL_StatusTypeDef motor_encoder_align_run(motor_encoder_align_t *align,
                                          uint16_t *aligned_mech_raw) {
    uint16_t mech_raw = 0U;

    if (align == NULL) {
        return HAL_ERROR;
    }

    if (motor_encoder_align_apply_vector(align) != HAL_OK) {
        (void)motor_pwm_stop(align->motor_pwm);
        return HAL_ERROR;
    }

    HAL_Delay(align->align_time_ms);

    if (motor_encoder_align_read_average_raw(align, &mech_raw) != HAL_OK) {
        (void)motor_pwm_stop(align->motor_pwm);
        return HAL_ERROR;
    }

    align->electrical_offset_raw =
        motor_encoder_align_mech_to_elec_raw(align, mech_raw);
    (void)motor_pwm_stop(align->motor_pwm);

    if (aligned_mech_raw != NULL) {
        *aligned_mech_raw = mech_raw;
    }

    return HAL_OK;
}

HAL_StatusTypeDef motor_encoder_align_set_offset_raw(
    motor_encoder_align_t *align,
    uint16_t electrical_offset_raw) {
    if (align == NULL) {
        return HAL_ERROR;
    }

    align->electrical_offset_raw = (uint16_t)(electrical_offset_raw & 0x3FFFU);
    return HAL_OK;
}
