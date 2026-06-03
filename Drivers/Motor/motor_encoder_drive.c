#include "motor_encoder_drive.h"

#include "as5047p.h"
#include "svpwm.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MOTOR_ENCODER_DRIVE_VOLTAGE_RAMP_MS 1000U
#define MOTOR_ENCODER_DRIVE_SPEED_KP 0.020f
#define MOTOR_ENCODER_DRIVE_SPEED_KI 0.500f
#define MOTOR_ENCODER_DRIVE_SPEED_FILTER_ALPHA 0.100f
#define MOTOR_ENCODER_DRIVE_SPEED_ESTIMATE_PERIOD_S 0.005f
#define MOTOR_ENCODER_DRIVE_CURRENT_Q_LIMIT_A 1.000f
#define MOTOR_ENCODER_DRIVE_CURRENT_D_KP 0.800f
#define MOTOR_ENCODER_DRIVE_CURRENT_D_KI 8.000f
#define MOTOR_ENCODER_DRIVE_CURRENT_Q_KP 0.800f
#define MOTOR_ENCODER_DRIVE_CURRENT_Q_KI 8.000f
#define MOTOR_ENCODER_DRIVE_CURRENT_TRACKING_ERR_A 0.250f
#define MOTOR_ENCODER_DRIVE_CURRENT_OBSERVER_ALPHA 0.250f
#define MOTOR_ENCODER_DRIVE_POSITION_KP_A_PER_RAD 1.500f
#define MOTOR_ENCODER_DRIVE_POSITION_KD_A_PER_RAD_S 0.030f
#define MOTOR_ENCODER_DRIVE_POSITION_CURRENT_LIMIT_A 0.800f
#define MOTOR_ENCODER_DRIVE_POSITION_MIN_MOVE_CURRENT_A 0.060f
#define MOTOR_ENCODER_DRIVE_POSITION_MIN_MOVE_ERR_COUNTS 23

static float motor_encoder_drive_raw_to_rad(uint16_t raw) {
    return ((float)(raw & 0x3FFFU) * (2.0f * (float)M_PI)) / 16384.0f;
}

static float motor_encoder_drive_clampf(float value, float min_value,
                                        float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float motor_encoder_drive_ramp_voltage(float target_voltage,
                                              uint32_t elapsed_ms) {
    if (elapsed_ms >= MOTOR_ENCODER_DRIVE_VOLTAGE_RAMP_MS) {
        return target_voltage;
    }

    return target_voltage *
           ((float)elapsed_ms / (float)MOTOR_ENCODER_DRIVE_VOLTAGE_RAMP_MS);
}

static float motor_encoder_drive_ramp_ratio(uint32_t elapsed_ms) {
    if (elapsed_ms >= MOTOR_ENCODER_DRIVE_VOLTAGE_RAMP_MS) {
        return 1.0f;
    }

    return (float)elapsed_ms / (float)MOTOR_ENCODER_DRIVE_VOLTAGE_RAMP_MS;
}

static int32_t motor_encoder_drive_mdeg_to_counts(int32_t mdeg) {
    int64_t counts = ((int64_t)mdeg * 16384LL) / 360000LL;

    if (counts > INT32_MAX) {
        return INT32_MAX;
    }
    if (counts < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)counts;
}

static int32_t motor_encoder_drive_counts_to_mdeg(int32_t counts) {
    return (int32_t)(((int64_t)counts * 360000LL) / 16384LL);
}

static void motor_encoder_drive_update_mechanical_position(
    motor_encoder_drive_t *drive,
    uint16_t mech_raw) {
    int32_t delta_raw;

    drive->mechanical_angle_raw = (uint16_t)(mech_raw & 0x3FFFU);
    if (drive->position_ready == 0U) {
        drive->last_mechanical_angle_raw = drive->mechanical_angle_raw;
        drive->mechanical_position_counts =
            (int32_t)drive->mechanical_angle_raw;
        drive->mechanical_zero_counts = 0;
        drive->position_ready = 1U;
        return;
    }

    delta_raw = (int32_t)drive->mechanical_angle_raw -
                (int32_t)drive->last_mechanical_angle_raw;
    if (delta_raw > 8191) {
        delta_raw -= 16384;
    } else if (delta_raw < -8192) {
        delta_raw += 16384;
    }

    drive->mechanical_position_counts += delta_raw;
    drive->last_mechanical_angle_raw = drive->mechanical_angle_raw;
}

static float motor_encoder_drive_estimate_speed(motor_encoder_drive_t *drive,
                                                uint16_t electrical_raw,
                                                float dt_s) {
    int32_t delta_raw;
    float speed_rad_s;

    if (dt_s <= 0.0f) {
        return drive->measured_speed_rad_s;
    }

    delta_raw = (int32_t)(electrical_raw & 0x3FFFU) -
                (int32_t)(drive->last_electrical_angle_raw & 0x3FFFU);
    if (delta_raw > 8191) {
        delta_raw -= 16384;
    } else if (delta_raw < -8192) {
        delta_raw += 16384;
    }

    drive->speed_delta_accum_raw += delta_raw;
    drive->speed_dt_accum_s += dt_s;
    if (drive->speed_dt_accum_s < MOTOR_ENCODER_DRIVE_SPEED_ESTIMATE_PERIOD_S) {
        return drive->measured_speed_rad_s;
    }

    speed_rad_s =
        ((float)drive->speed_delta_accum_raw * (2.0f * (float)M_PI)) /
        (16384.0f * drive->speed_dt_accum_s);
    drive->speed_delta_accum_raw = 0;
    drive->speed_dt_accum_s = 0.0f;

    return drive->measured_speed_rad_s +
           ((speed_rad_s - drive->measured_speed_rad_s) *
            MOTOR_ENCODER_DRIVE_SPEED_FILTER_ALPHA);
}

static float motor_encoder_drive_speed_to_iq_ref(motor_encoder_drive_t *drive,
                                                 float speed_error,
                                                 float dt_s,
                                                 uint8_t allow_integrator) {
    float current_q_ref;

    if (allow_integrator != 0U) {
        drive->speed_iq_integrator +=
            speed_error * MOTOR_ENCODER_DRIVE_SPEED_KI * dt_s;
        drive->speed_iq_integrator =
            motor_encoder_drive_clampf(drive->speed_iq_integrator,
                                       -MOTOR_ENCODER_DRIVE_CURRENT_Q_LIMIT_A,
                                       MOTOR_ENCODER_DRIVE_CURRENT_Q_LIMIT_A);
    }

    current_q_ref =
        (speed_error * MOTOR_ENCODER_DRIVE_SPEED_KP) +
        drive->speed_iq_integrator;

    return motor_encoder_drive_clampf(current_q_ref,
                                      -MOTOR_ENCODER_DRIVE_CURRENT_Q_LIMIT_A,
                                      MOTOR_ENCODER_DRIVE_CURRENT_Q_LIMIT_A);
}

static float motor_encoder_drive_position_to_iq_ref(
    motor_encoder_drive_t *drive) {
    const int32_t relative_counts =
        drive->mechanical_position_counts - drive->mechanical_zero_counts;
    const int32_t error_counts = drive->position_target_counts - relative_counts;
    int32_t abs_error_counts = error_counts;
    const float position_error_rad =
        ((float)error_counts * (2.0f * (float)M_PI)) / 16384.0f;
    const float mechanical_speed_rad_s =
        drive->measured_speed_rad_s / (float)drive->align->pole_pairs;
    float current_q_ref =
        (position_error_rad * MOTOR_ENCODER_DRIVE_POSITION_KP_A_PER_RAD) -
        (mechanical_speed_rad_s * MOTOR_ENCODER_DRIVE_POSITION_KD_A_PER_RAD_S);

    if (abs_error_counts < 0) {
        abs_error_counts = -abs_error_counts;
    }
    if ((abs_error_counts > MOTOR_ENCODER_DRIVE_POSITION_MIN_MOVE_ERR_COUNTS) &&
        (current_q_ref > -MOTOR_ENCODER_DRIVE_POSITION_MIN_MOVE_CURRENT_A) &&
        (current_q_ref < MOTOR_ENCODER_DRIVE_POSITION_MIN_MOVE_CURRENT_A)) {
        current_q_ref =
            (error_counts > 0) ? MOTOR_ENCODER_DRIVE_POSITION_MIN_MOVE_CURRENT_A
                               : -MOTOR_ENCODER_DRIVE_POSITION_MIN_MOVE_CURRENT_A;
    }

    return motor_encoder_drive_clampf(
        current_q_ref,
        -MOTOR_ENCODER_DRIVE_POSITION_CURRENT_LIMIT_A,
        MOTOR_ENCODER_DRIVE_POSITION_CURRENT_LIMIT_A);
}

static motor_foc_dq_t motor_encoder_drive_current_pi(
    motor_encoder_drive_t *drive,
    float current_q_ref,
    float voltage_limit,
    float dt_s) {
    const float current_d_error = 0.0f - drive->current_dq.d;
    const float current_q_error = current_q_ref - drive->current_dq.q;
    motor_foc_dq_t voltage_dq;

    voltage_dq.d =
        (current_d_error * MOTOR_ENCODER_DRIVE_CURRENT_D_KP) +
        drive->voltage_d_integrator;
    voltage_dq.q =
        (current_q_error * MOTOR_ENCODER_DRIVE_CURRENT_Q_KP) +
        drive->voltage_q_integrator;

    voltage_dq.d =
        motor_encoder_drive_clampf(voltage_dq.d, -voltage_limit, voltage_limit);
    voltage_dq.q =
        motor_encoder_drive_clampf(voltage_dq.q, -voltage_limit, voltage_limit);

    if ((voltage_dq.d > -voltage_limit) && (voltage_dq.d < voltage_limit)) {
        drive->voltage_d_integrator +=
            current_d_error * MOTOR_ENCODER_DRIVE_CURRENT_D_KI * dt_s;
        drive->voltage_d_integrator =
            motor_encoder_drive_clampf(drive->voltage_d_integrator,
                                       -voltage_limit, voltage_limit);
    }

    if ((voltage_dq.q > -voltage_limit) && (voltage_dq.q < voltage_limit)) {
        drive->voltage_q_integrator +=
            current_q_error * MOTOR_ENCODER_DRIVE_CURRENT_Q_KI * dt_s;
        drive->voltage_q_integrator =
            motor_encoder_drive_clampf(drive->voltage_q_integrator,
                                       -voltage_limit, voltage_limit);
    }

    return voltage_dq;
}

static void motor_encoder_drive_update_current_observer(
    motor_encoder_drive_t *drive,
    motor_foc_dq_t measured_current_dq) {
    drive->measured_current_dq = measured_current_dq;

    if (drive->current_observer_ready == 0U) {
        drive->current_dq = measured_current_dq;
        drive->current_observer_ready = 1U;
        return;
    }

    drive->current_dq.d +=
        (measured_current_dq.d - drive->current_dq.d) *
        MOTOR_ENCODER_DRIVE_CURRENT_OBSERVER_ALPHA;
    drive->current_dq.q +=
        (measured_current_dq.q - drive->current_dq.q) *
        MOTOR_ENCODER_DRIVE_CURRENT_OBSERVER_ALPHA;
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
    drive->last_electrical_angle_raw = 0U;
    drive->mechanical_angle_raw = 0U;
    drive->last_mechanical_angle_raw = 0U;
    drive->mechanical_position_counts = 0;
    drive->mechanical_zero_counts = 0;
    drive->position_target_counts = 0;
    drive->measured_speed_rad_s = 0.0f;
    drive->speed_delta_accum_raw = 0;
    drive->speed_dt_accum_s = 0.0f;
    drive->speed_iq_integrator = 0.0f;
    drive->voltage_d_integrator = 0.0f;
    drive->voltage_q_integrator = 0.0f;
    drive->current_dq.d = 0.0f;
    drive->current_dq.q = 0.0f;
    drive->measured_current_dq.d = 0.0f;
    drive->measured_current_dq.q = 0.0f;
    drive->current_q_ref = 0.0f;
    drive->voltage_d_applied = 0.0f;
    drive->voltage_q_applied = 0.0f;
    drive->start_tick_ms = HAL_GetTick();
    drive->last_tick_ms = HAL_GetTick();
    drive->current_observer_ready = 0U;
    drive->speed_ready = 0U;
    drive->position_ready = 0U;
    drive->mode = MOTOR_ENCODER_DRIVE_MODE_SPEED;
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
    drive->last_electrical_angle_raw = 0U;
    drive->mechanical_angle_raw = 0U;
    drive->last_mechanical_angle_raw = 0U;
    drive->mechanical_position_counts = 0;
    drive->mechanical_zero_counts = 0;
    drive->position_target_counts = 0;
    drive->measured_speed_rad_s = 0.0f;
    drive->speed_delta_accum_raw = 0;
    drive->speed_dt_accum_s = 0.0f;
    drive->speed_iq_integrator = 0.0f;
    drive->voltage_d_integrator = 0.0f;
    drive->voltage_q_integrator = 0.0f;
    drive->current_dq.d = 0.0f;
    drive->current_dq.q = 0.0f;
    drive->measured_current_dq.d = 0.0f;
    drive->measured_current_dq.q = 0.0f;
    drive->current_q_ref = 0.0f;
    drive->voltage_d_applied = 0.0f;
    drive->voltage_q_applied = 0.0f;
    drive->start_tick_ms = HAL_GetTick();
    drive->last_tick_ms = HAL_GetTick();
    drive->current_observer_ready = 0U;
    drive->speed_ready = 0U;
    drive->position_ready = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef motor_encoder_drive_stop(motor_encoder_drive_t *drive) {
    if ((drive == NULL) || (drive->align == NULL)) {
        return HAL_ERROR;
    }

    drive->started = 0U;
    return motor_pwm_stop(drive->align->motor_pwm);
}

HAL_StatusTypeDef motor_encoder_drive_update_with_current_dt(
    motor_encoder_drive_t *drive,
    const motor_foc_abc_t *phase_current_abc,
    float dt_s) {
    uint16_t mech_raw = 0U;
    uint16_t electrical_raw;
    uint32_t now_ms;
    float voltage_limit;
    float elapsed_ratio;
    float speed_error;
    float current_q_error;
    uint8_t speed_integrator_allowed = 1U;
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
    motor_encoder_drive_update_mechanical_position(drive, mech_raw);

    electrical_raw = motor_encoder_align_get_elec_raw(drive->align, mech_raw);
    if (drive->sensor_direction < 0) {
        electrical_raw = (uint16_t)((0x4000U - electrical_raw) & 0x3FFFU);
    }

    now_ms = HAL_GetTick();
    if (drive->speed_ready == 0U) {
        drive->measured_speed_rad_s = 0.0f;
        drive->speed_ready = 1U;
    } else {
        drive->measured_speed_rad_s =
            motor_encoder_drive_estimate_speed(drive, electrical_raw, dt_s);
    }
    drive->electrical_angle_raw = electrical_raw;
    drive->electrical_angle_rad = motor_encoder_drive_raw_to_rad(electrical_raw);
    drive->last_electrical_angle_raw = electrical_raw;
    drive->last_tick_ms = now_ms;

    theta = drive->electrical_angle_rad;
    if (phase_current_abc != NULL) {
        motor_encoder_drive_update_current_observer(
            drive, motor_foc_park(motor_foc_clarke(*phase_current_abc), theta));
    }

    voltage_limit =
        motor_encoder_drive_ramp_voltage(drive->voltage_q,
                                         now_ms - drive->start_tick_ms);
    current_q_error = drive->current_q_ref - drive->current_dq.q;
    if ((phase_current_abc != NULL) &&
        (fabsf(current_q_error) > MOTOR_ENCODER_DRIVE_CURRENT_TRACKING_ERR_A)) {
        speed_integrator_allowed = 0U;
    }

    if (drive->mode == MOTOR_ENCODER_DRIVE_MODE_POSITION) {
        drive->speed_iq_integrator = 0.0f;
        drive->current_q_ref = motor_encoder_drive_position_to_iq_ref(drive);
    } else {
        elapsed_ratio =
            motor_encoder_drive_ramp_ratio(now_ms - drive->start_tick_ms);
        speed_error = (drive->electrical_speed_rad_s * elapsed_ratio) -
                      drive->measured_speed_rad_s;
        drive->current_q_ref =
            motor_encoder_drive_speed_to_iq_ref(drive, speed_error, dt_s,
                                                speed_integrator_allowed);
    }
    if (phase_current_abc != NULL) {
        voltage_dq =
            motor_encoder_drive_current_pi(drive, drive->current_q_ref,
                                           voltage_limit, dt_s);
    } else {
        voltage_dq.q =
            motor_encoder_drive_clampf(drive->current_q_ref, -voltage_limit,
                                       voltage_limit);
    }
    drive->voltage_d_applied = voltage_dq.d;
    drive->voltage_q_applied = voltage_dq.q;
    voltage_alphabeta = motor_foc_inverse_park(voltage_dq, theta);

    svpwm_generate(voltage_alphabeta.alpha, voltage_alphabeta.beta,
                   drive->bus_voltage,
                   drive->modulation_limit, &duty);
    motor_pwm_set_duty(drive->align->motor_pwm, duty.duty_a, duty.duty_b,
                       duty.duty_c);

    return HAL_OK;
}

HAL_StatusTypeDef motor_encoder_drive_update_with_current(
    motor_encoder_drive_t *drive,
    const motor_foc_abc_t *phase_current_abc) {
    uint32_t now_ms;
    uint32_t delta_ms;

    if (drive == NULL) {
        return HAL_ERROR;
    }

    now_ms = HAL_GetTick();
    delta_ms = now_ms - drive->last_tick_ms;
    return motor_encoder_drive_update_with_current_dt(
        drive, phase_current_abc, (float)delta_ms * 0.001f);
}

HAL_StatusTypeDef motor_encoder_drive_update(motor_encoder_drive_t *drive) {
    return motor_encoder_drive_update_with_current(drive, NULL);
}

void motor_encoder_drive_set_mode(motor_encoder_drive_t *drive,
                                  motor_encoder_drive_mode_t mode) {
    if (drive == NULL) {
        return;
    }

    drive->mode = mode;
    drive->speed_iq_integrator = 0.0f;
}

HAL_StatusTypeDef motor_encoder_drive_set_position_target_mdeg(
    motor_encoder_drive_t *drive,
    int32_t target_mdeg) {
    if (drive == NULL) {
        return HAL_ERROR;
    }

    drive->position_target_counts =
        motor_encoder_drive_mdeg_to_counts(target_mdeg);
    return HAL_OK;
}

HAL_StatusTypeDef motor_encoder_drive_set_mechanical_speed_target_mhz(
    motor_encoder_drive_t *drive,
    int32_t target_mhz) {
    if ((drive == NULL) || (drive->align == NULL) ||
        (drive->align->pole_pairs == 0U)) {
        return HAL_ERROR;
    }

    drive->electrical_speed_rad_s =
        (2.0f * (float)M_PI * (float)target_mhz *
         (float)drive->align->pole_pairs) /
        1000.0f;
    drive->speed_iq_integrator = 0.0f;
    return HAL_OK;
}

uint32_t motor_encoder_drive_get_electrical_mdeg(
    const motor_encoder_drive_t *drive) {
    if (drive == NULL) {
        return 0U;
    }

    return motor_encoder_align_raw_to_mdeg(drive->electrical_angle_raw);
}

int32_t motor_encoder_drive_get_position_mdeg(
    const motor_encoder_drive_t *drive) {
    if ((drive == NULL) || (drive->position_ready == 0U)) {
        return 0;
    }

    return motor_encoder_drive_counts_to_mdeg(
        drive->mechanical_position_counts - drive->mechanical_zero_counts);
}

int32_t motor_encoder_drive_get_position_target_mdeg(
    const motor_encoder_drive_t *drive) {
    if (drive == NULL) {
        return 0;
    }

    return motor_encoder_drive_counts_to_mdeg(drive->position_target_counts);
}

float motor_encoder_drive_get_measured_speed_hz(
    const motor_encoder_drive_t *drive) {
    if (drive == NULL) {
        return 0.0f;
    }

    return drive->measured_speed_rad_s / (2.0f * (float)M_PI);
}

float motor_encoder_drive_get_voltage_q(const motor_encoder_drive_t *drive) {
    if (drive == NULL) {
        return 0.0f;
    }

    return drive->voltage_q_applied;
}

float motor_encoder_drive_get_current_q_ref(const motor_encoder_drive_t *drive) {
    if (drive == NULL) {
        return 0.0f;
    }

    return drive->current_q_ref;
}

motor_foc_dq_t motor_encoder_drive_get_current_dq(
    const motor_encoder_drive_t *drive) {
    motor_foc_dq_t current_dq = {0.0f, 0.0f};

    if (drive == NULL) {
        return current_dq;
    }

    return drive->current_dq;
}
