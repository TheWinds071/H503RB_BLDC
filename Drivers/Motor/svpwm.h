#ifndef SVPWM_H
#define SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_duty_t;

void svpwm_generate(float v_alpha, float v_beta, float bus_voltage,
                    float modulation_limit, svpwm_duty_t *duty);

#ifdef __cplusplus
}
#endif

#endif
