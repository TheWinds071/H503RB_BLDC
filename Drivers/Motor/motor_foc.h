#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float a;
    float b;
    float c;
} motor_foc_abc_t;

typedef struct {
    float alpha;
    float beta;
} motor_foc_alphabeta_t;

typedef struct {
    float d;
    float q;
} motor_foc_dq_t;

motor_foc_alphabeta_t motor_foc_clarke(motor_foc_abc_t abc);
motor_foc_dq_t motor_foc_park(motor_foc_alphabeta_t alphabeta,
                              float electrical_angle_rad);
motor_foc_alphabeta_t motor_foc_inverse_park(motor_foc_dq_t dq,
                                             float electrical_angle_rad);

#ifdef __cplusplus
}
#endif

#endif
