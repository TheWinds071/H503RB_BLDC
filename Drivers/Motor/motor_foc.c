#include "motor_foc.h"

#include <math.h>

motor_foc_alphabeta_t motor_foc_clarke(motor_foc_abc_t abc) {
    const float one_by_sqrt3 = 0.57735026919f;
    motor_foc_alphabeta_t alphabeta;

    alphabeta.alpha = abc.a;
    alphabeta.beta = (abc.a + (2.0f * abc.b)) * one_by_sqrt3;

    return alphabeta;
}

motor_foc_dq_t motor_foc_park(motor_foc_alphabeta_t alphabeta,
                              float electrical_angle_rad) {
    const float sin_theta = sinf(electrical_angle_rad);
    const float cos_theta = cosf(electrical_angle_rad);
    motor_foc_dq_t dq;

    dq.d = (alphabeta.alpha * cos_theta) + (alphabeta.beta * sin_theta);
    dq.q = (-alphabeta.alpha * sin_theta) + (alphabeta.beta * cos_theta);

    return dq;
}

motor_foc_alphabeta_t motor_foc_inverse_park(motor_foc_dq_t dq,
                                             float electrical_angle_rad) {
    const float sin_theta = sinf(electrical_angle_rad);
    const float cos_theta = cosf(electrical_angle_rad);
    motor_foc_alphabeta_t alphabeta;

    alphabeta.alpha = (dq.d * cos_theta) - (dq.q * sin_theta);
    alphabeta.beta = (dq.d * sin_theta) + (dq.q * cos_theta);

    return alphabeta;
}
