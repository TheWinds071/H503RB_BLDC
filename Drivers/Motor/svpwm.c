#include "svpwm.h"

#include <math.h>

static float svpwm_clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void svpwm_generate(float v_alpha, float v_beta, float bus_voltage,
                    float modulation_limit, svpwm_duty_t *duty) {
    const float sqrt3_by_2 = 0.86602540378f;
    const float max_modulation = svpwm_clampf(modulation_limit, 0.0f, 1.0f);
    const float max_vector = 0.57735026919f * bus_voltage * max_modulation;
    const float vector_mag = sqrtf((v_alpha * v_alpha) + (v_beta * v_beta));
    float scale = 1.0f;
    float v_a;
    float v_b;
    float v_c;
    float v_offset;

    if ((duty == NULL) || (bus_voltage <= 0.0f)) {
        return;
    }

    if ((vector_mag > max_vector) && (vector_mag > 0.0f)) {
        scale = max_vector / vector_mag;
    }

    v_alpha *= scale;
    v_beta *= scale;

    v_a = v_alpha;
    v_b = (-0.5f * v_alpha) + (sqrt3_by_2 * v_beta);
    v_c = (-0.5f * v_alpha) - (sqrt3_by_2 * v_beta);

    {
        float v_max = v_a;
        float v_min = v_a;

        if (v_b > v_max) {
            v_max = v_b;
        }
        if (v_c > v_max) {
            v_max = v_c;
        }
        if (v_b < v_min) {
            v_min = v_b;
        }
        if (v_c < v_min) {
            v_min = v_c;
        }

        v_offset = -0.5f * (v_max + v_min);
    }

    v_a += v_offset;
    v_b += v_offset;
    v_c += v_offset;

    duty->duty_a = svpwm_clampf(0.5f + (v_a / bus_voltage), 0.0f, 1.0f);
    duty->duty_b = svpwm_clampf(0.5f + (v_b / bus_voltage), 0.0f, 1.0f);
    duty->duty_c = svpwm_clampf(0.5f + (v_c / bus_voltage), 0.0f, 1.0f);
}
