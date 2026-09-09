#include "gimbal_pitch.h"

#include <math.h>
#include <stddef.h>

bool gimbal_pitch_gravity_torque(const gimbal_pitch_gravity_model_t *model,
                                float gravity_angle_rad, float *hold_nm)
{
    gimbal_pitch_gravity_model_t copy;
    float torque;
    if (hold_nm == NULL) return false;
    if (model == NULL) {
        *hold_nm = 0.0f;
        return false;
    }
    copy = *model;
    *hold_nm = 0.0f;
    if (!isfinite(copy.sin_nm) || !isfinite(copy.cos_nm) ||
        !isfinite(gravity_angle_rad)) return false;
    torque = copy.sin_nm * sinf(gravity_angle_rad) +
             copy.cos_nm * cosf(gravity_angle_rad);
    if (!isfinite(torque)) return false;
    *hold_nm = torque;
    return true;
}
