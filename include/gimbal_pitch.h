#ifndef GIMBAL_PITCH_H
#define GIMBAL_PITCH_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Signed joint-side HOLD torque in the fixed-plane approximation:
 * J*qdd = tau - B*qdot - G(qg) + disturbance,
 * G(qg) = sin_nm*sin(qg) + cos_nm*cos(qg).
 * Coefficients use N m, expressed in the same positive torque direction as
 * the controller. An angular zero offset can be absorbed into the two
 * coefficients, or into the caller's qg, but must not be counted twice. */
typedef struct {
    float sin_nm;
    float cos_nm;
} gimbal_pitch_gravity_model_t;

/* qg is the measured orientation relative to gravity in the calibrated plane.
 * It is independent of the control angle and of the encoder's joint-relative
 * angle. A tilted base generally makes those angles different. Arbitrary 3-D
 * base/axis motion needs the appropriate vector gravity model, not this helper.
 * Returns false for non-finite arguments/results or null pointers. A non-null
 * hold_nm is cleared on failure. No state, hardware access, or dynamic memory. */
bool gimbal_pitch_gravity_torque(const gimbal_pitch_gravity_model_t *model,
                                float gravity_angle_rad, float *hold_nm);

#ifdef __cplusplus
}
#endif
#endif
