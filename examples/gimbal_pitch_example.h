#ifndef GIMBAL_PITCH_EXAMPLE_H
#define GIMBAL_PITCH_EXAMPLE_H

#include "gimbal_controller_example.h"
#include "gimbal_pitch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gimbal_smc_config_t smc;
    gimbal_reference_config_t reference;
    gimbal_pitch_gravity_model_t gravity;
    float hard_min_rad;    /* Encoder joint coordinates, same positive direction. */
    float hard_max_rad;
    float soft_margin_rad; /* >=0; leaves a nonempty interval inside hard limits. */
} gimbal_pitch_example_config_t;

typedef struct {
    gimbal_example_snapshot_t axis;
    float joint_relative_angle_rad;
    float gravity_angle_rad;
    float joint_feedback_age_s;
    float gravity_feedback_age_s;
} gimbal_pitch_example_snapshot_t;

typedef struct {
    gimbal_example_axis_t axis;
    gimbal_pitch_example_config_t config; /* Initialize again to change config. */
    bool initialized;
    bool fault_latched;
} gimbal_pitch_example_t;

/* Separate namespace from control.flags; clipping is not a controller fault. */
enum {
    GIMBAL_PITCH_TARGET_CLIPPED    = 1u << 0,
    GIMBAL_PITCH_REFERENCE_CLIPPED = 1u << 1,
    GIMBAL_PITCH_HARD_LIMIT        = 1u << 2,
    GIMBAL_PITCH_FAULT_LATCHED     = 1u << 3,
    GIMBAL_PITCH_BAD_INPUT         = 1u << 4,
    GIMBAL_PITCH_BAD_CONFIG        = 1u << 5
};

typedef struct {
    gimbal_smc_output_t control;
    uint32_t pitch_flags;
    float clamped_target_rad; /* Control coordinates. */
    gimbal_smc_reference_t reference; /* The exact reference passed to the core. */
    float gravity_feedforward_nm;
} gimbal_pitch_example_output_t;

/* Both smc.wrap_angle and reference.wrap_angle MUST be false. No silent
 * override. Controller gains, inertia and reference limits remain per-axis. */
bool gimbal_pitch_example_init(gimbal_pitch_example_t *pitch,
                               const gimbal_pitch_example_config_t *config);

/* Explicit upper-supervisor acknowledgement of a hard-limit fault. Clears
 * dynamic state and latch; next valid step anchors to fresh control feedback.
 * It does not send FC or authorize/enable an actuator. */
void gimbal_pitch_example_reset(gimbal_pitch_example_t *pitch);

/* Position-target bridge for same-sign, locally additive pitch coordinates:
 * control_angle = joint_relative_angle + base_offset. The current offset maps
 * the mechanical soft interval into control coordinates. Both target and
 * generated reference are projected into that interval. Projection resets
 * reference velocity/acceleration too, including hidden reference state.
 *
 * This is a CURRENT-SNAPSHOT REFERENCE ENVELOPE, not a braking planner or a
 * guarantee of continuous joint travel bounds under moving-base dynamics.
 * Projection can break normal reference acceleration limits. The upper mode
 * supervisor must provide stopping distance/margin and mechanical protection.
 *
 * Inside hard limits, the soft boundary retains gravity support. Fresh enabled
 * feedback outside hard limits instead latches a fault with zero output until
 * reset() is explicitly called. A zero/disabled command releases gravity
 * support: the mechanism's balance/brake/support policy belongs to the caller.
 * Other invalid/stale/disabled samples clear active state; enabled must already
 * reflect the upper supervisor's recovery authorization. All feedback ages
 * participate in the controller timeout. No CAN frames are sent here. */
bool gimbal_pitch_example_step(gimbal_pitch_example_t *pitch,
                               const gimbal_pitch_example_snapshot_t *snapshot,
                               float target_control_rad,
                               gimbal_pitch_example_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
