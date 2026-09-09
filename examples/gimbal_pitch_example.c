#include "gimbal_pitch_example.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool pitch_config_valid(const gimbal_pitch_example_config_t *c)
{
    float lo, hi;
    if (c == NULL || c->smc.wrap_angle || c->reference.wrap_angle ||
        !isfinite(c->gravity.sin_nm) || !isfinite(c->gravity.cos_nm) ||
        !isfinite(c->hard_min_rad) || !isfinite(c->hard_max_rad) ||
        !isfinite(c->soft_margin_rad) || c->soft_margin_rad < 0.0f ||
        c->hard_min_rad >= c->hard_max_rad) return false;
    lo = c->hard_min_rad + c->soft_margin_rad;
    hi = c->hard_max_rad - c->soft_margin_rad;
    return isfinite(lo) && isfinite(hi) && lo < hi;
}

static void clear_dynamic(gimbal_pitch_example_t *pitch)
{
    if (pitch != NULL) {
        pitch->axis.active = false;
        gimbal_smc_reset(&pitch->axis.controller);
    }
}

static bool fail(gimbal_pitch_example_t *pitch,
                 gimbal_pitch_example_output_t *out,
                 uint32_t pitch_flags, uint32_t control_flags)
{
    clear_dynamic(pitch);
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        out->pitch_flags = pitch_flags;
        out->control.flags = control_flags;
    }
    return false;
}

bool gimbal_pitch_example_init(gimbal_pitch_example_t *pitch,
                               const gimbal_pitch_example_config_t *config)
{
    gimbal_pitch_example_config_t copy;
    if (pitch == NULL) return false;
    if (!pitch_config_valid(config)) {
        memset(pitch, 0, sizeof(*pitch));
        return false;
    }
    copy = *config;
    memset(pitch, 0, sizeof(*pitch));
    if (!gimbal_example_axis_init(&pitch->axis, &copy.smc, &copy.reference))
        return false;
    pitch->config = copy;
    pitch->initialized = true;
    return true;
}

void gimbal_pitch_example_reset(gimbal_pitch_example_t *pitch)
{
    if (pitch == NULL) return;
    clear_dynamic(pitch);
    pitch->fault_latched = false;
    (void)gimbal_reference_reset(&pitch->axis.reference, 0.0f);
}

static float clampf(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

bool gimbal_pitch_example_step(gimbal_pitch_example_t *pitch,
                               const gimbal_pitch_example_snapshot_t *sample,
                               float target_control_rad,
                               gimbal_pitch_example_output_t *out)
{
    gimbal_smc_input_t in = {0};
    gimbal_smc_reference_t ref;
    const gimbal_pitch_example_config_t *c;
    float offset, lo, hi, projected;
    if (out == NULL)
        return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_BAD_INPUT);
    memset(out, 0, sizeof(*out));
    if (pitch == NULL || sample == NULL)
        return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_BAD_INPUT);
    if (!pitch->initialized || !pitch_config_valid(&pitch->config))
        return fail(pitch, out, GIMBAL_PITCH_BAD_CONFIG, GIMBAL_SMC_BAD_CONFIG);
    if (pitch->fault_latched)
        return fail(pitch, out, GIMBAL_PITCH_HARD_LIMIT | GIMBAL_PITCH_FAULT_LATCHED,
                    GIMBAL_SMC_DISABLED);
    c = &pitch->config;
    if (!sample->axis.enabled || !sample->axis.feedback_valid)
        return fail(pitch, out, 0u, GIMBAL_SMC_DISABLED);

    /* Independently qualified, fresh joint geometry takes precedence over
     * target/control/gravity errors. None of those may hide a hard violation
     * and allow later recovery without the supervisor acknowledging it. */
    if (!isfinite(sample->joint_relative_angle_rad) ||
        !isfinite(sample->joint_feedback_age_s) || sample->joint_feedback_age_s < 0.0f)
        return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_BAD_INPUT);
    if (sample->joint_feedback_age_s > c->smc.feedback_timeout_s)
        return fail(pitch, out, 0u, GIMBAL_SMC_STALE_FEEDBACK);
    if (sample->joint_relative_angle_rad < c->hard_min_rad ||
        sample->joint_relative_angle_rad > c->hard_max_rad) {
        pitch->fault_latched = true;
        return fail(pitch, out, GIMBAL_PITCH_HARD_LIMIT | GIMBAL_PITCH_FAULT_LATCHED,
                    GIMBAL_SMC_DISABLED);
    }
    /* Validate every age before taking max: fmaxf alone can hide a NaN. */
    if (!isfinite(sample->axis.angle_rad) || !isfinite(sample->axis.rate_rad_s) ||
        !isfinite(sample->gravity_angle_rad) || !isfinite(target_control_rad) ||
        !isfinite(sample->axis.feedback_age_s) || sample->axis.feedback_age_s < 0.0f ||
        !isfinite(sample->gravity_feedback_age_s) || sample->gravity_feedback_age_s < 0.0f)
        return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_BAD_INPUT);
    in.feedback_age_s = fmaxf(sample->axis.feedback_age_s,
        fmaxf(sample->joint_feedback_age_s, sample->gravity_feedback_age_s));
    in.dt_s = sample->axis.dt_s;
    if (!isfinite(in.dt_s) || in.dt_s < c->smc.dt_min_s || in.dt_s > c->smc.dt_max_s)
        return fail(pitch, out, 0u, GIMBAL_SMC_BAD_DT);
    if (in.feedback_age_s > c->smc.feedback_timeout_s)
        return fail(pitch, out, 0u, GIMBAL_SMC_STALE_FEEDBACK);
    if (!gimbal_pitch_gravity_torque(&c->gravity, sample->gravity_angle_rad,
                                     &in.feedforward_nm))
        return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_NUMERIC_FAULT);
    offset = sample->axis.angle_rad - sample->joint_relative_angle_rad;
    lo = offset + (c->hard_min_rad + c->soft_margin_rad);
    hi = offset + (c->hard_max_rad - c->soft_margin_rad);
    if (!isfinite(offset) || !isfinite(lo) || !isfinite(hi) || lo >= hi)
        return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_NUMERIC_FAULT);
    out->clamped_target_rad = clampf(target_control_rad, lo, hi);
    if (out->clamped_target_rad != target_control_rad)
        out->pitch_flags |= GIMBAL_PITCH_TARGET_CLIPPED;
    if ((!pitch->axis.active &&
         !gimbal_reference_reset(&pitch->axis.reference, sample->axis.angle_rad)) ||
        !gimbal_reference_step(&pitch->axis.reference, out->clamped_target_rad,
                                in.dt_s, &ref))
        return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_BAD_INPUT);
    projected = clampf(ref.angle_rad, lo, hi);
    if (projected != ref.angle_rad ||
        (projected == lo && ref.rate_rad_s < 0.0f) ||
        (projected == hi && ref.rate_rad_s > 0.0f)) {
        /* Target clipping alone cannot contain a stateful reference. Project
         * its entire state before ONE controller update, never after it. */
        if (!gimbal_reference_reset(&pitch->axis.reference, projected))
            return fail(pitch, out, GIMBAL_PITCH_BAD_INPUT, GIMBAL_SMC_BAD_INPUT);
        ref = pitch->axis.reference.value;
        out->pitch_flags |= GIMBAL_PITCH_REFERENCE_CLIPPED;
    }
    in.angle_rad = sample->axis.angle_rad;
    in.rate_rad_s = sample->axis.rate_rad_s;
    in.reference_rad = ref.angle_rad;
    in.reference_rate_rad_s = ref.rate_rad_s;
    in.reference_accel_rad_s2 = ref.accel_rad_s2;
    in.enable = true;
    out->reference = ref;
    out->gravity_feedforward_nm = in.feedforward_nm;
    (void)gimbal_smc_update(&pitch->axis.controller, &in, &out->control);
    pitch->axis.active = out->control.valid;
    return out->control.valid;
}
