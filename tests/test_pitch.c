#include "gimbal_pitch_example.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_F 3.14159265358979323846f

static gimbal_pitch_example_config_t config(void)
{
    gimbal_pitch_example_config_t c;
    memset(&c, 0, sizeof(c));
    gimbal_smc_default_config(&c.smc);
    c.smc.torque_limit_nm = 1.0f;
    c.reference.max_rate_rad_s = 2.0f;
    c.reference.max_accel_rad_s2 = 10.0f;
    c.reference.position_gain_per_s = 10.0f;
    c.reference.dt_min_s = c.smc.dt_min_s;
    c.reference.dt_max_s = c.smc.dt_max_s;
    c.hard_min_rad = -0.6f;
    c.hard_max_rad = 0.6f;
    c.soft_margin_rad = 0.1f;
    c.gravity.sin_nm = 0.2f;
    c.gravity.cos_nm = -0.3f;
    return c;
}

static gimbal_pitch_example_snapshot_t snapshot(void)
{
    gimbal_pitch_example_snapshot_t s;
    memset(&s, 0, sizeof(s));
    s.axis.dt_s = 0.001f;
    s.axis.enabled = true;
    s.axis.feedback_valid = true;
    return s;
}

static void check_gravity(void)
{
    gimbal_pitch_gravity_model_t g = {0.2f, -0.3f};
    float torque = 99.0f;
    assert(gimbal_pitch_gravity_torque(&g, 0.0f, &torque));
    assert(fabsf(torque + 0.3f) < 1e-6f);
    assert(gimbal_pitch_gravity_torque(&g, PI_F * 0.5f, &torque));
    assert(fabsf(torque - 0.2f) < 1e-6f);
    assert(gimbal_pitch_gravity_torque(&g, -PI_F * 0.5f, &torque));
    assert(fabsf(torque + 0.2f) < 1e-6f);
    assert(gimbal_pitch_gravity_torque(&g, PI_F, &torque));
    assert(fabsf(torque - 0.3f) < 1e-6f);
    assert(!gimbal_pitch_gravity_torque(&g, NAN, &torque) && torque == 0.0f);
    torque = 99.0f;
    assert(!gimbal_pitch_gravity_torque(&g, INFINITY, &torque) && torque == 0.0f);
    assert(!gimbal_pitch_gravity_torque(NULL, 0.0f, &torque) && torque == 0.0f);
    assert(!gimbal_pitch_gravity_torque(&g, 0.0f, NULL));
    g.sin_nm = NAN;
    assert(!gimbal_pitch_gravity_torque(&g, 0.0f, &torque) && torque == 0.0f);
    g.sin_nm = g.cos_nm = FLT_MAX;
    assert(!gimbal_pitch_gravity_torque(&g, PI_F * 0.25f, &torque) && torque == 0.0f);
    g.sin_nm = 0.2f;
    g.cos_nm = -0.3f;
    assert(gimbal_pitch_gravity_torque(&g, 0.0f, &g.cos_nm));
    assert(fabsf(g.cos_nm + 0.3f) < 1e-6f);
}

static void check_configuration(void)
{
    gimbal_pitch_example_t pitch;
    gimbal_pitch_example_config_t c = config();
    assert(!gimbal_pitch_example_init(NULL, &c));
    assert(!gimbal_pitch_example_init(&pitch, NULL) && !pitch.initialized);
    c.smc.wrap_angle = true;
    assert(!gimbal_pitch_example_init(&pitch, &c) && !pitch.initialized);
    c = config();
    c.reference.wrap_angle = true;
    assert(!gimbal_pitch_example_init(&pitch, &c));
    c = config();
    c.soft_margin_rad = 0.6f;
    assert(!gimbal_pitch_example_init(&pitch, &c));
    c.soft_margin_rad = -0.1f;
    assert(!gimbal_pitch_example_init(&pitch, &c));
    c = config();
    c.hard_min_rad = NAN;
    assert(!gimbal_pitch_example_init(&pitch, &c));
    c = config();
    c.reference.max_accel_rad_s2 = 0.0f;
    assert(!gimbal_pitch_example_init(&pitch, &c));
    c = config();
    assert(gimbal_pitch_example_init(&pitch, &c));
    /* Reinitializing with the stored configuration must tolerate this alias. */
    assert(gimbal_pitch_example_init(&pitch, &pitch.config));
}

static void check_hold_coordinates_and_motors(void)
{
    gimbal_pitch_example_t pitch;
    gimbal_pitch_example_config_t c = config();
    gimbal_pitch_example_snapshot_t s = snapshot();
    gimbal_pitch_example_output_t out;
    int direction, sign;
    s.axis.angle_rad = 0.1f;
    s.joint_relative_angle_rad = -0.2f;
    assert(gimbal_pitch_example_init(&pitch, &c));
    for (sign = -1; sign <= 1; sign += 2) {
        /* Control angle and encoder angle stay fixed, while independently
         * supplied gravity phase changes the required signed hold torque. */
        s.gravity_angle_rad = sign < 0 ? 0.0f : PI_F;
        assert(gimbal_pitch_example_step(&pitch, &s, 0.1f, &out));
        assert(fabsf(out.control.error_rad) < 1e-7f);
        assert(fabsf(out.control.torque_nm - (float)sign * 0.3f) < 1e-6f);
        assert(fabsf(out.gravity_feedforward_nm - out.control.torque_nm) < 1e-6f);
        assert(out.pitch_flags == 0u);
        for (direction = -1; direction <= 1; direction += 2) {
            gimbal_dm4310_config_t dm = {1u, 0x11u, 1u, 12.5f, 30.0f, 10.0f,
                                         {1.0f, 1.0f, 1.0f, 1}};
            gimbal_gm6020_config_t gm = {2u, 1u, 0.741f, 1.0f, 0.0f,
                                         {1.0f, 1.0f, 1.0f, 1}};
            gimbal_can_frame_t frame;
            int16_t commands[4] = {111, 0, 333, 444};
            unsigned raw;
            float motor_torque;
            dm.mechanics.direction = gm.mechanics.direction = (int8_t)direction;
            assert(gimbal_example_dm_command(&dm, GIMBAL_EXAMPLE_DM_RUNNING,
                                              &out.control, &frame) == GIMBAL_MOTOR_OK);
            raw = ((unsigned)(frame.data[6] & 0x0fu) << 8u) | frame.data[7];
            motor_torque = (float)raw * 20.0f / 4095.0f - 10.0f;
            assert((float)(direction * sign) * motor_torque > 0.0f);
            assert(fabsf((float)direction * motor_torque - out.control.torque_nm) < 0.003f);
            assert(gimbal_example_gm_slot(&gm, &out.control, commands) == GIMBAL_MOTOR_OK);
            assert(direction * sign * (int)commands[1] > 0);
            assert(commands[0] == 111 && commands[2] == 333 && commands[3] == 444);
        }
    }

    /* The control-space interval is shifted by +0.5, from [-.5,.5] to [0,1]. */
    s.axis.angle_rad = 0.3f;
    s.joint_relative_angle_rad = -0.2f;
    gimbal_pitch_example_reset(&pitch);
    assert(gimbal_pitch_example_step(&pitch, &s, -20.0f, &out));
    assert((out.pitch_flags & GIMBAL_PITCH_TARGET_CLIPPED) != 0u);
    assert(fabsf(out.clamped_target_rad) < 1e-6f);
    assert(out.reference.angle_rad >= 0.0f && out.reference.angle_rad <= 1.0f);
    assert(gimbal_pitch_example_step(&pitch, &s, 20.0f, &out));
    assert(fabsf(out.clamped_target_rad - 1.0f) < 1e-6f);

    /* At either soft boundary a held reference must retain gravity torque. */
    for (sign = -1; sign <= 1; sign += 2) {
        s.axis.angle_rad = s.joint_relative_angle_rad = (float)sign * 0.5f;
        s.gravity_angle_rad = 0.0f;
        gimbal_pitch_example_reset(&pitch);
        assert(gimbal_pitch_example_step(&pitch, &s, (float)sign * 5.0f, &out));
        assert(fabsf(out.control.error_rad) < 1e-7f);
        assert(fabsf(out.control.torque_nm + 0.3f) < 1e-6f);
    }
}

static void check_reference_envelope(void)
{
    gimbal_pitch_example_t pitch;
    gimbal_pitch_example_config_t c = config();
    gimbal_pitch_example_snapshot_t s = snapshot();
    gimbal_pitch_example_output_t out;
    int sign;
    c.reference.position_gain_per_s = 40.0f;
    c.reference.max_accel_rad_s2 = 0.5f;
    s.axis.dt_s = 0.01f;
    for (sign = -1; sign <= 1; sign += 2) {
        gimbal_reference_t raw_reference;
        gimbal_smc_reference_t raw_out;
        bool raw_exceeded = false, projected = false;
        unsigned tick;
        assert(gimbal_pitch_example_init(&pitch, &c));
        assert(gimbal_reference_init(&raw_reference, &c.reference, 0.0f));
        for (tick = 0u; tick < 400u; ++tick) {
            /* An in-range target really produces an out-of-range state in the
             * unconstrained tracker; this does not inject an invented state. */
            assert(gimbal_reference_step(&raw_reference, (float)sign * 0.5f,
                                           s.axis.dt_s, &raw_out));
            if (fabsf(raw_out.angle_rad) > 0.5f) raw_exceeded = true;
            assert(gimbal_pitch_example_step(&pitch, &s, (float)sign * 0.5f, &out));
            assert(out.reference.angle_rad >= -0.5f && out.reference.angle_rad <= 0.5f);
            assert((out.pitch_flags & GIMBAL_PITCH_TARGET_CLIPPED) == 0u);
            if ((out.pitch_flags & GIMBAL_PITCH_REFERENCE_CLIPPED) != 0u) {
                projected = true;
                assert(out.reference.rate_rad_s == 0.0f && out.reference.accel_rad_s2 == 0.0f);
                assert(pitch.axis.reference.value.angle_rad == out.reference.angle_rad);
                assert(pitch.axis.reference.value.rate_rad_s == 0.0f);
                assert(pitch.axis.reference.value.accel_rad_s2 == 0.0f);
            }
        }
        assert(raw_exceeded && projected);
    }
}

static void check_faults_and_recovery(void)
{
    gimbal_pitch_example_t pitch;
    gimbal_pitch_example_config_t c = config();
    gimbal_pitch_example_snapshot_t s = snapshot();
    gimbal_pitch_example_output_t out;
    unsigned age;
    int sign;
    assert(gimbal_pitch_example_init(&pitch, &c));
    assert(gimbal_pitch_example_step(&pitch, &s, 0.0f, &out));
    assert(out.control.torque_nm != 0.0f);
    for (age = 0u; age < 3u; ++age) {
        float *ages[3] = {&s.axis.feedback_age_s, &s.joint_feedback_age_s,
                         &s.gravity_feedback_age_s};
        *ages[age] = 1.0f;
        assert(!gimbal_pitch_example_step(&pitch, &s, 0.0f, &out));
        assert((out.control.flags & GIMBAL_SMC_STALE_FEEDBACK) != 0u);
        assert(!out.control.valid && out.control.torque_nm == 0.0f && !pitch.axis.active);
        *ages[age] = 0.0f;
        s.axis.angle_rad = 0.2f;
        assert(gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
        assert(fabsf(out.control.error_rad) < 1e-7f);
        *ages[age] = NAN;
        assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
        assert((out.control.flags & GIMBAL_SMC_BAD_INPUT) != 0u);
        assert(out.control.torque_nm == 0.0f);
        *ages[age] = -0.01f;
        assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
        assert((out.control.flags & GIMBAL_SMC_BAD_INPUT) != 0u);
        *ages[age] = 0.0f;
    }
    s.gravity_angle_rad = NAN;
    assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
    assert((out.pitch_flags & GIMBAL_PITCH_BAD_INPUT) != 0u && out.control.torque_nm == 0.0f);
    s.gravity_angle_rad = 0.0f;
    s.joint_relative_angle_rad = INFINITY;
    assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out) && !pitch.fault_latched);
    s.joint_relative_angle_rad = 0.0f;
    assert(!gimbal_pitch_example_step(&pitch, &s, NAN, &out));
    s.axis.dt_s = NAN;
    assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
    assert((out.control.flags & GIMBAL_SMC_BAD_DT) != 0u);
    s.axis.dt_s = 0.001f;

    /* Stale geometry is rejected before it can latch a hard-limit fault. */
    s.joint_feedback_age_s = 1.0f;
    s.joint_relative_angle_rad = 0.7f;
    assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out) && !pitch.fault_latched);
    s.joint_feedback_age_s = 0.0f;
    /* Fresh joint violations cannot be masked by unrelated invalid targets,
     * gravity, control rate, period, or other stale feedback channels. */
    for (age = 0u; age < 7u; ++age) {
        float target = 0.0f;
        s = snapshot();
        s.joint_relative_angle_rad = 0.7f;
        if (age == 0u) target = NAN;
        if (age == 1u) s.gravity_angle_rad = NAN;
        if (age == 2u) s.axis.rate_rad_s = NAN;
        if (age == 3u) s.axis.dt_s = NAN;
        if (age == 4u) s.gravity_feedback_age_s = 1.0f;
        if (age == 5u) s.axis.feedback_age_s = 1.0f;
        if (age == 6u) s.axis.angle_rad = NAN;
        assert(!gimbal_pitch_example_step(&pitch, &s, target, &out));
        assert(pitch.fault_latched && (out.pitch_flags & GIMBAL_PITCH_HARD_LIMIT) != 0u);
        s = snapshot();
        assert(!gimbal_pitch_example_step(&pitch, &s, 0.0f, &out));
        assert((out.pitch_flags & GIMBAL_PITCH_FAULT_LATCHED) != 0u);
        gimbal_pitch_example_reset(&pitch);
        assert(gimbal_pitch_example_step(&pitch, &s, 0.0f, &out));
    }
    s.axis.angle_rad = 0.2f;
    for (sign = -1; sign <= 1; sign += 2) {
        s.joint_relative_angle_rad = (float)sign * 0.61f;
        assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
        assert(pitch.fault_latched && !out.control.valid && out.control.torque_nm == 0.0f);
        assert((out.pitch_flags & GIMBAL_PITCH_HARD_LIMIT) != 0u);
        s.joint_relative_angle_rad = 0.0f;
        assert(!gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
        assert((out.pitch_flags & GIMBAL_PITCH_FAULT_LATCHED) != 0u);
        gimbal_pitch_example_reset(&pitch);
        assert(!pitch.fault_latched && !pitch.axis.active);
        assert(gimbal_pitch_example_step(&pitch, &s, 0.2f, &out));
        assert(fabsf(out.control.error_rad) < 1e-7f);
    }

    s.axis.enabled = false;
    assert(!gimbal_pitch_example_step(&pitch, &s, 0.5f, &out));
    assert(out.control.torque_nm == 0.0f && !pitch.axis.active);
    assert((out.control.flags & GIMBAL_SMC_DISABLED) != 0u);
    s.axis.enabled = true;
    s.axis.angle_rad = -0.25f;
    assert(gimbal_pitch_example_step(&pitch, &s, -0.25f, &out));
    assert(fabsf(out.control.error_rad) < 1e-7f && fabsf(out.control.torque_nm + 0.3f) < 1e-6f);
    s.axis.feedback_valid = false;
    assert(!gimbal_pitch_example_step(&pitch, &s, -0.25f, &out));
    assert(out.control.torque_nm == 0.0f);
    s.axis.feedback_valid = true;
    assert(!gimbal_pitch_example_step(&pitch, NULL, 0.0f, &out));
    assert(out.control.torque_nm == 0.0f && !pitch.axis.active);
    assert(!gimbal_pitch_example_step(&pitch, &s, 0.0f, NULL));
    assert(!pitch.axis.active);
}

int main(void)
{
    check_gravity();
    check_configuration();
    check_hold_coordinates_and_motors();
    check_reference_envelope();
    check_faults_and_recovery();
    puts("pitch: signed hold torque, coordinates, both motor directions, reference bounds and recovery passed");
    return 0;
}
