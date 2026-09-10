#include "gimbal_identification.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); \
    exit(EXIT_FAILURE); } } while (0)
#define PI_D 3.14159265358979323846
#define SAMPLE_US 2000u

typedef struct {
    double j, b, fc, epsilon, a, c;
} truth_t;

static const truth_t yaw_truth = {0.021, 0.012, 0.037, 0.08, 0.0, 0.0};
static const truth_t pitch_truth = {0.014, 0.009, 0.028, 0.065, -0.07, 0.22};

static int near(double a, double b, double tolerance)
{
    return isfinite(a) && isfinite(b) && fabs(a - b) <= tolerance;
}

static gimbal_identification_config_t configuration(gimbal_identification_axis_t axis)
{
    gimbal_identification_config_t c;
    unsigned int i;
    gimbal_identification_default_config(&c, axis);
    c.configuration_id = axis == GIMBAL_IDENTIFICATION_YAW ? 101u : 202u;
    c.window_s = 0.05f;
    c.max_gap_s = 0.006f;
    c.feedback_timeout_s = 0.02f;
    c.friction_velocity_rad_s = (float)(axis == GIMBAL_IDENTIFICATION_YAW ? yaw_truth.epsilon : pitch_truth.epsilon);
    c.rls.excitation_window = 32u;
    c.rls.min_updates = 12u;
    /* Finite prior limits early corrections while the 5-column pitch model
     * becomes distinguishable. A huge P0 plus early bound projections can
     * leave finite-record bias; arbitrary priors are not certified here. */
    c.rls.covariance_initial = 20.0f;
    c.rls.covariance_limit = 1000000.0f;
    c.rls.forgetting_time_s = 0.0f;
    c.rls.output_scale = 1.0f;
    c.rls.dt_min_s = 0.04f;
    c.rls.dt_max_s = 0.06f;
    c.rls.min_feature_energy = 0.00000001f;
    c.rls.min_excitation_pivot = 0.001f;
    c.rls.innovation_rms_limit = 0.03f;
    for (i = 0u; i < c.rls.parameter_count; ++i) {
        c.rls.initial[i] = i < 3u ? 0.03f : 0.0f;
        c.rls.minimum[i] = i < 3u ? 0.0f : -1.0f;
        c.rls.maximum[i] = i < 3u ? 0.3f : 1.0f;
        c.rls.feature_scale[i] = 1.0f;
    }
    c.rls.minimum[0] = 0.00001f;
    c.rls.feature_scale[0] = 10.0f;
    c.rls.feature_scale[1] = 5.0f;
    return c;
}

/* Independent inverse-dynamics fixture: analytic q, qdot, qddot. There is no
 * numerical differentiator, regression window builder or RLS in this oracle.
 * No sensor noise is added; tolerances cover sampled trapezoids and float
 * coordinates, including continuous yaw angles well beyond one revolution. */
static gimbal_identification_sample_t analytic_sample(
    const gimbal_identification_config_t *c, double t, uint32_t timestamp,
    uint32_t segment, int heldout)
{
    const int yaw = c->axis == GIMBAL_IDENTIFICATION_YAW;
    const truth_t *truth = yaw ? &yaw_truth : &pitch_truth;
    const double amplitudes[3] = {yaw ? 0.9 : 0.45, yaw ? 0.3 : 0.21, yaw ? 0.09 : 0.07};
    const double frequencies[3] = {heldout ? 0.23 : 0.17, heldout ? 0.61 : 0.43, heldout ? 1.13 : 0.91};
    const double phases[3] = {heldout ? 0.7 : 0.1, heldout ? -0.4 : 0.8, heldout ? 1.1 : -0.3};
    const double tilt = heldout ? -0.19 : 0.23;
    double q = yaw ? 8.0 * PI_D + 0.7 * t : -0.05;
    double v = yaw ? 0.7 : 0.0, a = 0.0, gravity;
    gimbal_identification_sample_t s;
    unsigned int i;
    for (i = 0u; i < 3u; ++i) {
        const double w = 2.0 * PI_D * frequencies[i];
        const double argument = w * t + phases[i];
        q += amplitudes[i] * sin(argument);
        v += amplitudes[i] * w * cos(argument);
        a -= amplitudes[i] * w * w * sin(argument);
    }
    gravity = q + (yaw ? 0.0 : tilt);
    memset(&s, 0, sizeof(s));
    s.timestamp_us = timestamp;
    s.segment_id = segment;
    s.configuration_id = c->configuration_id;
    s.angle_rad = (float)q;
    s.rate_rad_s = (float)v;
    s.gravity_angle_rad = (float)gravity;
    s.torque_nm = (float)(truth->j * a + truth->b * v + truth->fc * tanh(v / truth->epsilon) +
                         truth->a * sin(gravity) + truth->c * cos(gravity));
    s.enabled = s.valid = s.operating_conditions_valid = true;
    s.torque_calibrated = s.torque_time_aligned = true;
    return s;
}

static void unchanged_model(const gimbal_identification_t *a,
                            const gimbal_identification_t *b)
{
    CHECK(memcmp(a->estimator.theta, b->estimator.theta, sizeof(a->estimator.theta)) == 0);
    CHECK(memcmp(a->estimator.covariance, b->estimator.covariance, sizeof(a->estimator.covariance)) == 0);
    CHECK(a->estimator.accepted_updates == b->estimator.accepted_updates);
}

static double heldout_acceleration(gimbal_identification_axis_t axis, double t)
{
    const int yaw = axis == GIMBAL_IDENTIFICATION_YAW;
    const double amplitudes[3] = {yaw ? 0.9 : 0.45, yaw ? 0.3 : 0.21, yaw ? 0.09 : 0.07};
    const double frequencies[3] = {0.23, 0.61, 1.13};
    const double phases[3] = {0.7, -0.4, 1.1};
    double acceleration = 0.0;
    unsigned int i;
    for (i = 0u; i < 3u; ++i) {
        const double w = 2.0 * PI_D * frequencies[i];
        acceleration -= amplitudes[i] * w * w * sin(w * t + phases[i]);
    }
    return acceleration;
}

static void test_analytic_recovery_and_fixed_tilt(void)
{
    unsigned int axis;
    for (axis = 0u; axis < 2u; ++axis) {
        const gimbal_identification_axis_t selected = axis == 0u ? GIMBAL_IDENTIFICATION_YAW : GIMBAL_IDENTIFICATION_PITCH;
        const truth_t *truth = axis == 0u ? &yaw_truth : &pitch_truth;
        gimbal_identification_config_t c = configuration(selected);
        gimbal_identification_t id, peer, untouched;
        gimbal_identification_output_t out, learned;
        unsigned int i, accepted = 0u, ready = 0u, projected = 0u;
        double smallest = 1e9, largest = -1e9;
        memset(&learned, 0, sizeof(learned));
        CHECK(gimbal_identification_init(&id, &c));
        CHECK(gimbal_identification_init(&peer, &c));
        untouched = peer;
        for (i = 0u; i <= 60000u; ++i) {
            gimbal_identification_sample_t s = analytic_sample(&c, i * 0.002, i * SAMPLE_US, 7u, 0);
            smallest = fmin(smallest, s.angle_rad); largest = fmax(largest, s.angle_rad);
            gimbal_identification_update(&id, &s, &out);
            if (i < 20u) CHECK(!out.ready && !out.window_updated);
            if (!out.window_updated) CHECK(!out.ready);
            if (out.window_updated) { ++accepted; learned = out; }
            if (out.ready) ++ready;
            if (out.rls.flags & GIMBAL_RLS_PROJECTED) ++projected;
        }
        CHECK(accepted > 1500u && ready > 100u);
        CHECK(memcmp(&peer, &untouched, sizeof(peer)) == 0);
        if (axis == 0u) CHECK(largest - smallest > 10.0 * PI_D);
        printf("%s: updates=%u ready=%u projected=%u J=%.9g B=%.9g Fc=%.9g A=%.9g C=%.9g\n",
               axis == 0u ? "yaw" : "pitch", accepted, ready, projected,
               learned.model.inertia_kg_m2, learned.model.viscous_nm_s_rad,
               learned.model.coulomb_nm, learned.model.gravity_sin_nm,
               learned.model.gravity_cos_nm);
        CHECK(near(learned.model.inertia_kg_m2, truth->j, 0.0004));
        CHECK(near(learned.model.viscous_nm_s_rad, truth->b, 0.0006));
        CHECK(near(learned.model.coulomb_nm, truth->fc, 0.0008));
        CHECK(near(learned.model.gravity_sin_nm, truth->a, 0.001));
        CHECK(near(learned.model.gravity_cos_nm, truth->c, 0.001));
        CHECK(near(learned.model.friction_velocity_rad_s, truth->epsilon, 0.000001));
        if (axis == 0u) CHECK(learned.model.gravity_sin_nm == 0.0f && learned.model.gravity_cos_nm == 0.0f);
        /* Independent held-out pointwise torque prediction, with different
         * frequencies and pitch base tilt. Not another fit to the same record. */
        {
            double error2 = 0.0;
            for (i = 0u; i < 2000u; ++i) {
                const double t = i * 0.0037;
                gimbal_identification_sample_t s = analytic_sample(&c, t, 0u, 0u, 1);
                const double acceleration = heldout_acceleration(c.axis, t);
                const double predicted = learned.model.inertia_kg_m2 * acceleration +
                    learned.model.viscous_nm_s_rad * s.rate_rad_s +
                    learned.model.coulomb_nm * tanh(s.rate_rad_s / learned.model.friction_velocity_rad_s) +
                    learned.model.gravity_sin_nm * sin(s.gravity_angle_rad) +
                    learned.model.gravity_cos_nm * cos(s.gravity_angle_rad);
                const double error = predicted - s.torque_nm;
                error2 += error * error;
            }
            CHECK(sqrt(error2 / 2000.0) < 0.003);
        }
        gimbal_identification_reset(&id);
        CHECK(id.estimator.accepted_updates == 0u && !id.have_timestamp && !id.window_active);
        {
            gimbal_identification_sample_t s = analytic_sample(&c, 0.0, 0u, 7u, 0);
            gimbal_identification_update(&id, &s, &out);
            CHECK(!out.ready && !out.window_updated);
            CHECK(near(out.model.inertia_kg_m2, c.rls.initial[0], 0.000001));
        }
    }
}

static void warm(gimbal_identification_t *id, const gimbal_identification_config_t *c)
{
    gimbal_identification_output_t out;
    unsigned int i, ready = 0u;
    CHECK(gimbal_identification_init(id, c));
    for (i = 0u; i <= 12000u; ++i) {
        gimbal_identification_sample_t s = analytic_sample(c, i * 0.002, i * SAMPLE_US, 7u, 0);
        gimbal_identification_update(id, &s, &out);
        if (out.ready) ++ready;
    }
    CHECK(ready > 0u);
}

static void test_breaks_do_not_bridge_samples_or_reuse_readiness(void)
{
    gimbal_identification_config_t c = configuration(GIMBAL_IDENTIFICATION_PITCH);
    gimbal_identification_t baseline, id, before;
    gimbal_identification_output_t out;
    unsigned int kind, j;
    warm(&baseline, &c);
    for (kind = 0u; kind < 16u; ++kind) {
        uint32_t expected;
        gimbal_identification_sample_t s = analytic_sample(&c, 24.002, 24002000u, 7u, 0);
        id = baseline;
        before = id;
        switch (kind) {
        case 0: s.enabled = false; expected = GIMBAL_IDENTIFICATION_DISABLED; break;
        case 1: s.valid = false; expected = GIMBAL_IDENTIFICATION_BAD_INPUT; break;
        case 2: s.saturated = true; expected = GIMBAL_IDENTIFICATION_SATURATED; break;
        case 3: s.feedback_age_s = 0.03f; expected = GIMBAL_IDENTIFICATION_STALE_FEEDBACK; break;
        case 4: s.operating_conditions_valid = false; expected = GIMBAL_IDENTIFICATION_UNQUALIFIED_OPERATION; break;
        case 5: s.torque_calibrated = false; expected = GIMBAL_IDENTIFICATION_TORQUE_UNCALIBRATED; break;
        case 6: s.torque_time_aligned = false; expected = GIMBAL_IDENTIFICATION_TORQUE_UNALIGNED; break;
        case 7: s.timestamp_us = 24000000u; expected = GIMBAL_IDENTIFICATION_BAD_TIMESTAMP; break;
        case 8: s.timestamp_us = 23999000u; expected = GIMBAL_IDENTIFICATION_BAD_TIMESTAMP; break;
        case 9: s.timestamp_us = 24020000u; expected = GIMBAL_IDENTIFICATION_GAP; break;
        case 10: s.segment_id = 8u; expected = GIMBAL_IDENTIFICATION_SEGMENT_CHANGE; break;
        case 11: s.configuration_id = 203u; expected = GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH; break;
        case 12: s.angle_rad = NAN; expected = GIMBAL_IDENTIFICATION_BAD_INPUT; break;
        case 13: s.rate_rad_s = INFINITY; expected = GIMBAL_IDENTIFICATION_BAD_INPUT; break;
        case 14: s.torque_nm = -INFINITY; expected = GIMBAL_IDENTIFICATION_BAD_INPUT; break;
        default: s.feedback_age_s = -1.0f; expected = GIMBAL_IDENTIFICATION_BAD_INPUT; break;
        }
        CHECK(gimbal_identification_update(&id, &s, &out) & expected);
        CHECK(!out.ready && !out.window_updated && !out.rls.ready);
        unchanged_model(&id, &before);
        CHECK(id.estimator.observation_count == 0u && id.estimator.consecutive_updates == 0u);
        /* A new valid partial window must never finish the discarded interval.
         * Preserve the new segment after a segment transition. */
        for (j = 1u; j <= 8u; ++j) {
            const uint32_t stamp = 24022000u + j * SAMPLE_US;
            gimbal_identification_sample_t next = analytic_sample(&c, stamp * 0.000001, stamp, kind == 10u ? 8u : 7u, 0);
            gimbal_identification_update(&id, &next, &out);
            CHECK(!out.ready && !out.window_updated && out.window_duration_s == 0.0f);
            unchanged_model(&id, &before);
        }
    }
}

static void test_timestamp_wrap_and_configuration_isolation(void)
{
    gimbal_identification_config_t c = configuration(GIMBAL_IDENTIFICATION_YAW), new_config;
    gimbal_identification_t normal, wrapped, before;
    gimbal_identification_output_t a, b;
    unsigned int i, updated = 0u;
    const uint32_t offset = UINT32_MAX - 49000u;
    CHECK(gimbal_identification_init(&normal, &c));
    CHECK(gimbal_identification_init(&wrapped, &c));
    for (i = 0u; i <= 12000u; ++i) {
        gimbal_identification_sample_t s = analytic_sample(&c, i * 0.002, i * SAMPLE_US, 9u, 0);
        gimbal_identification_sample_t w = s;
        w.timestamp_us += offset;
        gimbal_identification_update(&normal, &s, &a);
        gimbal_identification_update(&wrapped, &w, &b);
        CHECK(a.flags == b.flags && a.ready == b.ready && a.window_updated == b.window_updated);
        CHECK(memcmp(normal.estimator.theta, wrapped.estimator.theta, sizeof(normal.estimator.theta)) == 0);
        CHECK(memcmp(normal.estimator.covariance, wrapped.estimator.covariance, sizeof(normal.estimator.covariance)) == 0);
        if (a.window_updated) ++updated;
    }
    CHECK(updated > 200u);
    before = normal;
    for (i = 0u; i < 40u; ++i) {
        gimbal_identification_sample_t s = analytic_sample(&c, 24.002 + i * 0.002, 24002000u + i * SAMPLE_US, 9u, 0);
        s.configuration_id = c.configuration_id + 1u;
        CHECK(gimbal_identification_update(&normal, &s, &a) & GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH);
        CHECK(!a.ready && !a.window_updated);
        unchanged_model(&normal, &before);
    }
    new_config = c;
    new_config.configuration_id += 1u;
    new_config.rls.initial[0] = 0.06f;
    CHECK(gimbal_identification_init(&normal, &new_config));
    CHECK(normal.estimator.accepted_updates == 0u);
    {
        gimbal_identification_sample_t s = analytic_sample(&new_config, 0.0, 0u, 0u, 0);
        gimbal_identification_update(&normal, &s, &a);
        CHECK(!a.ready && !a.window_updated && near(a.model.inertia_kg_m2, 0.06, 0.000001));
    }
}

static void test_bad_configuration(void)
{
    gimbal_identification_t id;
    gimbal_identification_config_t c;
    unsigned int kind;
    for (kind = 0u; kind < 12u; ++kind) {
        c = configuration(GIMBAL_IDENTIFICATION_PITCH);
        switch (kind) {
        case 0: c.axis = (gimbal_identification_axis_t)2; break;
        case 1: c.window_s = 0.0f; break;
        case 2: c.max_gap_s = NAN; break;
        case 3: c.feedback_timeout_s = INFINITY; break;
        case 4: c.friction_velocity_rad_s = 0.0f; break;
        case 5: c.rls.parameter_count = 3u; break;
        case 6: c.rls.minimum[0] = 0.0f; break;
        case 7: c.rls.minimum[1] = -1.0f; break;
        case 8: c.rls.minimum[2] = -1.0f; break;
        case 9: c.rls.dt_min_s = 0.055f; break;
        case 10: c.rls.dt_max_s = 0.052f; break;
        default: c.window_s = 2200.0f; break;
        }
        CHECK(!gimbal_identification_init(&id, &c) && !id.initialized);
    }
    c = configuration(GIMBAL_IDENTIFICATION_YAW);
    CHECK(!gimbal_identification_init(NULL, &c));
    CHECK(!gimbal_identification_init(&id, NULL));
}

int main(void)
{
    test_analytic_recovery_and_fixed_tilt();
    test_breaks_do_not_bridge_samples_or_reuse_readiness();
    test_timestamp_wrap_and_configuration_isolation();
    test_bad_configuration();
    puts("Online integral identification: analytic recovery, boundaries and faults PASS");
    return 0;
}
