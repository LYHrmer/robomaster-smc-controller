#include "gimbal_rls.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); \
    exit(EXIT_FAILURE); } } while (0)

static int near(double a, double b, double tolerance)
{
    return isfinite(a) && isfinite(b) && fabs(a - b) <= tolerance;
}

static gimbal_rls_config_t configuration(unsigned int count)
{
    gimbal_rls_config_t c;
    unsigned int i;
    gimbal_rls_default_config(&c, (uint8_t)count);
    c.excitation_window = 16u;
    c.min_updates = 3u;
    c.covariance_initial = 20.0f;
    c.covariance_limit = 10000.0f;
    c.forgetting_time_s = 0.0f;
    c.dt_min_s = 0.001f;
    c.dt_max_s = 0.1f;
    c.min_feature_energy = 0.000001f;
    c.min_excitation_pivot = 0.00001f;
    c.innovation_rms_limit = 100.0f;
    c.output_scale = 1.0f;
    for (i = 0u; i < count; ++i) {
        c.initial[i] = 0.0f;
        c.minimum[i] = -20.0f;
        c.maximum[i] = 20.0f;
        c.feature_scale[i] = 1.0f;
    }
    return c;
}

static gimbal_rls_input_t observation(unsigned int index,
                                      const gimbal_rls_config_t *c)
{
    gimbal_rls_input_t in;
    memset(&in, 0, sizeof(in));
    in.enabled = in.valid = true;
    in.dt_s = 0.01f;
    in.features[0] = c->feature_scale[0] *
        (float)(sin(0.37 * index) + 0.2 * cos(0.11 * index));
    if (c->parameter_count > 1u)
        in.features[1] = c->feature_scale[1] *
            (float)(cos(0.23 * index) + 0.1 * sin(0.83 * index));
    if (c->parameter_count > 2u)
        in.features[2] = c->feature_scale[2] *
            (float)(sin(0.61 * index) + 0.3 * cos(0.41 * index));
    return in;
}

static void unchanged_model(const gimbal_rls_t *a, const gimbal_rls_t *b)
{
    CHECK(memcmp(a->theta, b->theta, sizeof(a->theta)) == 0);
    CHECK(memcmp(a->covariance, b->covariance, sizeof(a->covariance)) == 0);
    CHECK(a->accepted_updates == b->accepted_updates);
}

/* Independent batch normal-equation solve in double precision. This oracle
 * never repeats the recursive gain or Joseph update under test. */
static void solve3(const double matrix[3][3], const double rhs[3], double answer[3])
{
    double augmented[3][4];
    unsigned int i, j, k;
    for (i = 0u; i < 3u; ++i) {
        for (j = 0u; j < 3u; ++j) augmented[i][j] = matrix[i][j];
        augmented[i][3] = rhs[i];
    }
    for (i = 0u; i < 3u; ++i) {
        unsigned int pivot = i;
        double divisor;
        for (j = i + 1u; j < 3u; ++j)
            if (fabs(augmented[j][i]) > fabs(augmented[pivot][i])) pivot = j;
        CHECK(fabs(augmented[pivot][i]) > 1e-12);
        for (k = i; k < 4u; ++k) {
            const double temporary = augmented[i][k];
            augmented[i][k] = augmented[pivot][k];
            augmented[pivot][k] = temporary;
        }
        divisor = augmented[i][i];
        for (k = i; k < 4u; ++k) augmented[i][k] /= divisor;
        for (j = 0u; j < 3u; ++j) {
            double multiplier;
            if (j == i) continue;
            multiplier = augmented[j][i];
            for (k = i; k < 4u; ++k) augmented[j][k] -= multiplier * augmented[i][k];
        }
    }
    for (i = 0u; i < 3u; ++i) answer[i] = augmented[i][3];
}

static void test_batch_ridge_and_units(void)
{
    const double truth[3] = {2.4, 0.18, -0.025};
    double gram[3][3] = {{0.0}}, rhs[3] = {0.0}, answer[3];
    gimbal_rls_config_t c = configuration(3u);
    gimbal_rls_t r;
    gimbal_rls_output_t out;
    unsigned int i, j, k, accepted = 0u;
    c.feature_scale[0] = 0.07f;
    c.feature_scale[1] = 3.0f;
    c.feature_scale[2] = 11.0f;
    c.output_scale = 2.3f;
    c.initial[0] = 0.1f; c.initial[1] = 0.2f; c.initial[2] = -0.1f;
    CHECK(gimbal_rls_init(&r, &c));
    for (j = 0u; j < 3u; ++j) {
        gram[j][j] = 1.0 / c.covariance_initial;
        rhs[j] = c.initial[j] * (double)c.feature_scale[j] /
            c.output_scale / c.covariance_initial;
    }
    for (i = 0u; i < 600u; ++i) {
        gimbal_rls_input_t in = observation(i, &c);
        gimbal_rls_output_t prior;
        double target = 0.004 * sin(1.77 * i), prediction = 0.0;
        CHECK(gimbal_rls_snapshot(&r, &prior));
        CHECK(!prior.ready && !prior.updated);
        for (j = 0u; j < 3u; ++j) {
            target += truth[j] * in.features[j];
            prediction += prior.parameters[j] * (double)in.features[j];
        }
        in.target = (float)target;
        gimbal_rls_update(&r, &in, &out);
        if (!out.updated) { CHECK(!out.ready); continue; }
        CHECK(!(out.flags & GIMBAL_RLS_PROJECTED));
        CHECK(near(out.prediction, prediction, 0.00001));
        CHECK(near(out.residual, in.target - prediction, 0.00001));
        ++accepted;
        for (j = 0u; j < 3u; ++j) {
            const double xj = (double)in.features[j] / c.feature_scale[j];
            rhs[j] += xj * ((double)in.target / c.output_scale);
            for (k = 0u; k < 3u; ++k)
                gram[j][k] += xj * ((double)in.features[k] / c.feature_scale[k]);
        }
    }
    CHECK(accepted > 500u && out.accepted_updates == accepted && out.ready);
    solve3((const double (*)[3])gram, rhs, answer);
    for (j = 0u; j < 3u; ++j) {
        double unit[3] = {0.0}, inverse_column[3];
        const double physical = answer[j] * c.output_scale / c.feature_scale[j];
        /* Float recursive accumulation versus independent double batch solve. */
        CHECK(near(out.parameters[j], physical, 0.0001));
        unit[j] = 1.0;
        solve3((const double (*)[3])gram, unit, inverse_column);
        CHECK(near(out.covariance_diag[j], inverse_column[j], 0.00001));
    }
}

static void test_forgetting_and_instance_isolation(void)
{
    gimbal_rls_config_t c = configuration(2u), fixed_config = c;
    gimbal_rls_t adaptive, fixed, untouched, untouched_before;
    gimbal_rls_output_t adapting, averaging;
    unsigned int i;
    double adaptive_error, fixed_error;
    c.forgetting_time_s = 0.20f;
    CHECK(gimbal_rls_init(&adaptive, &c));
    CHECK(gimbal_rls_init(&fixed, &fixed_config));
    CHECK(gimbal_rls_init(&untouched, &c));
    untouched_before = untouched;
    for (i = 0u; i < 1600u; ++i) {
        gimbal_rls_input_t in = observation(i, &c);
        const float p0 = i < 800u ? 0.4f : 1.2f;
        const float p1 = i < 800u ? 0.7f : -0.3f;
        in.target = p0 * in.features[0] + p1 * in.features[1];
        gimbal_rls_update(&adaptive, &in, &adapting);
        gimbal_rls_update(&fixed, &in, &averaging);
    }
    CHECK(adapting.ready && averaging.ready);
    adaptive_error = hypot(adapting.parameters[0] - 1.2, adapting.parameters[1] + 0.3);
    fixed_error = hypot(averaging.parameters[0] - 1.2, averaging.parameters[1] + 0.3);
    CHECK(adaptive_error < 0.02 && adaptive_error < 0.1 * fixed_error);
    CHECK(memcmp(&untouched, &untouched_before, sizeof(untouched)) == 0);
}

static void test_excitation_freezes_model_and_clears_readiness(void)
{
    gimbal_rls_config_t c = configuration(2u);
    gimbal_rls_t r, frozen;
    gimbal_rls_output_t out;
    unsigned int i, kind;
    c.forgetting_time_s = 0.1f;
    CHECK(gimbal_rls_init(&r, &c));
    for (i = 0u; i < 100u; ++i) {
        gimbal_rls_input_t in = observation(i, &c);
        in.target = 0.4f * in.features[0] + 0.7f * in.features[1];
        gimbal_rls_update(&r, &in, &out);
    }
    CHECK(out.ready);
    for (kind = 0u; kind < 3u; ++kind) {
        gimbal_rls_clear_observations(&r);
        frozen = r;
        for (i = 0u; i < 100u; ++i) {
            gimbal_rls_input_t in;
            memset(&in, 0, sizeof(in));
            in.enabled = in.valid = true;
            in.dt_s = 0.01f;
            if (kind == 1u) { in.features[0] = 1.0f; in.features[1] = 2.0f; }
            if (kind == 2u) { in.features[0] = sinf((float)i); in.features[1] = 2.0f * in.features[0]; }
            in.target = 0.9f;
            gimbal_rls_update(&r, &in, &out);
            CHECK(!out.updated && !out.ready);
            CHECK(out.flags & GIMBAL_RLS_LOW_EXCITATION);
            CHECK(!out.innovation_rms_valid);
            unchanged_model(&r, &frozen);
        }
    }
}

static void test_projection_reset_and_innovation_gate(void)
{
    gimbal_rls_config_t c = configuration(1u);
    gimbal_rls_t r;
    gimbal_rls_input_t in;
    gimbal_rls_output_t out;
    unsigned int i;
    c.excitation_window = 4u;
    c.initial[0] = 0.2f; c.minimum[0] = 0.0f; c.maximum[0] = 1.0f;
    c.feature_scale[0] = 2.0f; c.output_scale = 3.0f;
    c.innovation_rms_limit = 0.01f;
    CHECK(gimbal_rls_init(&r, &c));
    memset(&in, 0, sizeof(in));
    in.enabled = in.valid = true; in.dt_s = 0.01f;
    in.features[0] = 2.0f; in.target = 20.0f;
    for (i = 0u; i < 12u; ++i) gimbal_rls_update(&r, &in, &out);
    CHECK(out.updated && !out.ready && (out.flags & GIMBAL_RLS_PROJECTED));
    CHECK(near(out.parameters[0], 1.0, 0.000001));
    CHECK(near(r.theta[0], 2.0 / 3.0, 0.000001));
    CHECK(out.consecutive_updates == 0u);
    gimbal_rls_reset(&r);
    CHECK(gimbal_rls_snapshot(&r, &out));
    CHECK(near(out.parameters[0], 0.2, 0.000001));
    CHECK(out.covariance_diag[0] == c.covariance_initial);
    CHECK(out.accepted_updates == 0u && !out.ready && !out.innovation_rms_valid);
    /* Inconsistent noisy targets stay within bounds but fail the residual gate. */
    c.minimum[0] = -10.0f; c.maximum[0] = 10.0f;
    CHECK(gimbal_rls_init(&r, &c));
    for (i = 0u; i < 80u; ++i) {
        in.target = (i & 1u) ? 2.0f : -2.0f;
        gimbal_rls_update(&r, &in, &out);
    }
    CHECK(out.updated && !out.ready && out.innovation_rms_valid);
    CHECK(out.innovation_rms > c.innovation_rms_limit);
}

static void test_rejected_input_and_covariance_are_transactional(void)
{
    const float invalid[] = {NAN, INFINITY, -INFINITY};
    gimbal_rls_config_t c = configuration(2u);
    gimbal_rls_t r, before;
    gimbal_rls_output_t out;
    unsigned int i, kind;
    CHECK(gimbal_rls_init(&r, &c));
    for (i = 0u; i < 100u; ++i) {
        gimbal_rls_input_t in = observation(i, &c);
        in.target = in.features[0] + in.features[1];
        gimbal_rls_update(&r, &in, &out);
    }
    CHECK(out.ready);
    for (kind = 0u; kind < 9u; ++kind) {
        gimbal_rls_input_t in = observation(20u, &c);
        before = r;
        if (kind < 3u) in.features[0] = invalid[kind];
        else if (kind < 6u) in.target = invalid[kind - 3u];
        else if (kind == 6u) in.dt_s = 0.0f;
        else if (kind == 7u) in.dt_s = 0.2f;
        else in.dt_s = NAN;
        CHECK(gimbal_rls_update(&r, &in, &out) != 0u);
        CHECK(!out.updated && !out.ready && !out.innovation_rms_valid);
        unchanged_model(&r, &before);
    }
    for (kind = 0u; kind < 2u; ++kind) {
        gimbal_rls_input_t in = observation(20u, &c);
        before = r;
        if (kind == 0u) in.enabled = false; else in.valid = false;
        CHECK(gimbal_rls_update(&r, &in, &out) != 0u);
        CHECK(!out.ready && !out.updated);
        unchanged_model(&r, &before);
    }
    /* Deliberate corruption is test-only. Production never edits private state. */
    for (kind = 0u; kind < 4u; ++kind) {
        gimbal_rls_input_t in = observation(20u, &c);
        CHECK(gimbal_rls_init(&r, &c));
        if (kind == 0u) r.covariance[0][0] = -1.0f;
        if (kind == 1u) r.covariance[0][0] = c.covariance_limit * 2.0f;
        if (kind == 2u) r.covariance[0][1] = r.covariance[1][0] = 40.0f;
        if (kind == 3u) r.covariance[0][0] = NAN;
        before = r;
        CHECK(gimbal_rls_update(&r, &in, &out) != 0u);
        CHECK(!out.ready && !out.updated);
        unchanged_model(&r, &before);
    }
    CHECK(gimbal_rls_init(&r, &c));
    r.config.forgetting_time_s = NAN;
    before = r;
    {
        gimbal_rls_input_t in = observation(20u, &c);
        CHECK(gimbal_rls_update(&r, &in, &out) != 0u);
        CHECK(!out.ready && !out.updated);
        unchanged_model(&r, &before);
    }
}

static void test_bad_configuration(void)
{
    gimbal_rls_t r;
    gimbal_rls_config_t c;
    unsigned int kind;
    for (kind = 0u; kind < 17u; ++kind) {
        c = configuration(2u);
        switch (kind) {
        case 0: c.parameter_count = 0u; break;
        case 1: c.parameter_count = 6u; break;
        case 2: c.excitation_window = 1u; break;
        case 3: c.excitation_window = 33u; break;
        case 4: c.min_updates = 0u; break;
        case 5: c.feature_scale[0] = 0.0f; break;
        case 6: c.output_scale = NAN; break;
        case 7: c.covariance_initial = 0.0f; break;
        case 8: c.covariance_limit = c.covariance_initial * 0.5f; break;
        case 9: c.forgetting_time_s = -1.0f; break;
        case 10: c.minimum[0] = c.maximum[0] + 1.0f; break;
        case 11: c.initial[0] = c.maximum[0] + 1.0f; break;
        case 12: c.min_feature_energy = 0.0f; break;
        case 13: c.min_excitation_pivot = 1.1f; break;
        case 14: c.innovation_rms_limit = INFINITY; break;
        case 15: c.dt_max_s = c.dt_min_s * 0.5f; break;
        default: c.initial[0] = FLT_MAX; break;
        }
        CHECK(!gimbal_rls_init(&r, &c) && !r.initialized);
    }
    c = configuration(2u);
    CHECK(!gimbal_rls_init(NULL, &c));
    CHECK(!gimbal_rls_init(&r, NULL));
}

int main(void)
{
    test_batch_ridge_and_units();
    test_forgetting_and_instance_isolation();
    test_excitation_freezes_model_and_clears_readiness();
    test_projection_reset_and_innovation_gate();
    test_rejected_input_and_covariance_are_transactional();
    test_bad_configuration();
    puts("RLS regression, independent batch oracle, gates and faults: PASS");
    return 0;
}
