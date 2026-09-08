#include "gimbal_smc.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

static int near(float a, float b, float tolerance)
{
    return isfinite(a) && isfinite(b) && fabsf(a - b) <= tolerance;
}

static gimbal_smc_input_t fresh_input(void)
{
    gimbal_smc_input_t in;
    memset(&in, 0, sizeof(in));
    in.enable = true;
    in.dt_s = 0.001f;
    return in;
}

static void test_sign_and_crossing(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    gimbal_smc_output_t out;
    gimbal_smc_input_t in = fresh_input();
    gimbal_smc_default_config(&cfg);
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in.reference_rad = 0.2f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && out.torque_nm > 0.0f && out.error_rad < 0.0f);
    in.reference_rad = -0.2f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && out.torque_nm < 0.0f);
    in.reference_rad = 0.0f;
    in.rate_rad_s = 3.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && out.torque_nm < 0.0f && out.error_rad == 0.0f);
    in.angle_rad = -0.00001f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && out.torque_nm < 0.0f); /* Brakes through target crossing. */
    in.angle_rad = 0.0f;
    in.rate_rad_s = -3.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && out.torque_nm > 0.0f);
    in.rate_rad_s = 0.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && out.torque_nm == 0.0f);
}

static void test_equivalent_control(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    gimbal_smc_output_t out;
    gimbal_smc_input_t in = fresh_input();
    gimbal_smc_default_config(&cfg);
    cfg.inertia_kg_m2 = 0.02f;
    cfg.viscous_nm_s_rad = 0.03f;
    cfg.torque_limit_nm = 10.0f;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in.angle_rad = in.reference_rad = 0.7f;
    in.rate_rad_s = in.reference_rate_rad_s = 0.8f;
    in.reference_accel_rad_s2 = 2.0f;
    in.feedforward_nm = 0.1f;
    CHECK(gimbal_smc_update(&ctrl, &in, &out) == 0u);
    CHECK(near(out.torque_nm, 0.164f, 0.000001f));
}

static void test_wrap(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    gimbal_smc_output_t out;
    gimbal_smc_input_t in = fresh_input();
    const float pi = 3.14159265358979323846f;
    gimbal_smc_default_config(&cfg);
    cfg.wrap_angle = true;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in.angle_rad = pi - 0.01f;
    in.reference_rad = -pi + 0.01f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(near(out.error_rad, -0.02f, 0.000002f) && out.torque_nm > 0.0f);
    in.angle_rad = -pi + 0.01f;
    in.reference_rad = pi - 0.01f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(near(out.error_rad, 0.02f, 0.000002f) && out.torque_nm < 0.0f);
    cfg.wrap_angle = false;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.error_rad < -6.0f && out.torque_nm > 0.0f);
}

static void check_fault(gimbal_smc_t *ctrl, gimbal_smc_input_t *in,
                        uint32_t expected)
{
    gimbal_smc_output_t out;
    CHECK(gimbal_smc_update(ctrl, in, &out) == expected);
    CHECK(!out.valid && out.torque_nm == 0.0f && out.unsaturated_torque_nm == 0.0f);
    CHECK(!ctrl->filter_initialized && ctrl->previous_torque_nm == 0.0f);
}

static void test_faults_and_recovery(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    gimbal_smc_output_t out;
    gimbal_smc_input_t in = fresh_input();
    gimbal_smc_default_config(&cfg);
    cfg.torque_slew_nm_s = 1.0f;
    cfg.rate_lpf_hz = 10.0f;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in.reference_rad = 2.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.torque_nm > 0.0f);
    in.enable = false;
    check_fault(&ctrl, &in, GIMBAL_SMC_DISABLED);
    in.enable = true;
    in.rate_rad_s = 2.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(near(out.filtered_rate_rad_s, 2.0f, 0.0f));
    CHECK(fabsf(out.torque_nm) <= 0.001001f);
    in.feedback_age_s = cfg.feedback_timeout_s + 0.001f;
    check_fault(&ctrl, &in, GIMBAL_SMC_STALE_FEEDBACK);
    in.feedback_age_s = cfg.feedback_timeout_s;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid);
    in.feedback_age_s = -1.0f;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_INPUT);
    in.feedback_age_s = 0.0f;
    in.dt_s = 0.0f;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_DT);
    in.dt_s = cfg.dt_max_s + 0.001f;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_DT);
    in.dt_s = NAN;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_DT);
    in.dt_s = cfg.dt_min_s;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid);
    in.dt_s = cfg.dt_max_s;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid);
    in.angle_rad = NAN;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_INPUT);
    in.angle_rad = 0.0f;
    in.rate_rad_s = INFINITY;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_INPUT);
    in = fresh_input();
    in.feedforward_nm = -INFINITY;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_INPUT);
    in = fresh_input();
    in.reference_accel_rad_s2 = NAN;
    check_fault(&ctrl, &in, GIMBAL_SMC_BAD_INPUT);
    in = fresh_input();
    in.angle_rad = FLT_MAX;
    check_fault(&ctrl, &in, GIMBAL_SMC_NUMERIC_FAULT);
    CHECK(gimbal_smc_update(&ctrl, NULL, &out) == GIMBAL_SMC_BAD_INPUT);
    CHECK(gimbal_smc_update(&ctrl, &in, NULL) == GIMBAL_SMC_BAD_INPUT);
    CHECK(gimbal_smc_update(NULL, &in, &out) == GIMBAL_SMC_BAD_CONFIG);
}

static void test_config_validation(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    int i;
    for (i = 0; i < 15; ++i) {
        gimbal_smc_default_config(&cfg);
        switch (i) {
        case 0: cfg.inertia_kg_m2 = 0.0f; break;
        case 1: cfg.viscous_nm_s_rad = -1.0f; break;
        case 2: cfg.lambda_per_s = NAN; break;
        case 3: cfg.reaching_per_s = -1.0f; break;
        case 4: cfg.robust_rad_s2 = INFINITY; break;
        case 5: cfg.boundary_rad_s = 0.0f; break;
        case 6: cfg.terminal_gain = -1.0f; break;
        case 7: cfg.terminal_power = 1.0f; break;
        case 8: cfg.terminal_epsilon_rad = 0.0f; break;
        case 9: cfg.torque_limit_nm = -1.0f; break;
        case 10: cfg.torque_slew_nm_s = -1.0f; break;
        case 11: cfg.rate_lpf_hz = NAN; break;
        case 12: cfg.dt_min_s = 0.0f; break;
        case 13: cfg.dt_max_s = cfg.dt_min_s * 0.5f; break;
        default: cfg.feedback_timeout_s = 0.0f; break;
        }
        CHECK(!gimbal_smc_init(&ctrl, &cfg));
        CHECK(!ctrl.initialized);
    }
    CHECK(!gimbal_smc_init(&ctrl, NULL));
    CHECK(!gimbal_smc_init(NULL, &cfg));
    gimbal_smc_default_config(&cfg);
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    CHECK(gimbal_smc_init(&ctrl, &ctrl.config)); /* Reinitialization alias. */
    CHECK(near(ctrl.config.inertia_kg_m2, cfg.inertia_kg_m2, 0.0f));
}

static void test_limits_and_filter(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    gimbal_smc_output_t out;
    gimbal_smc_input_t in = fresh_input();
    gimbal_smc_default_config(&cfg);
    cfg.torque_slew_nm_s = 2.0f;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in.reference_rad = 3.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && (out.flags & GIMBAL_SMC_AMPLITUDE_LIMIT) != 0u);
    CHECK((out.flags & GIMBAL_SMC_SLEW_LIMIT) != 0u);
    CHECK(near(out.torque_nm, 0.002f, 0.000001f));
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(near(out.torque_nm, 0.004f, 0.000001f));
    in.reference_rad = -3.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(near(out.torque_nm, 0.002f, 0.000001f));
    cfg.torque_slew_nm_s = 0.0f;
    cfg.rate_lpf_hz = 10.0f;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in = fresh_input();
    in.rate_rad_s = 3.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.filtered_rate_rad_s == 3.0f && out.torque_nm < 0.0f);
    in.rate_rad_s = 0.0f;
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.filtered_rate_rad_s < 3.0f && out.filtered_rate_rad_s > 2.0f);
    gimbal_smc_reset(&ctrl);
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.filtered_rate_rad_s == 0.0f && out.torque_nm == 0.0f);
}

/* Double-precision numerical differentiation independently checks f'(e) in
 * the torque equation rather than duplicating its analytic implementation. */
static double terminal_surface(double e, const gimbal_smc_config_t *cfg)
{
    const double delta = (double)cfg->terminal_epsilon_rad;
    return (double)cfg->lambda_per_s * e + (double)cfg->terminal_gain * e *
           pow(e * e + delta * delta, 0.5 * ((double)cfg->terminal_power - 1.0));
}

static void test_terminal_surface(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    gimbal_smc_output_t out;
    gimbal_smc_input_t in = fresh_input();
    const float errors[] = {-1.0f, -0.01f, -0.000001f, 0.0f, 0.000001f, 0.01f, 1.0f};
    size_t i;
    gimbal_smc_default_config(&cfg);
    cfg.terminal_gain = 2.0f;
    cfg.reaching_per_s = 0.0f;
    cfg.robust_rad_s2 = 0.0f;
    cfg.torque_limit_nm = 100.0f;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in.rate_rad_s = 0.7f;
    for (i = 0u; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        const double e = (double)errors[i];
        const double h = 0.0000001;
        const double derivative = (terminal_surface(e + h, &cfg) -
                                   terminal_surface(e - h, &cfg)) / (2.0 * h);
        const float expected = (float)(-(double)cfg.inertia_kg_m2 * derivative *
                                       (double)in.rate_rad_s);
        in.angle_rad = errors[i];
        gimbal_smc_update(&ctrl, &in, &out);
        CHECK(out.valid && isfinite(out.sliding_rad_s));
        CHECK(near(out.unsaturated_torque_nm, expected, 0.00001f));
        CHECK(out.torque_nm < 0.0f);
    }
    in = fresh_input();
    gimbal_smc_update(&ctrl, &in, &out);
    CHECK(out.valid && out.torque_nm == 0.0f);
}

static void test_closed_loop_disturbance(void)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t ctrl;
    gimbal_smc_output_t out;
    gimbal_smc_input_t in = fresh_input();
    float theta = 0.0f, omega = 0.0f;
    int i;
    gimbal_smc_default_config(&cfg);
    cfg.viscous_nm_s_rad = 0.02f;
    CHECK(gimbal_smc_init(&ctrl, &cfg));
    in.reference_rad = 0.4f;
    for (i = 0; i < 8000; ++i) {
        const float load = i >= 2000 ? -0.03f : 0.0f;
        float acceleration;
        in.angle_rad = theta;
        in.rate_rad_s = omega;
        gimbal_smc_update(&ctrl, &in, &out);
        CHECK(out.valid && fabsf(out.torque_nm) <= cfg.torque_limit_nm);
        acceleration = (out.torque_nm - cfg.viscous_nm_s_rad * omega + load) /
                       cfg.inertia_kg_m2;
        omega += acceleration * in.dt_s;
        theta += omega * in.dt_s;
    }
    CHECK(fabsf(theta - in.reference_rad) < 0.005f);
    CHECK(fabsf(omega) < 0.01f);
}

static void test_reference_tracker(void)
{
    const gimbal_reference_config_t cfg = {2.0f, 5.0f, 8.0f, 0.0001f, 0.01f, false};
    gimbal_reference_t ref;
    gimbal_smc_reference_t out;
    float previous_rate = 0.0f;
    int i;
    CHECK(gimbal_reference_init(&ref, &cfg, 0.0f));
    for (i = 0; i < 10000; ++i) {
        const float target = i < 3000 ? 1.0f : -0.5f;
        CHECK(gimbal_reference_step(&ref, target, 0.001f, &out));
        CHECK(fabsf(out.rate_rad_s) <= cfg.max_rate_rad_s);
        CHECK(fabsf(out.accel_rad_s2) <= cfg.max_accel_rad_s2 + 0.0002f);
        CHECK(fabsf(out.rate_rad_s - previous_rate) <= 0.005001f);
        previous_rate = out.rate_rad_s;
    }
    CHECK(near(out.angle_rad, -0.5f, 0.0001f));
    CHECK(fabsf(out.rate_rad_s) < 0.001f);
    {
        const float last_angle = ref.value.angle_rad;
        const float last_rate = ref.value.rate_rad_s;
        CHECK(!gimbal_reference_step(&ref, NAN, 0.001f, &out));
        CHECK(out.angle_rad == 0.0f && out.rate_rad_s == 0.0f);
        CHECK(ref.value.angle_rad == last_angle && ref.value.rate_rad_s == last_rate);
        CHECK(!gimbal_reference_step(&ref, 0.0f, 0.1f, &out));
        CHECK(ref.value.angle_rad == last_angle);
    }
    CHECK(gimbal_reference_reset(&ref, 0.4f));
    CHECK(ref.value.angle_rad == 0.4f && ref.value.rate_rad_s == 0.0f);
    {
        gimbal_reference_config_t wrapped = cfg;
        wrapped.wrap_angle = true;
        CHECK(gimbal_reference_init(&ref, &wrapped, 3.13f));
        CHECK(gimbal_reference_step(&ref, -3.13f, 0.001f, &out));
        CHECK(out.rate_rad_s > 0.0f && out.angle_rad >= 3.13f);
        wrapped.max_accel_rad_s2 = NAN;
        CHECK(!gimbal_reference_init(&ref, &wrapped, 0.0f));
    }
}

int main(void)
{
    test_sign_and_crossing();
    test_equivalent_control();
    test_wrap();
    test_faults_and_recovery();
    test_config_validation();
    test_limits_and_filter();
    test_terminal_surface();
    test_closed_loop_disturbance();
    test_reference_tracker();
    puts("SMC tests passed (9 groups)");
    return EXIT_SUCCESS;
}
