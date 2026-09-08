#include "gimbal_smc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define GIMBAL_TWO_PI_F 6.2831853071795864769f

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float angle_error(float angle, float target, bool wrap)
{
    if (wrap) {
        /* Reducing each operand first also avoids overflow in subtraction. */
        return remainderf(remainderf(angle, GIMBAL_TWO_PI_F) -
                          remainderf(target, GIMBAL_TWO_PI_F), GIMBAL_TWO_PI_F);
    }
    return angle - target;
}

static bool config_valid(const gimbal_smc_config_t *c)
{
    return c != NULL &&
        isfinite(c->inertia_kg_m2) && c->inertia_kg_m2 > 0.0f &&
        isfinite(c->viscous_nm_s_rad) && c->viscous_nm_s_rad >= 0.0f &&
        isfinite(c->lambda_per_s) && c->lambda_per_s > 0.0f &&
        isfinite(c->reaching_per_s) && c->reaching_per_s >= 0.0f &&
        isfinite(c->robust_rad_s2) && c->robust_rad_s2 >= 0.0f &&
        isfinite(c->boundary_rad_s) && c->boundary_rad_s > 0.0f &&
        isfinite(c->terminal_gain) && c->terminal_gain >= 0.0f &&
        isfinite(c->terminal_power) && c->terminal_power > 0.0f &&
        c->terminal_power < 1.0f &&
        isfinite(c->terminal_epsilon_rad) && c->terminal_epsilon_rad > 0.0f &&
        isfinite(c->torque_limit_nm) && c->torque_limit_nm > 0.0f &&
        isfinite(c->torque_slew_nm_s) && c->torque_slew_nm_s >= 0.0f &&
        isfinite(c->rate_lpf_hz) && c->rate_lpf_hz >= 0.0f &&
        isfinite(c->dt_min_s) && c->dt_min_s > 0.0f &&
        isfinite(c->dt_max_s) && c->dt_max_s >= c->dt_min_s &&
        isfinite(c->feedback_timeout_s) && c->feedback_timeout_s > 0.0f;
}

static bool input_valid(const gimbal_smc_input_t *in)
{
    return isfinite(in->angle_rad) && isfinite(in->rate_rad_s) &&
        isfinite(in->reference_rad) && isfinite(in->reference_rate_rad_s) &&
        isfinite(in->reference_accel_rad_s2) && isfinite(in->feedforward_nm) &&
        isfinite(in->feedback_age_s) && in->feedback_age_s >= 0.0f;
}

void gimbal_smc_default_config(gimbal_smc_config_t *c)
{
    if (c == NULL) {
        return;
    }
    c->inertia_kg_m2 = 0.01f;
    c->viscous_nm_s_rad = 0.0f;
    c->lambda_per_s = 12.0f;
    c->reaching_per_s = 10.0f;
    c->robust_rad_s2 = 20.0f;
    c->boundary_rad_s = 0.1f;
    c->terminal_gain = 0.0f;
    c->terminal_power = 0.5f;
    c->terminal_epsilon_rad = 0.01f;
    c->torque_limit_nm = 0.5f;
    c->torque_slew_nm_s = 0.0f;
    c->rate_lpf_hz = 0.0f;
    c->dt_min_s = 0.0001f;
    c->dt_max_s = 0.01f;
    c->feedback_timeout_s = 0.02f;
    c->wrap_angle = false;
}

void gimbal_smc_reset(gimbal_smc_t *controller)
{
    if (controller == NULL) {
        return;
    }
    controller->filtered_rate_rad_s = 0.0f;
    controller->previous_torque_nm = 0.0f;
    controller->filter_initialized = false;
}

bool gimbal_smc_init(gimbal_smc_t *controller, const gimbal_smc_config_t *config)
{
    gimbal_smc_config_t copy;
    if (controller == NULL) {
        return false;
    }
    if (!config_valid(config)) {
        memset(controller, 0, sizeof(*controller));
        return false;
    }
    copy = *config;
    memset(controller, 0, sizeof(*controller));
    controller->config = copy;
    controller->initialized = true;
    return true;
}

static uint32_t fault(gimbal_smc_t *controller, gimbal_smc_output_t *out,
                      uint32_t flags)
{
    gimbal_smc_reset(controller);
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        out->flags = flags;
    }
    return flags;
}

uint32_t gimbal_smc_update(gimbal_smc_t *controller,
                          const gimbal_smc_input_t *in,
                          gimbal_smc_output_t *out)
{
    const gimbal_smc_config_t *c;
    float rate, e, de, surface, slope, s, sat, raw, limited;
    uint32_t flags = 0u;

    if (out == NULL || in == NULL) {
        return fault(controller, out, GIMBAL_SMC_BAD_INPUT);
    }
    memset(out, 0, sizeof(*out));
    if (controller == NULL || !controller->initialized ||
        !config_valid(&controller->config)) {
        return fault(controller, out, GIMBAL_SMC_BAD_CONFIG);
    }
    c = &controller->config;
    if (!in->enable) {
        return fault(controller, out, GIMBAL_SMC_DISABLED);
    }
    if (!input_valid(in)) {
        return fault(controller, out, GIMBAL_SMC_BAD_INPUT);
    }
    if (!isfinite(in->dt_s) || in->dt_s < c->dt_min_s || in->dt_s > c->dt_max_s) {
        return fault(controller, out, GIMBAL_SMC_BAD_DT);
    }
    if (in->feedback_age_s > c->feedback_timeout_s) {
        return fault(controller, out, GIMBAL_SMC_STALE_FEEDBACK);
    }

    rate = in->rate_rad_s;
    if (c->rate_lpf_hz > 0.0f && controller->filter_initialized) {
        /* Backward-Euler first-order LPF: alpha = dt / (RC + dt).
         * Seed with the first actual gyro reading, never an artificial zero. */
        const float rc = (1.0f / c->rate_lpf_hz) / GIMBAL_TWO_PI_F;
        const float alpha = in->dt_s / (rc + in->dt_s);
        rate = controller->filtered_rate_rad_s +
               alpha * (rate - controller->filtered_rate_rad_s);
    }
    e = angle_error(in->angle_rad, in->reference_rad, c->wrap_angle);
    de = rate - in->reference_rate_rad_s;
    surface = c->lambda_per_s * e;
    slope = c->lambda_per_s;
    if (c->terminal_gain > 0.0f) {
        const float e2 = e * e;
        const float delta2 = c->terminal_epsilon_rad * c->terminal_epsilon_rad;
        const float base = e2 + delta2;
        /* f(e)=lambda*e+alpha*e*(e^2+delta^2)^((r-1)/2).
         * This exact derivative is finite at zero for practical delta > 0. */
        surface += c->terminal_gain * e *
                   powf(base, 0.5f * (c->terminal_power - 1.0f));
        slope += c->terminal_gain *
                 powf(base, 0.5f * (c->terminal_power - 3.0f)) *
                 (delta2 + c->terminal_power * e2);
    }
    s = de + surface;
    if (!isfinite(rate) || !isfinite(e) || !isfinite(de) ||
        !isfinite(surface) || !isfinite(slope) || !isfinite(s)) {
        return fault(controller, out, GIMBAL_SMC_NUMERIC_FAULT);
    }
    sat = s >= c->boundary_rad_s ? 1.0f :
          (s <= -c->boundary_rad_s ? -1.0f : s / c->boundary_rad_s);
    raw = c->inertia_kg_m2 * (in->reference_accel_rad_s2 - slope * de -
          c->reaching_per_s * s - c->robust_rad_s2 * sat) +
          c->viscous_nm_s_rad * rate + in->feedforward_nm;
    if (!isfinite(raw) || !isfinite(controller->previous_torque_nm)) {
        return fault(controller, out, GIMBAL_SMC_NUMERIC_FAULT);
    }
    limited = clampf(raw, -c->torque_limit_nm, c->torque_limit_nm);
    if (limited != raw) {
        flags |= GIMBAL_SMC_AMPLITUDE_LIMIT;
    }
    if (c->torque_slew_nm_s > 0.0f) {
        const float previous = clampf(controller->previous_torque_nm,
                                       -c->torque_limit_nm, c->torque_limit_nm);
        const float max_step = c->torque_slew_nm_s * in->dt_s;
        const float slew_limited = clampf(limited, previous - max_step,
                                          previous + max_step);
        if (slew_limited != limited) {
            flags |= GIMBAL_SMC_SLEW_LIMIT;
        }
        limited = slew_limited;
    }
    /* The final clamp also enforces a reduced limit after live config changes.
     * Use init to change config; external state edits are not synchronization. */
    limited = clampf(limited, -c->torque_limit_nm, c->torque_limit_nm);
    if (!isfinite(limited)) {
        return fault(controller, out, GIMBAL_SMC_NUMERIC_FAULT);
    }
    controller->filtered_rate_rad_s = rate;
    controller->filter_initialized = true;
    controller->previous_torque_nm = limited;
    out->torque_nm = limited;
    out->unsaturated_torque_nm = raw;
    out->error_rad = e;
    out->rate_error_rad_s = de;
    out->sliding_rad_s = s;
    out->filtered_rate_rad_s = rate;
    out->flags = flags;
    out->valid = true;
    return flags;
}

static bool reference_config_valid(const gimbal_reference_config_t *c)
{
    return c != NULL && isfinite(c->max_rate_rad_s) && c->max_rate_rad_s > 0.0f &&
        isfinite(c->max_accel_rad_s2) && c->max_accel_rad_s2 > 0.0f &&
        isfinite(c->position_gain_per_s) && c->position_gain_per_s > 0.0f &&
        isfinite(c->dt_min_s) && c->dt_min_s > 0.0f &&
        isfinite(c->dt_max_s) && c->dt_max_s >= c->dt_min_s;
}

bool gimbal_reference_init(gimbal_reference_t *reference,
                          const gimbal_reference_config_t *config,
                          float initial_angle_rad)
{
    gimbal_reference_config_t copy;
    if (reference == NULL) {
        return false;
    }
    if (!reference_config_valid(config) || !isfinite(initial_angle_rad)) {
        memset(reference, 0, sizeof(*reference));
        return false;
    }
    copy = *config;
    memset(reference, 0, sizeof(*reference));
    reference->config = copy;
    reference->value.angle_rad = initial_angle_rad;
    reference->initialized = true;
    return true;
}

bool gimbal_reference_reset(gimbal_reference_t *reference, float angle_rad)
{
    if (reference == NULL || !reference->initialized || !isfinite(angle_rad)) {
        return false;
    }
    reference->value.angle_rad = angle_rad;
    reference->value.rate_rad_s = 0.0f;
    reference->value.accel_rad_s2 = 0.0f;
    return true;
}

bool gimbal_reference_step(gimbal_reference_t *reference, float target_angle_rad,
                          float dt_s, gimbal_smc_reference_t *output)
{
    const gimbal_reference_config_t *c;
    gimbal_smc_reference_t next;
    float error, desired_rate, desired_accel;
    if (output == NULL) {
        return false;
    }
    /* output must not alias reference->value: a failed step preserves state. */
    if (reference != NULL && output == &reference->value) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if (reference == NULL || !reference->initialized ||
        !reference_config_valid(&reference->config) ||
        !isfinite(target_angle_rad) || !isfinite(dt_s)) {
        return false;
    }
    c = &reference->config;
    if (dt_s < c->dt_min_s || dt_s > c->dt_max_s ||
        !isfinite(reference->value.angle_rad) || !isfinite(reference->value.rate_rad_s)) {
        return false;
    }
    error = angle_error(target_angle_rad, reference->value.angle_rad, c->wrap_angle);
    desired_rate = c->position_gain_per_s * error;
    if (!isfinite(error) || !isfinite(desired_rate)) {
        return false;
    }
    desired_rate = clampf(desired_rate, -c->max_rate_rad_s, c->max_rate_rad_s);
    desired_accel = (desired_rate - reference->value.rate_rad_s) / dt_s;
    if (!isfinite(desired_accel)) {
        return false;
    }
    next.accel_rad_s2 = clampf(desired_accel, -c->max_accel_rad_s2, c->max_accel_rad_s2);
    next.rate_rad_s = clampf(reference->value.rate_rad_s + next.accel_rad_s2 * dt_s,
                             -c->max_rate_rad_s, c->max_rate_rad_s);
    /* Report the actual finite difference of rate if rounding or clamp acted. */
    next.accel_rad_s2 = (next.rate_rad_s - reference->value.rate_rad_s) / dt_s;
    next.angle_rad = reference->value.angle_rad +
                     0.5f * (reference->value.rate_rad_s + next.rate_rad_s) * dt_s;
    if (!isfinite(next.angle_rad) || !isfinite(next.rate_rad_s) ||
        !isfinite(next.accel_rad_s2)) {
        return false;
    }
    reference->value = next;
    *output = next;
    return true;
}
