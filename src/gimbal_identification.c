#include "gimbal_identification.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "gimbal_identification requires finite-value checks; disable fast-math and finite-math-only"
#endif

#define MICROSECONDS_PER_SECOND_F 1000000.0f
#define SECONDS_PER_MICROSECOND_F 0.000001f

static bool positive_finite(float value)
{
    return isfinite(value) && value > 0.0f;
}

static bool config_valid(const gimbal_identification_config_t *config)
{
    float span;
    uint8_t count;
    if (config == NULL ||
        (unsigned)config->axis > (unsigned)GIMBAL_IDENTIFICATION_PITCH ||
        !positive_finite(config->window_s) ||
        !positive_finite(config->max_gap_s) ||
        !positive_finite(config->feedback_timeout_s) ||
        !positive_finite(config->friction_velocity_rad_s) ||
        config->window_s < SECONDS_PER_MICROSECOND_F ||
        config->max_gap_s < SECONDS_PER_MICROSECOND_F)
        return false;
    /* Leave the entire possible window inside the unambiguous half of the
     * uint32 clock range. This also bounds the float-to-integer conversions.
     */
    span = config->window_s + config->max_gap_s;
    if (!isfinite(span) || span >= 2147.0f)
        return false;
    count = config->axis == GIMBAL_IDENTIFICATION_YAW ? 3u : 5u;
    if (config->rls.parameter_count != count ||
        !positive_finite(config->rls.minimum[0]) ||
        !isfinite(config->rls.minimum[1]) || config->rls.minimum[1] < 0.0f ||
        !isfinite(config->rls.minimum[2]) || config->rls.minimum[2] < 0.0f ||
        !positive_finite(config->rls.dt_min_s) ||
        !positive_finite(config->rls.dt_max_s) ||
        config->rls.dt_min_s > config->window_s ||
        config->rls.dt_max_s < span)
        return false;
    return true;
}

static void clear_window(gimbal_identification_t *identification)
{
    memset(&identification->previous, 0, sizeof(identification->previous));
    identification->start_angle_rad = 0.0f;
    identification->start_rate_rad_s = 0.0f;
    identification->integral_torque = 0.0f;
    identification->integral_friction = 0.0f;
    identification->integral_gravity_sin = 0.0f;
    identification->integral_gravity_cos = 0.0f;
    identification->window_start_us = 0u;
    identification->window_active = false;
}

static void break_observations(gimbal_identification_t *identification)
{
    clear_window(identification);
    gimbal_rls_clear_observations(&identification->estimator);
    (void)gimbal_rls_snapshot(&identification->estimator,
                             &identification->diagnostics);
}

static void anchor(gimbal_identification_t *identification,
                   const gimbal_identification_sample_t *sample)
{
    clear_window(identification);
    identification->previous = *sample;
    identification->start_angle_rad = sample->angle_rad;
    identification->start_rate_rad_s = sample->rate_rad_s;
    identification->window_start_us = sample->timestamp_us;
    identification->window_active = true;
}

static uint32_t finish(const gimbal_identification_t *identification,
                       gimbal_identification_output_t *output,
                       uint32_t flags, float window_duration_s)
{
    if (output == NULL) return flags;
    memset(output, 0, sizeof(*output));
    output->flags = flags;
    output->window_duration_s = window_duration_s;
    if (identification == NULL || !identification->initialized)
        return flags;
    output->rls = identification->diagnostics;
    output->model.inertia_kg_m2 = output->rls.parameters[0];
    output->model.viscous_nm_s_rad = output->rls.parameters[1];
    output->model.coulomb_nm = output->rls.parameters[2];
    output->model.friction_velocity_rad_s =
        identification->config.friction_velocity_rad_s;
    if (identification->config.axis == GIMBAL_IDENTIFICATION_PITCH) {
        output->model.gravity_sin_nm = output->rls.parameters[3];
        output->model.gravity_cos_nm = output->rls.parameters[4];
    }
    output->window_updated = output->rls.updated;
    output->ready = output->rls.updated && output->rls.ready;
    return flags;
}

void gimbal_identification_default_config(gimbal_identification_config_t *config,
                                          gimbal_identification_axis_t axis)
{
    uint8_t count;
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->axis = axis;
    config->window_s = 0.05f;
    config->max_gap_s = 0.02f;
    config->feedback_timeout_s = 0.02f;
    config->friction_velocity_rad_s = 0.08f;
    count = axis == GIMBAL_IDENTIFICATION_YAW ? 3u : 5u;
    gimbal_rls_default_config(&config->rls, count);
    config->rls.initial[0] = 0.01f;
    config->rls.initial[1] = 0.005f;
    config->rls.initial[2] = 0.02f;
    config->rls.minimum[0] = 0.000001f;
    config->rls.minimum[1] = 0.0f;
    config->rls.minimum[2] = 0.0f;
    config->rls.maximum[0] = 1.0f;
    config->rls.maximum[1] = 5.0f;
    config->rls.maximum[2] = 5.0f;
    config->rls.feature_scale[0] = 10.0f;
    if (axis == GIMBAL_IDENTIFICATION_PITCH) {
        config->rls.initial[3] = 0.0f;
        config->rls.initial[4] = 0.1f;
        config->rls.minimum[3] = -5.0f;
        config->rls.minimum[4] = -5.0f;
        config->rls.maximum[3] = 5.0f;
        config->rls.maximum[4] = 5.0f;
    }
    config->rls.dt_min_s = 0.000001f;
    config->rls.dt_max_s = 0.1f;
}

bool gimbal_identification_init(gimbal_identification_t *identification,
                                const gimbal_identification_config_t *config)
{
    gimbal_identification_config_t copy;
    if (identification == NULL) return false;
    if (!config_valid(config)) {
        memset(identification, 0, sizeof(*identification));
        return false;
    }
    copy = *config; /* Permit reinitialization with &identification->config. */
    memset(identification, 0, sizeof(*identification));
    identification->config = copy;
    identification->window_us =
        (uint32_t)ceilf(copy.window_s * MICROSECONDS_PER_SECOND_F);
    identification->max_gap_us =
        (uint32_t)floorf(copy.max_gap_s * MICROSECONDS_PER_SECOND_F);
    if (identification->window_us == 0u || identification->max_gap_us == 0u ||
        !gimbal_rls_init(&identification->estimator, &copy.rls) ||
        !gimbal_rls_snapshot(&identification->estimator,
                             &identification->diagnostics)) {
        memset(identification, 0, sizeof(*identification));
        return false;
    }
    identification->initialized = true;
    return true;
}

void gimbal_identification_reset(gimbal_identification_t *identification)
{
    if (identification == NULL || !identification->initialized) return;
    clear_window(identification);
    identification->last_timestamp_us = 0u;
    identification->have_timestamp = false;
    gimbal_rls_reset(&identification->estimator);
    (void)gimbal_rls_snapshot(&identification->estimator,
                             &identification->diagnostics);
}

void gimbal_identification_clear_observations(gimbal_identification_t *identification)
{
    if (identification == NULL || !identification->initialized) return;
    break_observations(identification);
    identification->last_timestamp_us = 0u;
    identification->have_timestamp = false;
}

static uint32_t qualify(const gimbal_identification_config_t *config,
                        const gimbal_identification_sample_t *sample)
{
    uint32_t flags = 0u;
    if (!sample->enabled) flags |= GIMBAL_IDENTIFICATION_DISABLED;
    if (!sample->valid || !isfinite(sample->angle_rad) ||
        !isfinite(sample->rate_rad_s) || !isfinite(sample->gravity_angle_rad) ||
        !isfinite(sample->torque_nm) || !isfinite(sample->feedback_age_s) ||
        sample->feedback_age_s < 0.0f)
        flags |= GIMBAL_IDENTIFICATION_BAD_INPUT;
    if (sample->saturated) flags |= GIMBAL_IDENTIFICATION_SATURATED;
    if (isfinite(sample->feedback_age_s) &&
        sample->feedback_age_s > config->feedback_timeout_s)
        flags |= GIMBAL_IDENTIFICATION_STALE_FEEDBACK;
    if (!sample->operating_conditions_valid)
        flags |= GIMBAL_IDENTIFICATION_UNQUALIFIED_OPERATION;
    if (!sample->torque_calibrated)
        flags |= GIMBAL_IDENTIFICATION_TORQUE_UNCALIBRATED;
    if (!sample->torque_time_aligned)
        flags |= GIMBAL_IDENTIFICATION_TORQUE_UNALIGNED;
    if (sample->configuration_id != config->configuration_id)
        flags |= GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH;
    return flags;
}

static bool integrate(gimbal_identification_t *identification,
                       const gimbal_identification_sample_t *sample,
                       float dt_s)
{
    const gimbal_identification_sample_t *previous = &identification->previous;
    const float epsilon = identification->config.friction_velocity_rad_s;
    const float previous_argument = previous->rate_rad_s / epsilon;
    const float argument = sample->rate_rad_s / epsilon;
    float torque, friction, gravity_sin, gravity_cos;
    if (!isfinite(previous_argument) || !isfinite(argument)) return false;
    /* Half each operand before adding: avoid an avoidable overflow in a+b. */
    torque = identification->integral_torque +
        (0.5f * previous->torque_nm + 0.5f * sample->torque_nm) * dt_s;
    friction = identification->integral_friction +
        (0.5f * tanhf(previous_argument) + 0.5f * tanhf(argument)) * dt_s;
    gravity_sin = identification->integral_gravity_sin;
    gravity_cos = identification->integral_gravity_cos;
    if (identification->config.axis == GIMBAL_IDENTIFICATION_PITCH) {
        gravity_sin += (0.5f * sinf(previous->gravity_angle_rad) +
                        0.5f * sinf(sample->gravity_angle_rad)) * dt_s;
        gravity_cos += (0.5f * cosf(previous->gravity_angle_rad) +
                        0.5f * cosf(sample->gravity_angle_rad)) * dt_s;
    }
    if (!isfinite(torque) || !isfinite(friction) ||
        !isfinite(gravity_sin) || !isfinite(gravity_cos)) return false;
    identification->integral_torque = torque;
    identification->integral_friction = friction;
    identification->integral_gravity_sin = gravity_sin;
    identification->integral_gravity_cos = gravity_cos;
    return true;
}

uint32_t gimbal_identification_update(gimbal_identification_t *identification,
                                      const gimbal_identification_sample_t *sample,
                                      gimbal_identification_output_t *output)
{
    gimbal_rls_input_t regression;
    uint32_t flags, elapsed_us, interval_us, rls_flags;
    float duration_s;
    unsigned index;
    const uint32_t rls_faults = GIMBAL_RLS_DISABLED | GIMBAL_RLS_BAD_CONFIG |
        GIMBAL_RLS_BAD_INPUT | GIMBAL_RLS_BAD_DT | GIMBAL_RLS_NUMERIC_FAULT |
        GIMBAL_RLS_COVARIANCE_FAULT;

    if (identification == NULL || !identification->initialized)
        return finish(NULL, output, GIMBAL_IDENTIFICATION_BAD_CONFIG, 0.0f);
    if (!config_valid(&identification->config)) {
        break_observations(identification);
        return finish(NULL, output, GIMBAL_IDENTIFICATION_BAD_CONFIG, 0.0f);
    }
    if (!gimbal_rls_snapshot(&identification->estimator,
                             &identification->diagnostics)) {
        break_observations(identification);
        return finish(identification, output, GIMBAL_IDENTIFICATION_RLS_REJECTED, 0.0f);
    }
    if (sample == NULL || output == NULL) {
        break_observations(identification);
        return finish(identification, output, GIMBAL_IDENTIFICATION_BAD_INPUT, 0.0f);
    }
    flags = qualify(&identification->config, sample);
    if ((flags & GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH) != 0u) {
        /* An unrelated configuration must not advance this instance's clock. */
        break_observations(identification);
        return finish(identification, output, flags, 0.0f);
    }
    if (identification->have_timestamp) {
        interval_us = sample->timestamp_us - identification->last_timestamp_us;
        if (interval_us == 0u || interval_us > UINT32_MAX / 2u) {
            /* Keep the last accepted clock position; do not rewind on error. */
            flags |= GIMBAL_IDENTIFICATION_BAD_TIMESTAMP;
            break_observations(identification);
            return finish(identification, output, flags, 0.0f);
        }
        if (interval_us > identification->max_gap_us)
            flags |= GIMBAL_IDENTIFICATION_GAP;
    }
    identification->last_timestamp_us = sample->timestamp_us;
    identification->have_timestamp = true;
    if ((flags & ~GIMBAL_IDENTIFICATION_GAP) != 0u) {
        break_observations(identification);
        return finish(identification, output, flags, 0.0f);
    }
    if (identification->window_active &&
        sample->segment_id != identification->previous.segment_id)
        flags |= GIMBAL_IDENTIFICATION_SEGMENT_CHANGE;
    if (flags != 0u)
        break_observations(identification);
    if (!identification->window_active) {
        anchor(identification, sample);
        return finish(identification, output, flags, 0.0f);
    }

    interval_us = sample->timestamp_us - identification->previous.timestamp_us;
    if (interval_us == 0u || interval_us > identification->max_gap_us ||
        !integrate(identification, sample,
                    (float)interval_us * SECONDS_PER_MICROSECOND_F)) {
        break_observations(identification);
        return finish(identification, output,
                       flags | GIMBAL_IDENTIFICATION_NUMERIC_FAULT, 0.0f);
    }
    identification->previous = *sample;
    elapsed_us = sample->timestamp_us - identification->window_start_us;
    if (elapsed_us < identification->window_us)
        return finish(identification, output, flags, 0.0f);

    duration_s = (float)elapsed_us * SECONDS_PER_MICROSECOND_F;
    memset(&regression, 0, sizeof(regression));
    regression.features[0] =
        (sample->rate_rad_s - identification->start_rate_rad_s) / duration_s;
    regression.features[1] =
        (sample->angle_rad - identification->start_angle_rad) / duration_s;
    regression.features[2] = identification->integral_friction / duration_s;
    if (identification->config.axis == GIMBAL_IDENTIFICATION_PITCH) {
        regression.features[3] = identification->integral_gravity_sin / duration_s;
        regression.features[4] = identification->integral_gravity_cos / duration_s;
    }
    regression.target = identification->integral_torque / duration_s;
    regression.dt_s = duration_s;
    regression.enabled = true;
    regression.valid = isfinite(regression.target) && positive_finite(duration_s);
    for (index = 0u; index < identification->config.rls.parameter_count; ++index)
        regression.valid = regression.valid && isfinite(regression.features[index]);
    if (!regression.valid) {
        break_observations(identification);
        return finish(identification, output,
                       flags | GIMBAL_IDENTIFICATION_NUMERIC_FAULT, 0.0f);
    }
    rls_flags = gimbal_rls_update(&identification->estimator, &regression,
                                  &identification->diagnostics);
    if ((rls_flags & rls_faults) != 0u) {
        break_observations(identification);
        identification->diagnostics.flags = rls_flags;
        return finish(identification, output,
                       flags | GIMBAL_IDENTIFICATION_RLS_REJECTED, duration_s);
    }
    /* Low excitation must retain the RLS feature history so PE can eventually
     * accumulate. It is not a discontinuity. Projection similarly retains the
     * accepted bounded estimate, but the core marks that update not ready.
     */
    anchor(identification, sample);
    return finish(identification, output, flags, duration_s);
}
