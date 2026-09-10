#include "gimbal_rls.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "gimbal_rls requires finite-value checks; disable fast-math and finite-math-only"
#endif

typedef float rls_matrix_t[GIMBAL_RLS_MAX_PARAMETERS][GIMBAL_RLS_MAX_PARAMETERS];

static bool positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static float to_normalized(const gimbal_rls_config_t *c, unsigned index, float value)
{
    return (value / c->output_scale) * c->feature_scale[index];
}

static bool config_valid(const gimbal_rls_config_t *c)
{
    unsigned i;
    if (c == NULL || c->parameter_count == 0u ||
        c->parameter_count > GIMBAL_RLS_MAX_PARAMETERS ||
        c->excitation_window < c->parameter_count ||
        c->excitation_window > GIMBAL_RLS_MAX_OBSERVATIONS ||
        c->min_updates == 0u || !positive(c->output_scale) ||
        !positive(c->covariance_initial) || !positive(c->covariance_limit) ||
        c->covariance_initial > c->covariance_limit ||
        !positive(c->dt_min_s) || !positive(c->dt_max_s) || c->dt_min_s > c->dt_max_s ||
        !isfinite(c->forgetting_time_s) || c->forgetting_time_s < 0.0f ||
        !positive(c->min_feature_energy) || !positive(c->min_excitation_pivot) ||
        c->min_excitation_pivot > 1.0f || !positive(c->innovation_rms_limit)) return false;
    if (c->forgetting_time_s > 0.0f) {
        float ratio = c->dt_max_s / c->forgetting_time_s;
        float rho = expf(-ratio);
        if (!isfinite(ratio) || !positive(rho)) return false;
    }
    for (i = 0u; i < c->parameter_count; ++i) {
        float lower, upper, initial;
        if (!positive(c->feature_scale[i]) || !isfinite(c->initial[i]) ||
            !isfinite(c->minimum[i]) || !isfinite(c->maximum[i]) ||
            c->minimum[i] > c->maximum[i] || c->initial[i] < c->minimum[i] ||
            c->initial[i] > c->maximum[i]) return false;
        lower = to_normalized(c, i, c->minimum[i]);
        upper = to_normalized(c, i, c->maximum[i]);
        initial = to_normalized(c, i, c->initial[i]);
        if (!isfinite(lower) || !isfinite(upper) || !isfinite(initial) ||
            lower > upper || initial < lower || initial > upper ||
            (c->minimum[i] != 0.0f && lower == 0.0f) ||
            (c->maximum[i] != 0.0f && upper == 0.0f) ||
            (c->initial[i] != 0.0f && initial == 0.0f)) return false;
    }
    return true;
}

/* Input is a flat fixed-size matrix to avoid pre-C2X array-qualifier issues. */
static bool cholesky(const float *a, unsigned n, float minimum_pivot)
{
    rls_matrix_t lower = {{0.0f}};
    unsigned i, j, k;
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j <= i; ++j) {
            float value = a[i * GIMBAL_RLS_MAX_PARAMETERS + j];
            for (k = 0u; k < j; ++k) value -= lower[i][k] * lower[j][k];
            if (!isfinite(value)) return false;
            if (i == j) {
                if (value <= 0.0f || value < minimum_pivot) return false;
                lower[i][j] = sqrtf(value);
            } else {
                lower[i][j] = value / lower[j][j];
                if (!isfinite(lower[i][j])) return false;
            }
        }
    }
    return true;
}

static bool covariance_valid(const float *p, unsigned n, float limit)
{
    unsigned i, j;
    for (i = 0u; i < n; ++i) {
        float diagonal = p[i * GIMBAL_RLS_MAX_PARAMETERS + i];
        if (!positive(diagonal) || diagonal > limit) return false;
        for (j = 0u; j < n; ++j) {
            float a = p[i * GIMBAL_RLS_MAX_PARAMETERS + j];
            float b = p[j * GIMBAL_RLS_MAX_PARAMETERS + i];
            float magnitude = fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));
            if (!isfinite(a) || !isfinite(b) ||
                fabsf(a - b) > 16.0f * FLT_EPSILON * magnitude) return false;
        }
    }
    return cholesky(p, n, 0.0f);
}

static uint32_t state_fault(const gimbal_rls_t *s)
{
    unsigned i;
    if (s == NULL || !s->initialized || !config_valid(&s->config)) return GIMBAL_RLS_BAD_CONFIG;
    if (s->observation_count > s->config.excitation_window ||
        s->observation_next >= s->config.excitation_window ||
        s->innovation_count > s->config.excitation_window ||
        s->innovation_next >= s->config.excitation_window) return GIMBAL_RLS_NUMERIC_FAULT;
    for (i = 0u; i < s->config.parameter_count; ++i) {
        float physical = (s->theta[i] / s->config.feature_scale[i]) * s->config.output_scale;
        if (!isfinite(s->theta[i]) || !isfinite(physical) ||
            s->theta[i] < to_normalized(&s->config, i, s->config.minimum[i]) ||
            s->theta[i] > to_normalized(&s->config, i, s->config.maximum[i]))
            return GIMBAL_RLS_NUMERIC_FAULT;
    }
    if (!covariance_valid(&s->covariance[0][0], s->config.parameter_count,
                          s->config.covariance_limit)) return GIMBAL_RLS_COVARIANCE_FAULT;
    return 0u;
}

static bool innovation_rms(const float *values, unsigned count, float *result)
{
    unsigned i;
    float maximum = 0.0f, sum = 0.0f;
    *result = 0.0f;
    for (i = 0u; i < count; ++i) {
        if (!isfinite(values[i])) return false;
        maximum = fmaxf(maximum, fabsf(values[i]));
    }
    if (count == 0u || maximum == 0.0f) return true;
    for (i = 0u; i < count; ++i) {
        float scaled = values[i] / maximum;
        sum += scaled * scaled;
    }
    *result = maximum * sqrtf(sum / (float)count);
    return isfinite(*result);
}

static void clear_innovations(gimbal_rls_t *s)
{
    memset(s->innovations, 0, sizeof(s->innovations));
    s->innovation_count = 0u;
    s->innovation_next = 0u;
    s->consecutive_updates = 0u;
}

void gimbal_rls_default_config(gimbal_rls_config_t *c, uint8_t parameter_count)
{
    unsigned i;
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    c->parameter_count = parameter_count;
    c->excitation_window = GIMBAL_RLS_MAX_OBSERVATIONS;
    c->min_updates = 32u;
    for (i = 0u; i < GIMBAL_RLS_MAX_PARAMETERS; ++i) {
        c->minimum[i] = -10.0f;
        c->maximum[i] = 10.0f;
        c->feature_scale[i] = 1.0f;
    }
    c->output_scale = 1.0f;
    c->covariance_initial = 10.0f;
    c->covariance_limit = 1000000.0f;
    c->dt_min_s = 0.0001f;
    c->dt_max_s = 0.5f;
    c->min_feature_energy = 0.0001f;
    c->min_excitation_pivot = 0.01f;
    c->innovation_rms_limit = 0.1f;
}

bool gimbal_rls_init(gimbal_rls_t *s, const gimbal_rls_config_t *config)
{
    unsigned i;
    gimbal_rls_config_t copy;
    if (s == NULL) return false;
    if (config == NULL) {
        memset(s, 0, sizeof(*s));
        return false;
    }
    copy = *config; /* Also permit init(s, &s->config). */
    memset(s, 0, sizeof(*s));
    if (!config_valid(&copy)) return false;
    s->config = copy;
    for (i = 0u; i < copy.parameter_count; ++i) {
        s->theta[i] = to_normalized(&copy, i, copy.initial[i]);
        s->covariance[i][i] = copy.covariance_initial;
    }
    s->initialized = true;
    if (state_fault(s) != 0u) {
        memset(s, 0, sizeof(*s));
        return false;
    }
    return true;
}

void gimbal_rls_reset(gimbal_rls_t *s)
{
    if (s != NULL) (void)gimbal_rls_init(s, &s->config);
}

void gimbal_rls_clear_observations(gimbal_rls_t *s)
{
    if (s == NULL) return;
    memset(s->observations, 0, sizeof(s->observations));
    s->observation_count = 0u;
    s->observation_next = 0u;
    clear_innovations(s);
}

bool gimbal_rls_snapshot(const gimbal_rls_t *s, gimbal_rls_output_t *output)
{
    unsigned i;
    uint32_t fault;
    if (output == NULL) return false;
    memset(output, 0, sizeof(*output));
    fault = state_fault(s);
    if (fault != 0u) {
        output->flags = fault;
        return false;
    }
    if (!innovation_rms(s->innovations, s->innovation_count, &output->innovation_rms)) {
        memset(output, 0, sizeof(*output));
        output->flags = GIMBAL_RLS_NUMERIC_FAULT;
        return false;
    }
    for (i = 0u; i < s->config.parameter_count; ++i) {
        float physical = (s->theta[i] / s->config.feature_scale[i]) * s->config.output_scale;
        /* Roundoff when converting an exact projected normalized bound must
         * not report a physical value one ULP beyond that configured bound. */
        output->parameters[i] = fminf(s->config.maximum[i], fmaxf(s->config.minimum[i], physical));
        output->covariance_diag[i] = s->covariance[i][i];
    }
    output->accepted_updates = s->accepted_updates;
    output->consecutive_updates = s->consecutive_updates;
    output->innovation_rms_valid = s->innovation_count > 0u;
    return true;
}

static uint32_t fail_update(gimbal_rls_t *s, gimbal_rls_output_t *out, uint32_t flags)
{
    gimbal_rls_clear_observations(s);
    if (out != NULL) {
        (void)gimbal_rls_snapshot(s, out);
        out->flags |= flags;
        out->ready = false;
        out->updated = false;
    }
    return flags;
}

static uint32_t excitation_status(const gimbal_rls_t *s)
{
    rls_matrix_t gram = {{0.0f}}, correlation = {{0.0f}};
    unsigned i, j, k, n = s->config.parameter_count;
    float count = (float)s->config.excitation_window;
    if (s->observation_count < s->config.excitation_window) return GIMBAL_RLS_LOW_EXCITATION;
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            float value = 0.0f;
            for (k = 0u; k < s->config.excitation_window; ++k)
                value += (s->observations[k][i] / count) * s->observations[k][j];
            if (!isfinite(value)) return GIMBAL_RLS_NUMERIC_FAULT;
            gram[i][j] = value;
        }
        if (gram[i][i] < s->config.min_feature_energy) return GIMBAL_RLS_LOW_EXCITATION;
    }
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            correlation[i][j] = (gram[i][j] / sqrtf(gram[i][i])) / sqrtf(gram[j][j]);
            if (!isfinite(correlation[i][j])) return GIMBAL_RLS_NUMERIC_FAULT;
        }
        correlation[i][i] = 1.0f;
    }
    return cholesky(&correlation[0][0], n, s->config.min_excitation_pivot)
        ? 0u : GIMBAL_RLS_LOW_EXCITATION;
}

uint32_t gimbal_rls_update(gimbal_rls_t *s, const gimbal_rls_input_t *in,
                         gimbal_rls_output_t *out)
{
    rls_matrix_t prior = {{0.0f}}, identity_minus_gain = {{0.0f}};
    rls_matrix_t temporary = {{0.0f}}, next_covariance = {{0.0f}};
    float x[GIMBAL_RLS_MAX_PARAMETERS] = {0.0f};
    float gain[GIMBAL_RLS_MAX_PARAMETERS] = {0.0f};
    float next_theta[GIMBAL_RLS_MAX_PARAMETERS] = {0.0f};
    float next_innovations[GIMBAL_RLS_MAX_OBSERVATIONS];
    float normalized_target, normalized_prediction = 0.0f;
    float prediction, residual, normalized_residual, denominator = 1.0f, rho = 1.0f, rms;
    unsigned i, j, k, n;
    uint8_t next_innovation_count, next_innovation_slot;
    uint32_t flags;
    if (out == NULL) return fail_update(s, NULL, GIMBAL_RLS_BAD_INPUT);
    memset(out, 0, sizeof(*out));
    flags = state_fault(s);
    if (flags != 0u) return fail_update(s, out, flags);
    if (in == NULL) return fail_update(s, out, GIMBAL_RLS_BAD_INPUT);
    if (!in->enabled) return fail_update(s, out, GIMBAL_RLS_DISABLED);
    if (!in->valid) return fail_update(s, out, GIMBAL_RLS_BAD_INPUT);
    if (!isfinite(in->dt_s) || in->dt_s < s->config.dt_min_s || in->dt_s > s->config.dt_max_s)
        return fail_update(s, out, GIMBAL_RLS_BAD_DT);
    if (!isfinite(in->target)) return fail_update(s, out, GIMBAL_RLS_BAD_INPUT);
    n = s->config.parameter_count;
    normalized_target = in->target / s->config.output_scale;
    if (!isfinite(normalized_target)) return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
    for (i = 0u; i < n; ++i) {
        if (!isfinite(in->features[i])) return fail_update(s, out, GIMBAL_RLS_BAD_INPUT);
        x[i] = in->features[i] / s->config.feature_scale[i];
        normalized_prediction += x[i] * s->theta[i];
        if (!isfinite(x[i]) || !isfinite(normalized_prediction))
            return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
    }
    prediction = normalized_prediction * s->config.output_scale;
    residual = in->target - prediction;
    normalized_residual = normalized_target - normalized_prediction;
    if (!isfinite(prediction) || !isfinite(residual) || !isfinite(normalized_residual))
        return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);

    for (i = 0u; i < n; ++i) s->observations[s->observation_next][i] = x[i];
    s->observation_next = (uint8_t)((s->observation_next + 1u) % s->config.excitation_window);
    if (s->observation_count < s->config.excitation_window) ++s->observation_count;
    flags = excitation_status(s);
    if (flags != 0u) {
        if (flags != GIMBAL_RLS_LOW_EXCITATION) return fail_update(s, out, flags);
        clear_innovations(s); /* Keep the feature ring so new information can restore PE. */
        (void)gimbal_rls_snapshot(s, out);
        out->prediction = prediction;
        out->residual = residual;
        out->flags = flags;
        return flags;
    }

    if (s->config.forgetting_time_s > 0.0f) rho = expf(-in->dt_s / s->config.forgetting_time_s);
    if (!positive(rho)) return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            prior[i][j] = s->covariance[i][j] / rho;
            if (!isfinite(prior[i][j])) return fail_update(s, out, GIMBAL_RLS_COVARIANCE_FAULT);
            gain[i] += prior[i][j] * x[j];
        }
        if (!isfinite(gain[i])) return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
        denominator += x[i] * gain[i];
    }
    if (!positive(denominator)) return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
    flags = 0u;
    for (i = 0u; i < n; ++i) {
        float lower = to_normalized(&s->config, i, s->config.minimum[i]);
        float upper = to_normalized(&s->config, i, s->config.maximum[i]);
        gain[i] /= denominator;
        next_theta[i] = s->theta[i] + gain[i] * normalized_residual;
        if (!isfinite(gain[i]) || !isfinite(next_theta[i]))
            return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
        if (next_theta[i] < lower) { next_theta[i] = lower; flags |= GIMBAL_RLS_PROJECTED; }
        if (next_theta[i] > upper) { next_theta[i] = upper; flags |= GIMBAL_RLS_PROJECTED; }
        for (j = 0u; j < n; ++j) {
            identity_minus_gain[i][j] = (i == j ? 1.0f : 0.0f) - gain[i] * x[j];
            if (!isfinite(identity_minus_gain[i][j]))
                return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
        }
    }
    /* Joseph form: (I-K*x')*Pprior*(I-K*x')' + K*1*K'. */
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            for (k = 0u; k < n; ++k) temporary[i][j] += identity_minus_gain[i][k] * prior[k][j];
            if (!isfinite(temporary[i][j])) return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
        }
    }
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < n; ++j) {
            next_covariance[i][j] = gain[i] * gain[j];
            for (k = 0u; k < n; ++k)
                next_covariance[i][j] += temporary[i][k] * identity_minus_gain[j][k];
            if (!isfinite(next_covariance[i][j])) return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
        }
    }
    for (i = 0u; i < n; ++i) {
        for (j = 0u; j < i; ++j) {
            float symmetric = 0.5f * next_covariance[i][j] + 0.5f * next_covariance[j][i];
            next_covariance[i][j] = symmetric;
            next_covariance[j][i] = symmetric;
        }
    }
    if (!covariance_valid(&next_covariance[0][0], n, s->config.covariance_limit))
        return fail_update(s, out, GIMBAL_RLS_COVARIANCE_FAULT);
    for (i = 0u; i < n; ++i) {
        float physical = (next_theta[i] / s->config.feature_scale[i]) * s->config.output_scale;
        if (!isfinite(physical)) return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);
    }
    memcpy(next_innovations, s->innovations, sizeof(next_innovations));
    next_innovations[s->innovation_next] = residual;
    next_innovation_count = s->innovation_count;
    if (next_innovation_count < s->config.excitation_window) ++next_innovation_count;
    next_innovation_slot = (uint8_t)((s->innovation_next + 1u) % s->config.excitation_window);
    if (!innovation_rms(next_innovations, next_innovation_count, &rms))
        return fail_update(s, out, GIMBAL_RLS_NUMERIC_FAULT);

    /* Commit only after all numerical, covariance and physical checks pass. */
    memcpy(s->theta, next_theta, sizeof(s->theta));
    memcpy(s->covariance, next_covariance, sizeof(s->covariance));
    memcpy(s->innovations, next_innovations, sizeof(s->innovations));
    s->innovation_count = next_innovation_count;
    s->innovation_next = next_innovation_slot;
    if (s->accepted_updates != UINT32_MAX) ++s->accepted_updates;
    if ((flags & GIMBAL_RLS_PROJECTED) != 0u) s->consecutive_updates = 0u;
    else if (s->consecutive_updates != UINT32_MAX) ++s->consecutive_updates;
    (void)gimbal_rls_snapshot(s, out);
    out->prediction = prediction;
    out->residual = residual;
    out->innovation_rms = rms;
    out->innovation_rms_valid = true;
    out->flags = flags;
    out->updated = true;
    out->ready = (flags & GIMBAL_RLS_PROJECTED) == 0u &&
        s->consecutive_updates >= s->config.min_updates && rms <= s->config.innovation_rms_limit;
    return flags;
}
