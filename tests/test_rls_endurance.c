#include "gimbal_rls.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    exit(EXIT_FAILURE); } } while (0)

#define PARAMETERS 5u
#define STEPS 120000u
#define CYCLE_STEPS 10000u
#define RECOVERY_SAMPLES 800u

/* Synthetic regression, not a mechanical plant: the five independently
 * excited directions isolate float RLS endurance, drift tracking and recovery.
 * Truth is generated in double precision without using estimator state.
 * No result here establishes motor calibration or real-device performance. */
static void truth_normalized(double time_s, double truth[PARAMETERS])
{
    const double tau = 6.283185307179586476925286766559;
    truth[0] = 0.70 + 0.14 * sin(tau * time_s / 110.0);
    truth[1] = 0.35 + 0.08 * cos(tau * time_s / 170.0 + 0.3);
    truth[2] = 0.25 + 0.06 * sin(tau * time_s / 135.0 + 0.8);
    truth[3] = 0.12 + 0.08 * cos(tau * time_s / 190.0 + 0.4);
    truth[4] = -0.20 + 0.07 * sin(tau * time_s / 155.0 + 0.2);
}

static gimbal_rls_config_t configuration(void)
{
    const float scales[PARAMETERS] = {64.0f, 3.0f, 0.5f, 1.25f, 4.0f};
    double initial[PARAMETERS];
    gimbal_rls_config_t config;
    unsigned i;
    truth_normalized(0.0, initial);
    gimbal_rls_default_config(&config, PARAMETERS);
    config.excitation_window = 32u;
    config.min_updates = 48u;
    config.output_scale = 0.75f;
    config.covariance_initial = 4.0f;
    config.covariance_limit = 100.0f;
    config.forgetting_time_s = 2.0f;
    config.dt_min_s = 0.005f;
    config.dt_max_s = 0.02f;
    config.min_feature_energy = 0.05f;
    config.min_excitation_pivot = 0.1f;
    config.innovation_rms_limit = 0.07f;
    for (i = 0u; i < PARAMETERS; ++i) {
        config.feature_scale[i] = scales[i];
        config.initial[i] = (float)(initial[i] * config.output_scale / scales[i]);
        config.minimum[i] = (i < 3u ? 0.01f : -1.0f) * config.output_scale / scales[i];
        config.maximum[i] = 1.5f * config.output_scale / scales[i];
    }
    return config;
}

/* Independent double Cholesky audit of the stored float covariance. Reading
 * private storage is intentional for this test; production uses snapshot(). */
static void audit_covariance(const gimbal_rls_t *estimator,
                             const gimbal_rls_output_t *output)
{
    double lower[PARAMETERS][PARAMETERS] = {{0.0}};
    unsigned i, j, k;
    for (i = 0u; i < PARAMETERS; ++i) {
        CHECK(isfinite(output->parameters[i]));
        CHECK(isfinite(output->covariance_diag[i]));
        CHECK(output->parameters[i] >= estimator->config.minimum[i]);
        CHECK(output->parameters[i] <= estimator->config.maximum[i]);
        CHECK(output->covariance_diag[i] > 0.0f);
        CHECK(output->covariance_diag[i] <= estimator->config.covariance_limit);
        for (j = 0u; j < PARAMETERS; ++j) {
            CHECK(isfinite(estimator->covariance[i][j]));
            CHECK(estimator->covariance[i][j] == estimator->covariance[j][i]);
        }
        for (j = 0u; j <= i; ++j) {
            double value = estimator->covariance[i][j];
            for (k = 0u; k < j; ++k) value -= lower[i][k] * lower[j][k];
            CHECK(isfinite(value));
            if (i == j) {
                CHECK(value > 0.0);
                lower[i][j] = sqrt(value);
            } else {
                lower[i][j] = value / lower[j][j];
                CHECK(isfinite(lower[i][j]));
            }
        }
    }
}

int main(void)
{
    const float amplitudes[PARAMETERS] = {1.0f, 0.8f, 1.2f, 0.6f, 1.4f};
    const uint32_t unexpected = GIMBAL_RLS_BAD_CONFIG | GIMBAL_RLS_BAD_INPUT |
        GIMBAL_RLS_BAD_DT | GIMBAL_RLS_PROJECTED | GIMBAL_RLS_NUMERIC_FAULT |
        GIMBAL_RLS_COVARIANCE_FAULT;
    const gimbal_rls_config_t config = configuration();
    double sum_squared[PARAMETERS] = {0.0}, fixed_squared[PARAMETERS] = {0.0};
    double peak_error[PARAMETERS] = {0.0}, initial_truth[PARAMETERS];
    gimbal_rls_t estimator;
    gimbal_rls_output_t output;
    uint32_t generator = UINT32_C(0x9e3779b9);
    uint32_t accepted = 0u, scored = 0u, disabled_frozen = 0u, pe_frozen = 0u;
    unsigned step, i, uninterrupted = 0u, recovered_intervals = 0u;
    clock_t started = clock();
    truth_normalized(0.0, initial_truth);
    CHECK(gimbal_rls_init(&estimator, &config));
    for (step = 0u; step < STEPS; ++step) {
        const unsigned phase = step % CYCLE_STEPS;
        const bool disabled = phase >= 3000u && phase < 3200u;
        const bool weak = phase >= 6500u && phase < 6850u;
        const uint32_t before_updates = estimator.accepted_updates;
        float saved_theta[GIMBAL_RLS_MAX_PARAMETERS];
        float saved_covariance[GIMBAL_RLS_MAX_PARAMETERS][GIMBAL_RLS_MAX_PARAMETERS];
        double truth[PARAMETERS], target = 0.0;
        gimbal_rls_input_t input;
        uint32_t flags;
        memset(&input, 0, sizeof(input));
        input.enabled = !disabled;
        input.valid = true;
        input.dt_s = 0.01f;
        truth_normalized((double)step * 0.01, truth);
        for (i = 0u; i < PARAMETERS; ++i) {
            /* Five orthogonal binary directions over each 32-sample period.
             * During low excitation all columns become proportional while
             * retaining nonzero energy, exercising the correlation gate. */
            float feature = weak ? 1.0f : (((step >> i) & 1u) != 0u ? amplitudes[i] : -amplitudes[i]);
            double physical_parameter = truth[i] * config.output_scale / config.feature_scale[i];
            input.features[i] = feature * config.feature_scale[i];
            target += physical_parameter * input.features[i];
        }
        generator = generator * UINT32_C(1664525) + UINT32_C(1013904223);
        target += config.output_scale * 0.006 *
            (2.0 * (double)(generator >> 8u) / 16777215.0 - 1.0);
        input.target = (float)target;
        memcpy(saved_theta, estimator.theta, sizeof(saved_theta));
        memcpy(saved_covariance, estimator.covariance, sizeof(saved_covariance));
        flags = gimbal_rls_update(&estimator, &input, &output);
        CHECK(flags == output.flags);
        CHECK((flags & unexpected) == 0u);
        CHECK(isfinite(output.prediction) && isfinite(output.residual));
        CHECK(isfinite(output.innovation_rms));
        audit_covariance(&estimator, &output);

        if (disabled || (flags & GIMBAL_RLS_LOW_EXCITATION) != 0u) {
            /* No floating tolerance: a held model must be bitwise unchanged,
             * including P despite a configured positive forgetting time. */
            CHECK(memcmp(saved_theta, estimator.theta, sizeof(saved_theta)) == 0);
            CHECK(memcmp(saved_covariance, estimator.covariance, sizeof(saved_covariance)) == 0);
            CHECK(estimator.accepted_updates == before_updates);
            CHECK(!output.updated && !output.ready);
            if (disabled) ++disabled_frozen;
            else ++pe_frozen;
        }
        if (disabled) CHECK((flags & GIMBAL_RLS_DISABLED) != 0u);
        if (weak && phase >= 6500u + config.excitation_window)
            CHECK((flags & GIMBAL_RLS_LOW_EXCITATION) != 0u);
        if (output.updated) ++accepted;
        CHECK(output.accepted_updates == accepted);
        if (disabled || weak) {
            uninterrupted = 0u;
        } else {
            ++uninterrupted;
            if (uninterrupted == RECOVERY_SAMPLES) ++recovered_intervals;
            if (uninterrupted >= RECOVERY_SAMPLES) {
                CHECK(output.updated && output.ready && output.innovation_rms_valid);
                ++scored;
                for (i = 0u; i < PARAMETERS; ++i) {
                    double normalized = (double)output.parameters[i] *
                        config.feature_scale[i] / config.output_scale;
                    double error = normalized - truth[i];
                    double fixed_error = initial_truth[i] - truth[i];
                    sum_squared[i] += error * error;
                    fixed_squared[i] += fixed_error * fixed_error;
                    peak_error[i] = fmax(peak_error[i], fabs(error));
                }
            }
        }
    }

    CHECK(accepted > 110000u);
    CHECK(scored > 90000u);
    CHECK(disabled_frozen == (STEPS / CYCLE_STEPS) * 200u);
    CHECK(pe_frozen > 4000u);
    CHECK(recovered_intervals == 1u + 2u * (STEPS / CYCLE_STEPS));
    printf("RLS endurance: %u observations, %u accepted updates, %u scored; "
           "%u disabled/%u low-PE frozen calls, %u recovered intervals\n",
           STEPS, accepted, scored, disabled_frozen, pe_frozen, recovered_intervals);
    puts("parameter,normalized_rmse,normalized_peak_error,fixed_initial_rmse");
    for (i = 0u; i < PARAMETERS; ++i) {
        double rmse = sqrt(sum_squared[i] / scored);
        double fixed_rmse = sqrt(fixed_squared[i] / scored);
        /* Predeclared gates in normalized parameter coordinates. They allow
         * expected lag of a 2 s forgetting time against 110..190 s drift and
         * still reject simply retaining the initial parameter model. */
        CHECK(rmse < 0.035);
        CHECK(peak_error[i] < 0.07);
        CHECK(rmse < 0.5 * fixed_rmse);
        printf("%u,%.9g,%.9g,%.9g\n", i, rmse, peak_error[i], fixed_rmse);
    }
    printf("Synthetic numeric endurance PASS; host CPU %.3f s. "
           "This is not a hardware performance test.\n",
           (double)(clock() - started) / CLOCKS_PER_SEC);
    return EXIT_SUCCESS;
}
