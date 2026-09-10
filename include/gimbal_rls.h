#ifndef GIMBAL_RLS_H
#define GIMBAL_RLS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GIMBAL_RLS_MAX_PARAMETERS 5u
#define GIMBAL_RLS_MAX_OBSERVATIONS 32u

/* One instance, one caller task; synchronize external measurements separately.
 * Pure C99 / float / static storage. Compile without -ffast-math.
 * A ready result is a candidate-data gate, NOT parameter identifiability,
 * a confidence interval, permission to change control gains, or motor enable.
 */
typedef struct {
    uint8_t parameter_count;       /* 1..5 */
    uint8_t excitation_window;     /* parameter_count..32, fixed rolling window */
    uint32_t min_updates;          /* >=1 consecutive non-projected updates */
    float initial[GIMBAL_RLS_MAX_PARAMETERS]; /* Physical parameter units. */
    float minimum[GIMBAL_RLS_MAX_PARAMETERS]; /* Inclusive physical bounds. */
    float maximum[GIMBAL_RLS_MAX_PARAMETERS];
    float feature_scale[GIMBAL_RLS_MAX_PARAMETERS]; /* Finite, >0, fixed. */
    float output_scale;            /* Finite, >0, fixed. */
    float covariance_initial;      /* P0 diagonal in normalized coordinates. */
    float covariance_limit;        /* Maximum allowed diagonal of P, >=P0. */
    float forgetting_time_s;       /* 0: no forgetting; else rho=exp(-dt/tau). */
    float dt_min_s;
    float dt_max_s;
    float min_feature_energy;      /* Mean squared normalized feature, >0. */
    float min_excitation_pivot;    /* (0,1]; correlation Cholesky pivot BEFORE sqrt. */
    float innovation_rms_limit;    /* Physical output units; >0. */
} gimbal_rls_config_t;

typedef struct {
    float features[GIMBAL_RLS_MAX_PARAMETERS]; /* Active columns only. */
    float target;
    float dt_s;                   /* Interval for this regression observation. */
    bool enabled;
    bool valid;
} gimbal_rls_input_t;

enum {
    GIMBAL_RLS_DISABLED         = 1u << 0,
    GIMBAL_RLS_BAD_CONFIG       = 1u << 1,
    GIMBAL_RLS_BAD_INPUT        = 1u << 2,
    GIMBAL_RLS_BAD_DT           = 1u << 3,
    GIMBAL_RLS_LOW_EXCITATION   = 1u << 4,
    GIMBAL_RLS_PROJECTED        = 1u << 5,
    GIMBAL_RLS_NUMERIC_FAULT    = 1u << 6,
    GIMBAL_RLS_COVARIANCE_FAULT = 1u << 7
};

typedef struct {
    float parameters[GIMBAL_RLS_MAX_PARAMETERS]; /* Retained/updated physical values. */
    float prediction;             /* Current observation, BEFORE parameter update. */
    float residual;               /* target - prediction, physical output units. */
    float innovation_rms;          /* RMS of up to excitation_window pre-update
                                   * residuals from successful updates (including
                                   * projection). Cleared on insufficient PE/fault. */
    float covariance_diag[GIMBAL_RLS_MAX_PARAMETERS]; /* Normalized P, NOT CI. */
    uint32_t flags;
    uint32_t accepted_updates;     /* Lifetime accepted updates, including projection. */
    uint32_t consecutive_updates;  /* Consecutive accepted non-projected updates. */
    bool updated;
    bool ready;
    bool innovation_rms_valid;
} gimbal_rls_output_t;

/* Storage is public only for static allocation. ALL fields are PRIVATE to this
 * module; never edit config/theta/covariance/rings after init. Use snapshot().
 * theta_i = physical_parameter_i * feature_scale_i / output_scale;
 * normalized_feature_i = physical_feature_i / feature_scale_i.
 */
typedef struct {
    gimbal_rls_config_t config;
    float theta[GIMBAL_RLS_MAX_PARAMETERS];
    float covariance[GIMBAL_RLS_MAX_PARAMETERS][GIMBAL_RLS_MAX_PARAMETERS];
    float observations[GIMBAL_RLS_MAX_OBSERVATIONS][GIMBAL_RLS_MAX_PARAMETERS];
    float innovations[GIMBAL_RLS_MAX_OBSERVATIONS];
    uint32_t accepted_updates;
    uint32_t consecutive_updates;
    uint8_t observation_count;
    uint8_t observation_next;
    uint8_t innovation_count;
    uint8_t innovation_next;
    bool initialized;
} gimbal_rls_t;

/* Defaults demonstrate synthetic regression, not calibrated motor parameters. */
void gimbal_rls_default_config(gimbal_rls_config_t *config, uint8_t parameter_count);
/* A failed init clears the entire object. Configuration changes require init. */
bool gimbal_rls_init(gimbal_rls_t *estimator, const gimbal_rls_config_t *config);
/* Restore configured initial parameters and P0; clear observations and counters. */
void gimbal_rls_reset(gimbal_rls_t *estimator);
/* Keep parameters/P and lifetime accepted_updates. Clear PE, innovations and
 * consecutive_updates. Call on discontinuities; update also calls this on
 * disable/invalid input/numerical faults. Low PE retains only the feature ring.
 */
void gimbal_rls_clear_observations(gimbal_rls_t *estimator);
/* Read retained diagnostics only; always updated=false and ready=false.
 * On failure output is cleared, apart from a diagnostic fault flag. */
bool gimbal_rls_snapshot(const gimbal_rls_t *estimator, gimbal_rls_output_t *output);
/* Every call produces a new ready decision. PE requires a full feature window,
 * per-column energy and correlation pivots. Insufficient PE freezes BOTH theta
 * and P; time spent without PE is not later applied as accumulated forgetting.
 * Joseph update uses normalized measurement variance R=1, symmetrization and
 * Cholesky SPD checks. Numerical failures roll back theta/P for the whole call.
 * Physical bounds project theta itself; a projected update cannot be ready.
 * Failed/disabled calls return updated=false/ready=false; retained physical
 * parameters/P are returned only when that state passes snapshot checks.
 */
uint32_t gimbal_rls_update(gimbal_rls_t *estimator, const gimbal_rls_input_t *input,
                         gimbal_rls_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
