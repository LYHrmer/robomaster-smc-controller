#ifndef GIMBAL_IDENTIFICATION_H
#define GIMBAL_IDENTIFICATION_H

#include "gimbal_rls.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Online integral-regression bridge. One instance per fixed axis/configuration.
 * Pure C99; no allocation, HAL calls, motor commands or SMC configuration edits.
 * The application must synchronize each complete sample before calling update.
 */
typedef enum {
    GIMBAL_IDENTIFICATION_YAW = 0,
    GIMBAL_IDENTIFICATION_PITCH = 1
} gimbal_identification_axis_t;

typedef struct {
    gimbal_identification_axis_t axis;
    float window_s;
    float max_gap_s;
    float feedback_timeout_s;
    float friction_velocity_rad_s; /* Fixed positive epsilon in tanh(rate/eps). */
    uint32_t configuration_id;
    /* Parameter order: J, B, Fc for yaw; J, B, Fc, A, C for pitch.
     * Required bounds: minimum[J] > 0; minimum[B/Fc] >= 0.
     * A/C are signed. Bounds and initial values belong to this configuration.
     * RLS dt limits must cover window_s through window_s + max_gap_s.
     */
    gimbal_rls_config_t rls;
} gimbal_identification_config_t;

typedef struct {
    /* Acquisition time of a NEW coherent sample, not the time cached data is
     * read. Unsigned wrap is supported; ordering is unambiguous only when
     * successive timestamps are separated by less than 2^31 microseconds.
     */
    uint32_t timestamp_us;
    uint32_t segment_id;
    uint32_t configuration_id;
    /* Continuous joint coordinate, no shortest wrap. For long-running yaw,
     * rebase before float resolution is lost and START A NEW segment_id.
     * A coordinate shift inside an integral window corrupts delta-angle.
     */
    float angle_rad;
    float rate_rad_s;         /* Derivative of angle in the SAME fixed frame. */
    float gravity_angle_rad;  /* Fixed-plane gravity phase; finite even for yaw. */
    float torque_nm;          /* Time-aligned actual/qualified joint torque. */
    float feedback_age_s;     /* Conservative age of oldest required feedback. */
    bool enabled;
    bool valid;
    bool saturated;
    /* Caller assertions, NOT automatically inferred from the measurements.
     * operating_conditions_valid means fixed base/configuration and the
     * selected model applies. Torque calibration and alignment must already
     * be independently established; low residuals cannot establish either.
     */
    bool operating_conditions_valid;
    bool torque_calibrated;
    bool torque_time_aligned;
} gimbal_identification_sample_t;

typedef struct {
    float inertia_kg_m2;
    float viscous_nm_s_rad;
    float coulomb_nm;
    float friction_velocity_rad_s;
    float gravity_sin_nm;     /* Always zero for the yaw model. */
    float gravity_cos_nm;     /* Always zero for the yaw model. */
} gimbal_identification_model_t;

enum {
    GIMBAL_IDENTIFICATION_DISABLED               = 1u << 0,
    GIMBAL_IDENTIFICATION_BAD_INPUT              = 1u << 1,
    GIMBAL_IDENTIFICATION_SATURATED              = 1u << 2,
    GIMBAL_IDENTIFICATION_STALE_FEEDBACK         = 1u << 3,
    GIMBAL_IDENTIFICATION_BAD_TIMESTAMP          = 1u << 4,
    GIMBAL_IDENTIFICATION_GAP                    = 1u << 5,
    GIMBAL_IDENTIFICATION_SEGMENT_CHANGE         = 1u << 6,
    GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH = 1u << 7,
    GIMBAL_IDENTIFICATION_UNQUALIFIED_OPERATION  = 1u << 8,
    GIMBAL_IDENTIFICATION_TORQUE_UNCALIBRATED     = 1u << 9,
    GIMBAL_IDENTIFICATION_TORQUE_UNALIGNED       = 1u << 10,
    GIMBAL_IDENTIFICATION_NUMERIC_FAULT          = 1u << 11,
    GIMBAL_IDENTIFICATION_BAD_CONFIG             = 1u << 12,
    GIMBAL_IDENTIFICATION_RLS_REJECTED            = 1u << 13
};

typedef struct {
    gimbal_identification_model_t model;
    gimbal_rls_output_t rls;
    uint32_t flags; /* Wrapper namespace, independent of rls.flags. */
    float window_duration_s; /* Nonzero only when a full window was submitted. */
    /* Both are per-call events, never stale readiness from an earlier window.
     * window_updated means RLS accepted this newly completed window.
     * ready additionally requires this update to satisfy the RLS readiness
     * gates. Neither flag authorizes applying the estimate to a controller.
     */
    bool window_updated;
    bool ready;
} gimbal_identification_output_t;

/* Storage is exposed only for static allocation. Treat every member as private;
 * changing model/configuration/clock state requires the lifecycle APIs below.
 */
typedef struct {
    gimbal_identification_config_t config;
    gimbal_rls_t estimator;
    gimbal_rls_output_t diagnostics;
    gimbal_identification_sample_t previous;
    float start_angle_rad;
    float start_rate_rad_s;
    float integral_torque;
    float integral_friction;
    float integral_gravity_sin;
    float integral_gravity_cos;
    uint32_t window_start_us;
    uint32_t last_timestamp_us;
    uint32_t window_us;
    uint32_t max_gap_us;
    bool initialized;
    bool have_timestamp;
    bool window_active;
} gimbal_identification_t;

/* Defaults are synthetic starting values, NOT calibrated hardware parameters. */
void gimbal_identification_default_config(gimbal_identification_config_t *config,
                                          gimbal_identification_axis_t axis);
bool gimbal_identification_init(gimbal_identification_t *identification,
                                const gimbal_identification_config_t *config);
/* Restore initial parameters/P and clear counts, observations and timestamps. */
void gimbal_identification_reset(gimbal_identification_t *identification);
/* Keep learned parameters/P and total accepted-update count; clear readiness,
 * excitation/innovation histories, partial integral and timestamp continuity.
 */
void gimbal_identification_clear_observations(gimbal_identification_t *identification);
/* Samples must arrive once in acquisition order. Invalid/disabled/saturated/
 * stale/unqualified samples and all temporal/segment/configuration breaks drop
 * partial integration and clear RLS observation readiness while retaining the
 * learned parameters/P. Gaps/segment changes may anchor a fresh valid sample;
 * they never integrate across the break. Wrong configuration_id is rejected;
 * use its own instance or explicitly reinitialize for the new configuration.
 * Complete adjacent windows share an endpoint, never a time interval.
 */
uint32_t gimbal_identification_update(gimbal_identification_t *identification,
                                      const gimbal_identification_sample_t *sample,
                                      gimbal_identification_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
