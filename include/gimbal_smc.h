#ifndef GIMBAL_SMC_H
#define GIMBAL_SMC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pure C99, single-axis controller. All angles/rates/torques use rad, rad/s,
 * and N m. Call one instance from one task; synchronize feedback externally. */
typedef struct {
    float inertia_kg_m2;        /* J > 0, expressed at the controlled axis. */
    float viscous_nm_s_rad;     /* B >= 0, optional nominal friction. */
    float lambda_per_s;        /* Linear sliding-surface gain > 0. */
    float reaching_per_s;      /* k >= 0 in -k*s. */
    float robust_rad_s2;       /* eta >= 0 in -eta*sat(s/phi). */
    float boundary_rad_s;      /* phi > 0. */
    float terminal_gain;       /* alpha >= 0; zero selects linear SMC. */
    float terminal_power;      /* 0 < r < 1, even when alpha is zero. */
    float terminal_epsilon_rad;/* delta > 0, regularizes the terminal term. */
    float torque_limit_nm;     /* Absolute commanded torque > 0. */
    float torque_slew_nm_s;    /* >= 0; zero disables the slew limiter. */
    float rate_lpf_hz;         /* >= 0; zero disables gyro LPF. */
    float dt_min_s;            /* 0 < dt_min <= dt <= dt_max. */
    float dt_max_s;
    float feedback_timeout_s;  /* > 0; caller supplies conservative age. */
    bool wrap_angle;           /* Shortest-angle error; disable for joint limits. */
} gimbal_smc_config_t;

typedef struct {
    float angle_rad;
    float rate_rad_s;          /* Actual derivative of angle in SAME frame. */
    float reference_rad;
    float reference_rate_rad_s;
    float reference_accel_rad_s2;
    float feedforward_nm;      /* e.g. calibrated gravity compensation. */
    float feedback_age_s;      /* Age of oldest required feedback sample. */
    float dt_s;
    bool enable;
} gimbal_smc_input_t;

enum {
    GIMBAL_SMC_DISABLED        = 1u << 0,
    GIMBAL_SMC_BAD_CONFIG      = 1u << 1,
    GIMBAL_SMC_BAD_INPUT       = 1u << 2,
    GIMBAL_SMC_BAD_DT          = 1u << 3,
    GIMBAL_SMC_STALE_FEEDBACK  = 1u << 4,
    GIMBAL_SMC_NUMERIC_FAULT   = 1u << 5,
    GIMBAL_SMC_AMPLITUDE_LIMIT = 1u << 6,
    GIMBAL_SMC_SLEW_LIMIT      = 1u << 7
};

typedef struct {
    float torque_nm;
    float unsaturated_torque_nm;
    float error_rad;
    float rate_error_rad_s;
    float sliding_rad_s;
    float filtered_rate_rad_s;
    uint32_t flags;
    bool valid; /* true even if torque is limited; false always gives zero. */
} gimbal_smc_output_t;

typedef struct {
    gimbal_smc_config_t config;
    float filtered_rate_rad_s;
    float previous_torque_nm;
    bool filter_initialized;
    bool initialized;
} gimbal_smc_t;

/* Defaults are illustrative simulation parameters, NOT calibrated motor gains. */
void gimbal_smc_default_config(gimbal_smc_config_t *config);
bool gimbal_smc_init(gimbal_smc_t *controller,
                     const gimbal_smc_config_t *config);
/* Clears dynamic state while retaining initialization/configuration. */
void gimbal_smc_reset(gimbal_smc_t *controller);
/* A fault/disable immediately zeros torque and resets LPF/slew state. */
uint32_t gimbal_smc_update(gimbal_smc_t *controller,
                          const gimbal_smc_input_t *input,
                          gimbal_smc_output_t *output);

typedef struct {
    float angle_rad;
    float rate_rad_s;
    float accel_rad_s2;
} gimbal_smc_reference_t;

typedef struct {
    float max_rate_rad_s;
    float max_accel_rad_s2;
    float position_gain_per_s;
    float dt_min_s;
    float dt_max_s;
    bool wrap_angle;
} gimbal_reference_config_t;

typedef struct {
    gimbal_reference_config_t config;
    gimbal_smc_reference_t value;
    bool initialized;
} gimbal_reference_t;

/* Bounded-speed/acceleration target tracker, not a no-overshoot trajectory
 * planner. Its angle is continuous/unwrapped even when wrap_angle is true. */
bool gimbal_reference_init(gimbal_reference_t *reference,
                          const gimbal_reference_config_t *config,
                          float initial_angle_rad);
bool gimbal_reference_reset(gimbal_reference_t *reference, float angle_rad);
/* output must be separate storage, not &reference->value.
 * Invalid step leaves state unchanged, writes zero output, returns false.
 * The caller must then disable the actuator; never reuse stale output. */
bool gimbal_reference_step(gimbal_reference_t *reference, float target_angle_rad,
                          float dt_s, gimbal_smc_reference_t *output);

#ifdef __cplusplus
}
#endif
#endif
