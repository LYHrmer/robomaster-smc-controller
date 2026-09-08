#ifndef GIMBAL_CONTROLLER_EXAMPLE_H
#define GIMBAL_CONTROLLER_EXAMPLE_H

#include "gimbal_motor.h"
#include "gimbal_smc.h"

/* Instantiate once for yaw and once for pitch. Configurations are per axis. */
typedef struct {
    gimbal_smc_t controller;
    gimbal_reference_t reference;
    bool active;
} gimbal_example_axis_t;

typedef struct {
    float angle_rad;       /* Official feedback-updated absolute_angle. */
    float rate_rad_s;      /* Official feedback-updated motor_gyro. */
    float feedback_age_s;  /* Max age of all required motor/INS data. */
    float dt_s;
    bool feedback_valid;   /* Coherent snapshot, no motor fault. */
    bool enabled;          /* Upper mode/limit/remote-control supervisor. */
} gimbal_example_snapshot_t;

bool gimbal_example_axis_init(gimbal_example_axis_t *axis,
    const gimbal_smc_config_t *smc, const gimbal_reference_config_t *reference);
/* Position-target example. On first enable, start the reference at measured
 * angle. On any fault, clear active; the next valid enable re-anchors it.
 * The upper mode manager decides whether a previous target may be reused. */
bool gimbal_example_axis_step(gimbal_example_axis_t *axis,
    const gimbal_example_snapshot_t *snapshot, float target_rad,
    float gravity_feedforward_nm, gimbal_smc_output_t *output);

typedef enum {
    GIMBAL_EXAMPLE_DM_DISABLED,
    GIMBAL_EXAMPLE_DM_ARMING,
    GIMBAL_EXAMPLE_DM_RUNNING,
    GIMBAL_EXAMPLE_DM_FAULT
} gimbal_example_dm_phase_t;

/* Upper manager sends FC on entry to ARMING and awaits fresh state=1 feedback.
 * ARMING returns NOT_ENABLED with dlc=0: no command, so this helper cannot
 * cancel an in-flight enable. Timeout/fault -> FAULT; only then send FD.
 * A faulted output in RUNNING generates FD. No automatic FC/re-enable. */
gimbal_motor_status_t gimbal_example_dm_command(
    const gimbal_dm4310_config_t *cfg, gimbal_example_dm_phase_t phase,
    const gimbal_smc_output_t *output,
    gimbal_can_frame_t *frame);
/* Only writes this motor's slot. The CAN-group owner supplies all other slots
 * and calls pack_current_group ONCE per cycle. No implicit transmission. */
gimbal_motor_status_t gimbal_example_gm_slot(
    const gimbal_gm6020_config_t *cfg, const gimbal_smc_output_t *output,
    int16_t commands[4]);

#endif
