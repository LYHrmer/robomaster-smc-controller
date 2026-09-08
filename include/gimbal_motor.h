#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A standard CAN DATA frame. The application must reject RTR/extended frames. */
typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} gimbal_can_frame_t;

typedef enum {
    GIMBAL_MOTOR_OK = 0,
    GIMBAL_MOTOR_CLAMPED = 1,
    GIMBAL_MOTOR_BAD_ARGUMENT = -1,
    GIMBAL_MOTOR_BAD_CONFIG = -2,
    GIMBAL_MOTOR_BAD_FRAME = -3,
    GIMBAL_MOTOR_NOT_ENABLED = -4,
    GIMBAL_MOTOR_FAULT = -5
} gimbal_motor_status_t;

/* gear_ratio = actuator module OUTPUT-shaft speed / joint speed; external
 * transmission only, directly coupled = 1. DM4310 internal gearing is already
 * included in its MIT output-shaft torque/velocity: do not apply it again.
 * direction is exactly +1 or -1. Torque limit is at the module output shaft;
 * limits are application limits, not ratings. */
typedef struct {
    float gear_ratio;
    float efficiency;
    float motor_torque_limit_nm;
    int8_t direction;
} gimbal_motor_mechanics_t;

typedef struct {
    uint16_t command_id;
    uint16_t feedback_id; /* DM MASTER_ID, independently configured. */
    uint8_t motor_id;     /* Low nibble in feedback payload, 0..15. */
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;  /* Read PMAX/VMAX/TMAX from this motor's settings. */
    gimbal_motor_mechanics_t mechanics;
} gimbal_dm4310_config_t;

typedef struct {
    float position_rad;   /* Motor coordinates; may wrap at configured PMAX. */
    float velocity_rad_s;
    float torque_nm;
    uint8_t state;        /* 0 disabled, 1 enabled, 8..14 faults. */
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
} gimbal_dm4310_feedback_t;

/* Pure MIT feed-forward torque: p=v=kp=kd=0. Quantization is unavoidable.
 * On negative status, frame and applied_joint_nm are cleared: do not transmit.
 * applied_joint_nm is the decoded quantized command in joint coordinates. */
gimbal_motor_status_t gimbal_dm4310_pack_torque(
    const gimbal_dm4310_config_t *cfg, float joint_torque_nm,
    gimbal_can_frame_t *frame, float *applied_joint_nm);

/* Generates enable/disable only. Caller controls when/if transmission is safe. */
gimbal_motor_status_t gimbal_dm4310_pack_enable(
    const gimbal_dm4310_config_t *cfg, uint8_t enable,
    gimbal_can_frame_t *frame);

/* Fault/not-enabled returns negative and only preserves diagnostic state/temps. */
gimbal_motor_status_t gimbal_dm4310_decode(
    const gimbal_dm4310_config_t *cfg, const gimbal_can_frame_t *frame,
    gimbal_dm4310_feedback_t *feedback);

typedef struct {
    uint8_t motor_id; /* 1..7 */
    uint8_t current_mode_confirmed; /* Must be 1 after firmware/setup verification. */
    float torque_constant_nm_per_a;
    float current_limit_a; /* >0, <=3 A; choose according to thermal/mechanical limits. */
    /* 0 means unknown: feedback current remains raw. Not inferred from command scale. */
    float feedback_current_a_per_count;
    gimbal_motor_mechanics_t mechanics;
} gimbal_gm6020_config_t;

typedef struct {
    uint16_t encoder;
    int16_t speed_rpm;
    int16_t current_raw;
    float position_rad;   /* Motor coordinates, [0,2*pi). */
    float velocity_rad_s;
    float current_a;
    uint8_t current_a_valid;
    uint8_t temperature_c;
} gimbal_gm6020_feedback_t;

/* Converts joint Nm to one current command; owner combines all four slots. */
gimbal_motor_status_t gimbal_gm6020_torque_to_current(
    const gimbal_gm6020_config_t *cfg, float joint_torque_nm,
    int16_t *command, float *applied_joint_nm);

/* first_motor_id must be 1 or 5. For group 5, commands[3] must be 0.
 * All four values must be supplied by a single CAN-group owner each cycle.
 * Current range +/-16384; voltage range +/-25000 (raw, not Nm or volts). */
gimbal_motor_status_t gimbal_gm6020_pack_current_group(
    uint8_t first_motor_id, const int16_t commands[4], gimbal_can_frame_t *frame);
gimbal_motor_status_t gimbal_gm6020_pack_voltage_group(
    uint8_t first_motor_id, const int16_t commands[4], gimbal_can_frame_t *frame);
gimbal_motor_status_t gimbal_gm6020_decode(
    const gimbal_gm6020_config_t *cfg, const gimbal_can_frame_t *frame,
    gimbal_gm6020_feedback_t *feedback);

#ifdef __cplusplus
}
#endif
#endif
