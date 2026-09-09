#include "gimbal_controller_example.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

bool gimbal_example_axis_init(gimbal_example_axis_t *axis,
    const gimbal_smc_config_t *smc, const gimbal_reference_config_t *reference)
{
    if (axis == NULL) return false;
    memset(axis, 0, sizeof(*axis));
    if (!gimbal_smc_init(&axis->controller, smc) ||
        !gimbal_reference_init(&axis->reference, reference, 0.0f)) {
        memset(axis, 0, sizeof(*axis));
        return false;
    }
    return true;
}

bool gimbal_example_axis_step(gimbal_example_axis_t *axis,
    const gimbal_example_snapshot_t *sample, float target_rad,
    float gravity_feedforward_nm, gimbal_smc_output_t *output)
{
    gimbal_smc_input_t in = {0};
    gimbal_smc_reference_t ref;
    if (output == NULL) {
        if (axis != NULL) {
            axis->active = false;
            gimbal_smc_reset(&axis->controller);
        }
        return false;
    }
    memset(output, 0, sizeof(*output));
    if (axis == NULL || sample == NULL) {
        if (axis != NULL) {
            axis->active = false;
            gimbal_smc_reset(&axis->controller);
        }
        output->flags = GIMBAL_SMC_BAD_INPUT;
        return false;
    }
    in.angle_rad = sample->angle_rad;
    in.rate_rad_s = sample->rate_rad_s;
    in.feedback_age_s = sample->feedback_age_s;
    in.dt_s = sample->dt_s;
    in.feedforward_nm = gravity_feedforward_nm;
    in.enable = sample->enabled && sample->feedback_valid;

    if (!in.enable) {
        axis->active = false;
        (void)gimbal_smc_update(&axis->controller, &in, output);
        return false;
    }
    /* Check freshness/period before advancing a stateful reference. The core
     * still validates independently, including finite-value checks. */
    if (!isfinite(in.angle_rad) || !isfinite(in.rate_rad_s) ||
        !isfinite(in.feedback_age_s) || in.feedback_age_s < 0.0f ||
        in.feedback_age_s > axis->controller.config.feedback_timeout_s ||
        !isfinite(in.dt_s) || in.dt_s < axis->controller.config.dt_min_s ||
        in.dt_s > axis->controller.config.dt_max_s) {
        axis->active = false;
        (void)gimbal_smc_update(&axis->controller, &in, output);
        return false;
    }
    if ((!axis->active &&
         !gimbal_reference_reset(&axis->reference, in.angle_rad)) ||
        !gimbal_reference_step(&axis->reference, target_rad, in.dt_s, &ref)) {
        axis->active = false;
        gimbal_smc_reset(&axis->controller);
        output->flags = GIMBAL_SMC_BAD_INPUT;
        return false;
    }
    in.reference_rad = ref.angle_rad;
    in.reference_rate_rad_s = ref.rate_rad_s;
    in.reference_accel_rad_s2 = ref.accel_rad_s2;
    (void)gimbal_smc_update(&axis->controller, &in, output);
    axis->active = output->valid;
    return output->valid;
}

gimbal_motor_status_t gimbal_example_dm_command(
    const gimbal_dm4310_config_t *cfg, gimbal_example_dm_phase_t phase,
    const gimbal_smc_output_t *output,
    gimbal_can_frame_t *frame)
{
    float applied;
    if (frame != NULL) memset(frame, 0, sizeof(*frame));
    /* Enum signedness differs across host/Arm ABIs. One unsigned range check
     * rejects both negative casts and values above the last valid phase. */
    if (frame == NULL || (unsigned int)phase > (unsigned int)GIMBAL_EXAMPLE_DM_FAULT)
        return GIMBAL_MOTOR_BAD_ARGUMENT;
    if (phase == GIMBAL_EXAMPLE_DM_ARMING) return GIMBAL_MOTOR_NOT_ENABLED;
    if (phase != GIMBAL_EXAMPLE_DM_RUNNING || output == NULL || !output->valid)
        return gimbal_dm4310_pack_enable(cfg, 0u, frame);
    return gimbal_dm4310_pack_torque(cfg, output->torque_nm, frame, &applied);
}

gimbal_motor_status_t gimbal_example_gm_slot(
    const gimbal_gm6020_config_t *cfg, const gimbal_smc_output_t *output,
    int16_t commands[4])
{
    unsigned slot;
    float applied;
    if (cfg == NULL || commands == NULL || cfg->motor_id < 1u || cfg->motor_id > 7u)
        return GIMBAL_MOTOR_BAD_ARGUMENT;
    slot = (unsigned)(cfg->motor_id - 1u) % 4u;
    commands[slot] = 0;
    if (output == NULL || !output->valid) return GIMBAL_MOTOR_OK;
    return gimbal_gm6020_torque_to_current(cfg, output->torque_nm,
                                         &commands[slot], &applied);
}
