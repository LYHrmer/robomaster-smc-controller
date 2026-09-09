#include "gimbal_controller_example.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    gimbal_example_axis_t axis;
    gimbal_smc_config_t config;
    const gimbal_reference_config_t rc = {2.0f, 10.0f, 10.0f, 0.0001f, 0.01f, false};
    gimbal_example_snapshot_t s = {0.2f, 0.0f, 0.0f, 0.001f, true, true};
    gimbal_smc_output_t out;
    gimbal_can_frame_t frame;
    const gimbal_dm4310_config_t dm = {1u, 0x11u, 1u, 12.5f, 30.0f, 10.0f,
                                      {1.0f, 1.0f, 0.5f, 1}};
    const gimbal_gm6020_config_t gm = {2u, 1u, 0.741f, 1.0f, 0.0f,
                                      {1.0f, 1.0f, 0.5f, 1}};
    int16_t commands[4] = {111, 222, 333, 444};
    gimbal_smc_default_config(&config);
    assert(gimbal_example_axis_init(&axis, &config, &rc));
    assert(gimbal_example_axis_step(&axis, &s, 0.5f, 0.0f, &out));
    assert(fabsf(out.error_rad) < 0.001f); /* Re-anchor reference at feedback. */
    assert(out.torque_nm > 0.0f);
    assert(gimbal_example_gm_slot(&gm, &out, commands) >= 0);
    assert(commands[0] == 111 && commands[2] == 333 && commands[3] == 444);
    assert(commands[1] > 0);

    s.feedback_age_s = 1.0f;
    assert(!gimbal_example_axis_step(&axis, &s, 0.5f, 0.0f, &out));
    assert(!out.valid && out.torque_nm == 0.0f && !axis.active);
    assert((out.flags & GIMBAL_SMC_STALE_FEEDBACK) != 0u);
    assert(gimbal_example_gm_slot(&gm, &out, commands) == GIMBAL_MOTOR_OK);
    assert(commands[1] == 0 && commands[0] == 111 && commands[2] == 333);
    assert(gimbal_example_dm_command(&dm, GIMBAL_EXAMPLE_DM_RUNNING,
                                     &out, &frame) == GIMBAL_MOTOR_OK);
    assert(frame.data[7] == 0xfdu); /* Disabled, not biased quantized zero. */
    /* Do not undo FC while waiting for the first enabled motor feedback. */
    assert(gimbal_dm4310_pack_enable(&dm, 1u, &frame) == GIMBAL_MOTOR_OK);
    assert(frame.data[7] == 0xfcu);
    assert(gimbal_example_dm_command(&dm, GIMBAL_EXAMPLE_DM_ARMING,
                                     &out, &frame) == GIMBAL_MOTOR_NOT_ENABLED);
    assert(frame.dlc == 0u);
    assert(gimbal_example_dm_command(&dm, (gimbal_example_dm_phase_t)-1,
                                     &out, &frame) == GIMBAL_MOTOR_BAD_ARGUMENT);
    assert(frame.dlc == 0u);
    assert(gimbal_example_dm_command(&dm, (gimbal_example_dm_phase_t)(GIMBAL_EXAMPLE_DM_FAULT + 1),
                                     &out, &frame) == GIMBAL_MOTOR_BAD_ARGUMENT);
    assert(frame.dlc == 0u);

    s.feedback_age_s = 0.0f;
    s.angle_rad = -0.7f;
    assert(gimbal_example_axis_step(&axis, &s, -0.7f, 0.0f, &out));
    assert(fabsf(out.error_rad) < 1e-6f && fabsf(out.torque_nm) < 1e-6f);
    assert(!gimbal_example_axis_step(&axis, &s, NAN, 0.0f, &out));
    assert(!out.valid && !axis.active);
    puts("example: enable, stale feedback, recovery and CAN ownership passed");
    return 0;
}
