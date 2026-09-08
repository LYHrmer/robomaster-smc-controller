#include "gimbal_motor.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static gimbal_dm4310_config_t dm_cfg(void)
{
    gimbal_dm4310_config_t c = {
        1u, 0x11u, 1u, 12.5f, 30.0f, 10.0f,
        {1.0f, 1.0f, 2.0f, 1}
    };
    return c;
}

static gimbal_gm6020_config_t gm_cfg(void)
{
    gimbal_gm6020_config_t c = {
        1u, 1u, 0.741f, 0.5f, 0.0f, {1.0f, 1.0f, 1.0f, 1}
    };
    return c;
}

static void assert_empty(const gimbal_can_frame_t *frame)
{
    unsigned i;
    assert(frame->id == 0u && frame->dlc == 0u);
    for (i = 0u; i < 8u; ++i) assert(frame->data[i] == 0u);
}

static void test_dm(void)
{
    gimbal_dm4310_config_t c = dm_cfg();
    gimbal_can_frame_t f;
    gimbal_dm4310_feedback_t fb;
    float applied;
    unsigned i;
    assert(gimbal_dm4310_pack_torque(&c, 1.0f, &f, &applied) == GIMBAL_MOTOR_OK);
    assert(f.id == 1u && f.dlc == 8u);
    /* Independent golden protocol vector: p=0x8000,v=0x800,kp=kd=0,t=2252. */
    {
        const uint8_t expected[8] = {0x80,0,0x80,0,0,0,0x08,0xcc};
        assert(memcmp(f.data, expected, 8u) == 0);
    }
    assert(fabsf(applied - 1.0f) < 0.0025f);
    c.mechanics.direction = -1;
    assert(gimbal_dm4310_pack_torque(&c, 1.0f, &f, &applied) == GIMBAL_MOTOR_OK);
    assert(f.data[6] == 0x07u && f.data[7] == 0x33u);
    assert(applied > 0.99f);
    c.mechanics.gear_ratio = 2.0f;
    c.mechanics.efficiency = 0.8f;
    assert(gimbal_dm4310_pack_torque(&c, FLT_MAX, &f, &applied) == GIMBAL_MOTOR_CLAMPED);
    assert(applied <= 3.20001f && applied > 3.19f);
    assert(gimbal_dm4310_pack_torque(&c, -FLT_MAX, &f, &applied) == GIMBAL_MOTOR_CLAMPED);
    assert(applied >= -3.20001f && applied < -3.19f);
    assert(gimbal_dm4310_pack_torque(&c, NAN, &f, &applied) == GIMBAL_MOTOR_BAD_ARGUMENT);
    assert_empty(&f); assert(applied == 0.0f);
    assert(gimbal_dm4310_pack_torque(&c, INFINITY, &f, &applied) < 0);
    assert_empty(&f);
    c.torque_max_nm = NAN;
    assert(gimbal_dm4310_pack_torque(&c, 0, &f, &applied) == GIMBAL_MOTOR_BAD_CONFIG);
    assert_empty(&f);
    c = dm_cfg();
    c.command_id = 0x801u;
    assert(gimbal_dm4310_pack_enable(&c, 1, &f) == GIMBAL_MOTOR_BAD_CONFIG);
    assert_empty(&f);
    c = dm_cfg();
    assert(gimbal_dm4310_pack_enable(&c, 1, &f) == GIMBAL_MOTOR_OK);
    for (i = 0; i < 7u; ++i) assert(f.data[i] == 0xffu);
    assert(f.data[7] == 0xfcu);
    assert(gimbal_dm4310_pack_enable(&c, 0, &f) == GIMBAL_MOTOR_OK);
    assert(f.data[7] == 0xfdu);
    f.id = 0x11u; f.dlc = 8u;
    { const uint8_t payload[8] = {0x11,0xff,0xff,0,0x0f,0xff,35,36};
      memcpy(f.data, payload, 8u); }
    assert(gimbal_dm4310_decode(&c, &f, &fb) == GIMBAL_MOTOR_OK);
    assert(fb.position_rad == 12.5f && fb.velocity_rad_s == -30.0f && fb.torque_nm == 10.0f);
    f.id = 0x12u;
    assert(gimbal_dm4310_decode(&c, &f, &fb) == GIMBAL_MOTOR_BAD_FRAME);
    assert(fb.torque_nm == 0.0f);
    f.id = 0x11u; f.dlc = 7u;
    assert(gimbal_dm4310_decode(&c, &f, &fb) == GIMBAL_MOTOR_BAD_FRAME);
    f.dlc = 8u; f.data[0] = 0x12u;
    assert(gimbal_dm4310_decode(&c, &f, &fb) == GIMBAL_MOTOR_BAD_FRAME);
    f.data[0] = 0xa1u;
    assert(gimbal_dm4310_decode(&c, &f, &fb) == GIMBAL_MOTOR_FAULT);
    assert(fb.state == 10u && fb.mos_temperature_c == 35u && fb.position_rad == 0.0f);
    f.data[0] = 0x01u;
    assert(gimbal_dm4310_decode(&c, &f, &fb) == GIMBAL_MOTOR_NOT_ENABLED);
    c.feedback_id = 0u; f.id = 0u; f.data[0] = 0x11u;
    assert(gimbal_dm4310_decode(&c, &f, &fb) == GIMBAL_MOTOR_OK);
}

static void test_gm(void)
{
    gimbal_gm6020_config_t c = gm_cfg();
    gimbal_can_frame_t f;
    gimbal_gm6020_feedback_t fb;
    int16_t command;
    int16_t group[4] = {16384, -16384, 1234, -1};
    float applied;
    assert(gimbal_gm6020_torque_to_current(&c, 0.18525f, &command, &applied) == GIMBAL_MOTOR_OK);
    assert(command == 1365 && fabsf(applied - 0.18525f) < 0.00014f);
    c.mechanics.direction = -1;
    assert(gimbal_gm6020_torque_to_current(&c, 0.18525f, &command, &applied) == GIMBAL_MOTOR_OK);
    assert(command == -1365 && applied > 0.185f);
    c.mechanics.gear_ratio = 2.0f; c.mechanics.efficiency = 0.8f;
    assert(gimbal_gm6020_torque_to_current(&c, FLT_MAX, &command, &applied) == GIMBAL_MOTOR_CLAMPED);
    assert(command == -2730 && applied <= 0.5928f && applied > 0.592f);
    c.current_mode_confirmed = 0u;
    assert(gimbal_gm6020_torque_to_current(&c, 1, &command, &applied) == GIMBAL_MOTOR_BAD_CONFIG);
    assert(command == 0 && applied == 0.0f);
    c = gm_cfg(); c.torque_constant_nm_per_a = INFINITY;
    assert(gimbal_gm6020_torque_to_current(&c, 1, &command, &applied) < 0);
    c = gm_cfg();
    assert(gimbal_gm6020_torque_to_current(&c, NAN, &command, &applied) < 0);
    assert(gimbal_gm6020_pack_current_group(1, group, &f) == GIMBAL_MOTOR_OK);
    assert(f.id == 0x1feu);
    { const uint8_t expected[8] = {0x40,0,0xc0,0,0x04,0xd2,0xff,0xff};
      assert(memcmp(f.data, expected, 8u) == 0); }
    assert(gimbal_gm6020_pack_current_group(5, group, &f) < 0); assert_empty(&f);
    group[3] = 0;
    assert(gimbal_gm6020_pack_current_group(5, group, &f) == GIMBAL_MOTOR_OK);
    assert(f.id == 0x2feu);
    group[0] = 25000; group[1] = -25000;
    assert(gimbal_gm6020_pack_current_group(1, group, &f) < 0); assert_empty(&f);
    assert(gimbal_gm6020_pack_voltage_group(1, group, &f) == GIMBAL_MOTOR_OK);
    assert(f.id == 0x1ffu && f.data[0] == 0x61u && f.data[1] == 0xa8u);
    assert(gimbal_gm6020_pack_voltage_group(5, group, &f) == GIMBAL_MOTOR_OK);
    assert(f.id == 0x2ffu);
    group[0] = 25001;
    assert(gimbal_gm6020_pack_voltage_group(1, group, &f) < 0); assert_empty(&f);
    f.id = 0x205u; f.dlc = 8u;
    { const uint8_t payload[8] = {0x10,0,0xff,0xc4,0xfe,0,42,0};
      memcpy(f.data, payload, 8u); }
    assert(gimbal_gm6020_decode(&c, &f, &fb) == GIMBAL_MOTOR_OK);
    assert(fb.encoder == 4096u && fb.speed_rpm == -60 && fb.current_raw == -512);
    assert(fabsf(fb.position_rad - 3.14159265f) < 1e-6f);
    assert(fabsf(fb.velocity_rad_s + 6.2831853f) < 1e-6f);
    assert(fb.current_a_valid == 0u);
    c.feedback_current_a_per_count = 0.001f;
    assert(gimbal_gm6020_decode(&c, &f, &fb) == GIMBAL_MOTOR_OK);
    assert(fb.current_a_valid == 1u && fabsf(fb.current_a + 0.512f) < 1e-6f);
    f.data[0] = 0x20u;
    assert(gimbal_gm6020_decode(&c, &f, &fb) == GIMBAL_MOTOR_BAD_FRAME);
    assert(fb.encoder == 0u);
    f.data[0] = 0x10u; f.id = 0x206u;
    assert(gimbal_gm6020_decode(&c, &f, &fb) == GIMBAL_MOTOR_BAD_FRAME);
    f.id = 0x205u; f.dlc = 7u;
    assert(gimbal_gm6020_decode(&c, &f, &fb) == GIMBAL_MOTOR_BAD_FRAME);
}

int main(void)
{
    test_dm();
    test_gm();
    puts("motor protocol tests passed");
    return 0;
}
