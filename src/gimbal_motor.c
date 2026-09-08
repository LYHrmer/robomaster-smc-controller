#include "gimbal_motor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define GM_PI 3.14159265358979323846f

static int positive_finite(float x) { return isfinite(x) && x > 0.0f; }
static float clip(float x, float low, float high)
{ return x < low ? low : (x > high ? high : x); }

static int mechanics_valid(const gimbal_motor_mechanics_t *m)
{
    float gain;
    if (!positive_finite(m->gear_ratio) || !positive_finite(m->efficiency) ||
        m->efficiency > 1.0f || !positive_finite(m->motor_torque_limit_nm) ||
        (m->direction != 1 && m->direction != -1)) return 0;
    gain = m->gear_ratio * m->efficiency;
    return positive_finite(gain) &&
           positive_finite(gain * m->motor_torque_limit_nm);
}

static int dm_valid(const gimbal_dm4310_config_t *c)
{
    return c != NULL && c->command_id <= 0x7ffu &&
        c->feedback_id <= 0x7ffu && c->motor_id <= 15u &&
        (c->command_id & 15u) == c->motor_id &&
        positive_finite(c->position_max_rad) &&
        positive_finite(c->velocity_max_rad_s) && positive_finite(c->torque_max_nm) &&
        mechanics_valid(&c->mechanics) &&
        c->mechanics.motor_torque_limit_nm <= c->torque_max_nm &&
        c->mechanics.motor_torque_limit_nm >= c->torque_max_nm / 4095.0f;
}

static int gm_valid(const gimbal_gm6020_config_t *c)
{
    return c != NULL && c->motor_id >= 1u && c->motor_id <= 7u &&
        c->current_mode_confirmed <= 1u &&
        positive_finite(c->torque_constant_nm_per_a) &&
        positive_finite(c->current_limit_a) && c->current_limit_a <= 3.0f &&
        isfinite(c->feedback_current_a_per_count) &&
        c->feedback_current_a_per_count >= 0.0f &&
        isfinite(c->feedback_current_a_per_count * 32768.0f) &&
        mechanics_valid(&c->mechanics) &&
        positive_finite(c->torque_constant_nm_per_a * c->current_limit_a) &&
        positive_finite(c->torque_constant_nm_per_a * c->current_limit_a *
                        c->mechanics.gear_ratio * c->mechanics.efficiency);
}

static float dm_decode_value(uint16_t value, float max, float count_max)
{ return (((float)value - 0.5f * count_max) / (0.5f * count_max)) * max; }

gimbal_motor_status_t gimbal_dm4310_pack_torque(
    const gimbal_dm4310_config_t *cfg, float joint_torque_nm,
    gimbal_can_frame_t *frame, float *applied_joint_nm)
{
    float gain, joint_limit, bounded_joint, motor_nm, encoded, decoded, applied;
    float code_low, code_high;
    uint16_t torque_code;
    int clamped;
    if (frame != NULL) memset(frame, 0, sizeof(*frame));
    if (applied_joint_nm != NULL) *applied_joint_nm = 0.0f;
    if (frame == NULL || applied_joint_nm == NULL || !isfinite(joint_torque_nm))
        return GIMBAL_MOTOR_BAD_ARGUMENT;
    if (!dm_valid(cfg)) return GIMBAL_MOTOR_BAD_CONFIG;
    gain = cfg->mechanics.gear_ratio * cfg->mechanics.efficiency;
    joint_limit = cfg->mechanics.motor_torque_limit_nm * gain;
    bounded_joint = clip(joint_torque_nm, -joint_limit, joint_limit);
    clamped = bounded_joint != joint_torque_nm;
    motor_nm = bounded_joint / gain * (float)cfg->mechanics.direction;
    encoded = (motor_nm / cfg->torque_max_nm + 1.0f) * 2047.5f;
    code_low = ceilf((1.0f - cfg->mechanics.motor_torque_limit_nm /
                     cfg->torque_max_nm) * 2047.5f);
    code_high = floorf((1.0f + cfg->mechanics.motor_torque_limit_nm /
                       cfg->torque_max_nm) * 2047.5f);
    if (code_low > code_high) return GIMBAL_MOTOR_BAD_CONFIG;
    encoded = floorf(encoded + 0.5f);
    if (encoded < code_low || encoded > code_high) clamped = 1;
    torque_code = (uint16_t)clip(encoded, code_low, code_high);
    decoded = dm_decode_value(torque_code, cfg->torque_max_nm, 4095.0f);
    /* Correct a possible float rounding error at the application limit. */
    if (fabsf(decoded) > cfg->mechanics.motor_torque_limit_nm) {
        torque_code = decoded > 0.0f ? (uint16_t)(torque_code - 1u) :
                                      (uint16_t)(torque_code + 1u);
        decoded = dm_decode_value(torque_code, cfg->torque_max_nm, 4095.0f);
        clamped = 1;
    }
    if (fabsf(decoded) > cfg->mechanics.motor_torque_limit_nm)
        return GIMBAL_MOTOR_BAD_CONFIG;
    applied = decoded * gain * (float)cfg->mechanics.direction;
    if (!isfinite(applied)) return GIMBAL_MOTOR_BAD_CONFIG;
    /* p/v encode nearest zero; kp and kd encode exact zero. */
    frame->id = cfg->command_id;
    frame->dlc = 8u;
    frame->data[0] = 0x80u;
    frame->data[1] = 0x00u;
    frame->data[2] = 0x80u;
    frame->data[3] = 0x00u;
    frame->data[6] = (uint8_t)(torque_code >> 8);
    frame->data[7] = (uint8_t)(torque_code & 0xffu);
    *applied_joint_nm = applied;
    return clamped ? GIMBAL_MOTOR_CLAMPED : GIMBAL_MOTOR_OK;
}

gimbal_motor_status_t gimbal_dm4310_pack_enable(
    const gimbal_dm4310_config_t *cfg, uint8_t enable, gimbal_can_frame_t *frame)
{
    if (frame != NULL) memset(frame, 0, sizeof(*frame));
    if (frame == NULL || enable > 1u) return GIMBAL_MOTOR_BAD_ARGUMENT;
    if (!dm_valid(cfg)) return GIMBAL_MOTOR_BAD_CONFIG;
    frame->id = cfg->command_id;
    frame->dlc = 8u;
    memset(frame->data, 0xff, sizeof(frame->data));
    frame->data[7] = enable != 0u ? 0xfcu : 0xfdu;
    return GIMBAL_MOTOR_OK;
}

gimbal_motor_status_t gimbal_dm4310_decode(
    const gimbal_dm4310_config_t *cfg, const gimbal_can_frame_t *frame,
    gimbal_dm4310_feedback_t *feedback)
{
    uint16_t p, v, t;
    if (feedback != NULL) memset(feedback, 0, sizeof(*feedback));
    if (frame == NULL || feedback == NULL) return GIMBAL_MOTOR_BAD_ARGUMENT;
    if (!dm_valid(cfg)) return GIMBAL_MOTOR_BAD_CONFIG;
    if (frame->id != cfg->feedback_id || frame->dlc != 8u ||
        (frame->data[0] & 0x0fu) != cfg->motor_id) return GIMBAL_MOTOR_BAD_FRAME;
    feedback->state = frame->data[0] >> 4;
    feedback->mos_temperature_c = frame->data[6];
    feedback->rotor_temperature_c = frame->data[7];
    if (feedback->state == 0u) return GIMBAL_MOTOR_NOT_ENABLED;
    if (feedback->state != 1u) return GIMBAL_MOTOR_FAULT;
    p = (uint16_t)((uint16_t)frame->data[1] << 8) | frame->data[2];
    v = (uint16_t)((uint16_t)frame->data[3] << 4) | (frame->data[4] >> 4);
    t = (uint16_t)((uint16_t)(frame->data[4] & 0x0fu) << 8) | frame->data[5];
    feedback->position_rad = dm_decode_value(p, cfg->position_max_rad, 65535.0f);
    feedback->velocity_rad_s = dm_decode_value(v, cfg->velocity_max_rad_s, 4095.0f);
    feedback->torque_nm = dm_decode_value(t, cfg->torque_max_nm, 4095.0f);
    return GIMBAL_MOTOR_OK;
}

gimbal_motor_status_t gimbal_gm6020_torque_to_current(
    const gimbal_gm6020_config_t *cfg, float joint_torque_nm,
    int16_t *command, float *applied_joint_nm)
{
    float gain, current_limit, max_code, joint_limit, bounded, amps, raw;
    int clamped;
    if (command != NULL) *command = 0;
    if (applied_joint_nm != NULL) *applied_joint_nm = 0.0f;
    if (command == NULL || applied_joint_nm == NULL || !isfinite(joint_torque_nm))
        return GIMBAL_MOTOR_BAD_ARGUMENT;
    if (!gm_valid(cfg) || cfg->current_mode_confirmed != 1u)
        return GIMBAL_MOTOR_BAD_CONFIG;
    gain = cfg->mechanics.gear_ratio * cfg->mechanics.efficiency;
    current_limit = fminf(cfg->current_limit_a, cfg->mechanics.motor_torque_limit_nm /
                         cfg->torque_constant_nm_per_a);
    max_code = floorf(current_limit * (16384.0f / 3.0f));
    joint_limit = current_limit * cfg->torque_constant_nm_per_a * gain;
    bounded = clip(joint_torque_nm, -joint_limit, joint_limit);
    clamped = bounded != joint_torque_nm;
    amps = bounded / gain / cfg->torque_constant_nm_per_a *
           (float)cfg->mechanics.direction;
    raw = roundf(amps * (16384.0f / 3.0f));
    if (raw > max_code || raw < -max_code) clamped = 1;
    raw = clip(raw, -max_code, max_code);
    joint_limit = raw * (3.0f / 16384.0f) *
        cfg->torque_constant_nm_per_a * gain * (float)cfg->mechanics.direction;
    if (!isfinite(joint_limit)) return GIMBAL_MOTOR_BAD_CONFIG;
    *command = (int16_t)raw;
    *applied_joint_nm = joint_limit;
    return clamped ? GIMBAL_MOTOR_CLAMPED : GIMBAL_MOTOR_OK;
}

static gimbal_motor_status_t gm_pack_group(uint8_t first, const int16_t commands[4],
    int limit, uint16_t base_id, gimbal_can_frame_t *frame)
{
    unsigned i;
    if (frame != NULL) memset(frame, 0, sizeof(*frame));
    if (commands == NULL || frame == NULL || (first != 1u && first != 5u))
        return GIMBAL_MOTOR_BAD_ARGUMENT;
    for (i = 0; i < 4u; ++i)
        if (commands[i] < -limit || commands[i] > limit)
            return GIMBAL_MOTOR_BAD_ARGUMENT;
    if (first == 5u && commands[3] != 0) return GIMBAL_MOTOR_BAD_ARGUMENT;
    frame->id = (uint16_t)(base_id + (first == 5u ? 0x100u : 0u));
    frame->dlc = 8u;
    for (i = 0; i < 4u; ++i) {
        uint16_t value = (uint16_t)commands[i];
        frame->data[2u * i] = (uint8_t)(value >> 8);
        frame->data[2u * i + 1u] = (uint8_t)(value & 0xffu);
    }
    return GIMBAL_MOTOR_OK;
}

gimbal_motor_status_t gimbal_gm6020_pack_current_group(
    uint8_t first, const int16_t commands[4], gimbal_can_frame_t *frame)
{ return gm_pack_group(first, commands, 16384, 0x1feu, frame); }

gimbal_motor_status_t gimbal_gm6020_pack_voltage_group(
    uint8_t first, const int16_t commands[4], gimbal_can_frame_t *frame)
{ return gm_pack_group(first, commands, 25000, 0x1ffu, frame); }

static int16_t signed_be(const uint8_t *data)
{
    uint16_t u = (uint16_t)((uint16_t)data[0] << 8) | data[1];
    int32_t v = u <= 32767u ? (int32_t)u : (int32_t)u - 65536;
    return (int16_t)v;
}

gimbal_motor_status_t gimbal_gm6020_decode(
    const gimbal_gm6020_config_t *cfg, const gimbal_can_frame_t *frame,
    gimbal_gm6020_feedback_t *feedback)
{
    uint16_t encoder;
    if (feedback != NULL) memset(feedback, 0, sizeof(*feedback));
    if (frame == NULL || feedback == NULL) return GIMBAL_MOTOR_BAD_ARGUMENT;
    if (!gm_valid(cfg)) return GIMBAL_MOTOR_BAD_CONFIG;
    if (frame->id != (uint16_t)(0x204u + cfg->motor_id) || frame->dlc != 8u)
        return GIMBAL_MOTOR_BAD_FRAME;
    encoder = (uint16_t)((uint16_t)frame->data[0] << 8) | frame->data[1];
    if (encoder > 8191u) return GIMBAL_MOTOR_BAD_FRAME;
    feedback->encoder = encoder;
    feedback->speed_rpm = signed_be(&frame->data[2]);
    feedback->current_raw = signed_be(&frame->data[4]);
    feedback->position_rad = (float)encoder * (2.0f * GM_PI / 8192.0f);
    feedback->velocity_rad_s = (float)feedback->speed_rpm * (2.0f * GM_PI / 60.0f);
    feedback->temperature_c = frame->data[6];
    if (cfg->feedback_current_a_per_count > 0.0f) {
        feedback->current_a = (float)feedback->current_raw * cfg->feedback_current_a_per_count;
        feedback->current_a_valid = 1u;
    }
    return GIMBAL_MOTOR_OK;
}
