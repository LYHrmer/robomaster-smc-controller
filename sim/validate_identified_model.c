/* Host-only synthetic validation of an exported candidate model. The actual
 * C99 controller and motor adapters run here. This does not validate hardware,
 * identify actuator dynamics, or qualify parameters for automatic deployment. */
#include "gimbal_smc.h"
#include "gimbal_motor.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef IDENTIFIED_MODEL_HEADER
#error "Compile with IDENTIFIED_MODEL_HEADER pointing to a generated header"
#endif
#include IDENTIFIED_MODEL_HEADER

#define PI_D 3.14159265358979323846
#define DT 0.001
#define STEPS 18000u
#define SUBSTEPS 10u
#define DELAY_SAMPLES 3u

typedef struct {
    double inertia, damping, coulomb, friction_rate, gravity_sin, gravity_cos;
} model_t;

static int number(const char *text, double *out)
{
    char *end;
    errno = 0;
    *out = strtod(text, &end);
    return end != text && *end == '\0' && errno == 0 && isfinite(*out);
}

static void reference(double t, int yaw, double *angle, double *rate, double *accel)
{
    const double w1 = 2.0 * PI_D * 0.23, w2 = 2.0 * PI_D * 0.61;
    const double drift = yaw ? 1.1 : 0.0;
    const double a1 = yaw ? 0.45 : 0.65, a2 = 0.10;
    *angle = drift * t + a1 * sin(w1 * t) + a2 * sin(w2 * t);
    *rate = drift + a1 * w1 * cos(w1 * t) + a2 * w2 * cos(w2 * t);
    *accel = -a1 * w1 * w1 * sin(w1 * t) - a2 * w2 * w2 * sin(w2 * t);
}

static double load(double angle, double rate, const model_t *m, double tilt)
{
    return m->damping * rate + m->coulomb * tanh(rate / m->friction_rate) +
        m->gravity_sin * sin(angle + tilt) + m->gravity_cos * cos(angle + tilt);
}

int main(int argc, char **argv)
{
    model_t truth;
    double parameters[6], angle, rate, unused, torque = 0.0;
    double delay[DELAY_SAMPLES] = {0.0}, sum2 = 0.0, peak = 0.0;
    double upward2 = 0.0, downward2 = 0.0, peak_torque = 0.0;
    unsigned int head = 0u, samples = 0u, saturation = 0u, up = 0u, down = 0u;
    unsigned int i, j;
    int yaw, dm, stress, identified;
    double tilt;
    gimbal_smc_t controller;
    gimbal_smc_config_t config;
    gimbal_dm4310_config_t dm_config = {
        1u, 0x11u, 1u, 12.5f, 30.0f, 10.0f, {1.0f, 1.0f, 1.5f, 1}
    };
    gimbal_gm6020_config_t gm_config = {
        1u, 1u, 0.741f, 2.0f, 0.0f, {1.0f, 1.0f, 1.5f, 1}
    };
    if (argc != 11 || (strcmp(argv[1], "yaw") != 0 && strcmp(argv[1], "pitch") != 0) ||
        (strcmp(argv[2], "dm") != 0 && strcmp(argv[2], "gm") != 0) ||
        (strcmp(argv[3], "nominal") != 0 && strcmp(argv[3], "stress") != 0) ||
        (strcmp(argv[4], "identified") != 0 && strcmp(argv[4], "coarse") != 0)) {
        fprintf(stderr, "usage: %s yaw|pitch dm|gm nominal|stress identified|coarse J B Fc eps A C\n", argv[0]);
        return 2;
    }
    for (i = 0u; i < 6u; ++i) {
        if (!number(argv[5u + i], &parameters[i])) return 2;
    }
    truth.inertia = parameters[0]; truth.damping = parameters[1];
    truth.coulomb = parameters[2]; truth.friction_rate = parameters[3];
    truth.gravity_sin = parameters[4]; truth.gravity_cos = parameters[5];
    if (truth.inertia <= 0.0 || truth.damping < 0.0 || truth.coulomb < 0.0 ||
        truth.friction_rate <= 0.0) return 2;
    yaw = strcmp(argv[1], "yaw") == 0;
    dm = strcmp(argv[2], "dm") == 0;
    stress = strcmp(argv[3], "stress") == 0;
    identified = strcmp(argv[4], "identified") == 0;
    tilt = yaw ? 0.0 : 0.27;
    gimbal_smc_default_config(&config);
    config.inertia_kg_m2 = identified ? smc_ident_inertia_kg_m2 : (float)(truth.inertia * 0.7);
    config.viscous_nm_s_rad = identified ? smc_ident_viscous_nm_s_rad : (float)(truth.damping * 0.5);
    config.lambda_per_s = 8.0f;
    config.reaching_per_s = 8.0f;
    config.robust_rad_s2 = 1.0f;
    config.boundary_rad_s = 0.12f;
    config.torque_limit_nm = 1.45f;
    config.wrap_angle = false;
    if (!gimbal_smc_init(&controller, &config)) return 2;
    if (stress) {
        truth.inertia *= 1.2;
        truth.gravity_sin *= 1.1;
        truth.gravity_cos *= 1.1;
    }
    reference(0.0, yaw, &angle, &rate, &unused);
    for (i = 0u; i < STEPS; ++i) {
        const double t = i * DT;
        double target, target_rate, target_accel, delayed;
        float applied = 0.0f;
        gimbal_can_frame_t frame;
        gimbal_smc_input_t input;
        gimbal_smc_output_t output;
        gimbal_motor_status_t status;
        reference(t, yaw, &target, &target_rate, &target_accel);
        memset(&input, 0, sizeof(input));
        input.angle_rad = (float)(angle + 0.0002 * sin(2.0 * PI_D * 31.0 * t));
        input.rate_rad_s = (float)(rate + 0.002 * sin(2.0 * PI_D * 43.0 * t));
        input.reference_rad = (float)target;
        input.reference_rate_rad_s = (float)target_rate;
        input.reference_accel_rad_s2 = (float)target_accel;
        input.dt_s = (float)DT;
        input.enable = true;
        if (identified) {
            input.feedforward_nm = smc_ident_coulomb_nm * tanhf(input.rate_rad_s / smc_ident_friction_velocity_rad_s) +
                smc_ident_gravity_sin_nm * sinf(input.angle_rad + (float)tilt) +
                smc_ident_gravity_cos_nm * cosf(input.angle_rad + (float)tilt);
        }
        gimbal_smc_update(&controller, &input, &output);
        if (!output.valid) return 1;
        if (dm) {
            status = gimbal_dm4310_pack_torque(&dm_config, output.torque_nm, &frame, &applied);
        } else {
            int16_t commands[4] = {0, 0, 0, 0};
            status = gimbal_gm6020_torque_to_current(&gm_config, output.torque_nm, &commands[0], &applied);
            if (status >= 0 && gimbal_gm6020_pack_current_group(1u, commands, &frame) < 0) return 1;
        }
        if (status < 0 || frame.dlc != 8u || !isfinite(applied)) return 1;
        /* Drive the plant from independently decoded wire bytes, not the
         * adapter's own applied_joint_nm return value. Direct drive, dir=+1. */
        if (dm) {
            const unsigned int code = ((unsigned int)(frame.data[6] & 0x0fu) << 8) | frame.data[7];
            const unsigned int kp = ((unsigned int)(frame.data[3] & 0x0fu) << 8) | frame.data[4];
            const unsigned int kd = ((unsigned int)frame.data[5] << 4) | (frame.data[6] >> 4);
            if (frame.id != 1u || kp != 0u || kd != 0u) return 1;
            applied = (float)((double)code * 20.0 / 4095.0 - 10.0);
        } else {
            const unsigned int bits = ((unsigned int)frame.data[0] << 8) | frame.data[1];
            const int command = bits < 32768u ? (int)bits : (int)bits - 65536;
            if (frame.id != 0x1feu) return 1;
            for (j = 2u; j < 8u; ++j) if (frame.data[j] != 0u) return 1;
            applied = (float)((double)command * 3.0 / 16384.0 * 0.741);
        }
        if (output.flags & GIMBAL_SMC_AMPLITUDE_LIMIT) ++saturation;
        if (status == GIMBAL_MOTOR_CLAMPED) ++saturation;
        peak_torque = fmax(peak_torque, fabs(applied));
        delayed = delay[head]; delay[head] = applied;
        head = (head + 1u) % DELAY_SAMPLES;
        if (i >= 1000u) {
            const double error = angle - target;
            sum2 += error * error; peak = fmax(peak, fabs(error)); ++samples;
            if (target_rate > 0.05) { upward2 += error * error; ++up; }
            if (target_rate < -0.05) { downward2 += error * error; ++down; }
        }
        for (j = 0u; j < SUBSTEPS; ++j) {
            const double dt = DT / SUBSTEPS;
            const double disturbance = stress ? 0.01 * sin(2.0 * PI_D * 1.7 * (t + j * dt)) : 0.0;
            double acceleration;
            torque += dt / 0.003 * (delayed - torque);
            acceleration = (torque - load(angle, rate, &truth, tilt) + disturbance) / truth.inertia;
            angle += rate * dt + 0.5 * acceleration * dt * dt;
            rate += acceleration * dt;
        }
        if (!isfinite(angle) || !isfinite(rate) || fabs(rate) > 100.0) return 1;
    }
    printf("{\"axis\":\"%s\",\"motor\":\"%s\",\"condition\":\"%s\",\"model\":\"%s\","
           "\"rms_rad\":%.10g,\"peak_rad\":%.10g,\"up_samples\":%u,\"down_samples\":%u,\"up_rms_rad\":",
           argv[1], dm ? "DM4310" : "GM6020", argv[3], argv[4], sqrt(sum2 / samples), peak, up, down);
    if (up) printf("%.10g", sqrt(upward2 / up)); else printf("null");
    printf(",\"down_rms_rad\":");
    if (down) printf("%.10g", sqrt(downward2 / down)); else printf("null");
    printf(",\"saturation_events\":%u,\"peak_torque_nm\":%.10g,\"final_angle_rad\":%.10g}",
           saturation, peak_torque, angle);
    /* These are declared synthetic acceptance gates, not hardware accuracy. */
    if (identified && (sqrt(sum2 / samples) > 0.035 || peak > 0.10 || saturation > 100u)) return 1;
    if (yaw && angle < 4.0 * PI_D) return 1;
    return 0;
}
