/* Synthetic end-to-end online fitting. Actual C SMC, protocol packers and RLS
 * run at their declared rates; joint torque feedback is known by construction.
 * This is not a hardware calibration or an unbiased closed-loop LS proof. */
#include "gimbal_identification.h"
#include "gimbal_motor.h"
#include "gimbal_smc.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI_D 3.14159265358979323846
#define DT 0.001
#define STEPS 44000u
#define DELAY 3u

typedef struct { double j, b, fc, eps, a, c; } model_t;
typedef struct {
    double rms, peak, model_error, apply_time;
    gimbal_identification_model_t candidate;
    unsigned int updates, pause_checks;
} result_t;

static void reference(double t, int pitch, double *q, double *v, double *a)
{
    const double frequencies[3] = {0.19, 0.63, 1.07};
    const double amplitudes[3] = {0.50, 0.12, 0.04};
    const double drift = pitch ? 0.0 : 0.70;
    unsigned int i;
    *q = drift * t; *v = drift; *a = 0.0;
    for (i = 0; i < 3u; ++i) {
        /* Held-out closed-loop trajectory starts at 28 s; scoring at 30 s. */
        const double w = 2.0 * PI_D * (frequencies[i] + (t >= 28.0 ? 0.08 : 0.0));
        const double phase = w * t + 0.2 * i;
        *q += amplitudes[i] * sin(phase);
        *v += amplitudes[i] * w * cos(phase);
        *a -= amplitudes[i] * w * w * sin(phase);
    }
}

static double load(double q, double v, double tilt, const model_t *m)
{
    return m->b * v + m->fc * tanh(v / m->eps) +
        m->a * sin(q + tilt) + m->c * cos(q + tilt);
}

static int run(int pitch, int dm, int apply, result_t *result)
{
    const model_t truth = {pitch ? 0.012 : 0.018, pitch ? 0.011 : 0.008,
        pitch ? 0.025 : 0.045, pitch ? 0.075 : 0.09,
        pitch ? 0.055 : 0.0, pitch ? 0.19 : 0.0};
    const double tilt = pitch ? 0.21 : 0.0;
    gimbal_identification_config_t ic;
    static gimbal_identification_t identification; /* MCU allocation is static too. */
    gimbal_smc_config_t cc;
    gimbal_smc_t controller;
    gimbal_identification_model_t active = {0.02f, 0.02f, 0.015f, 0.09f, 0.02f, 0.10f};
    gimbal_dm4310_config_t dc = {1u, 0x11u, 1u, 12.5f, 30.0f, 10.0f, {1.0f, 1.0f, 1.5f, 1}};
    gimbal_gm6020_config_t gc = {1u, 1u, 0.741f, 2.0f, 0.0f, {1.0f, 1.0f, 1.5f, 1}};
    double q, v, unused, actuator = 0.0, delay[DELAY] = {0.0};
    double sum2 = 0.0, peak = 0.0;
    float paused_theta[5], paused_p[5][5];
    unsigned int i, sub, head = 0u, count = 0u;
    int captured = 0, previous_saturation = 0;
    memset(result, 0, sizeof(*result));
    gimbal_identification_default_config(&ic, pitch ? GIMBAL_IDENTIFICATION_PITCH : GIMBAL_IDENTIFICATION_YAW);
    ic.configuration_id = 42u;
    ic.friction_velocity_rad_s = (float)truth.eps;
    ic.window_s = 0.05f; ic.max_gap_s = 0.01f;
    ic.rls.dt_min_s = 0.049f; ic.rls.dt_max_s = 0.061f;
    ic.rls.covariance_initial = 1000.0f; ic.rls.covariance_limit = 100000.0f;
    ic.rls.forgetting_time_s = 20.0f;
    ic.rls.excitation_window = 32u; ic.rls.min_updates = 20u;
    ic.rls.min_feature_energy = 1e-5f; ic.rls.min_excitation_pivot = 1e-5f;
    ic.rls.innovation_rms_limit = 0.015f;
    ic.rls.output_scale = 1.0f;
    for (i = 0u; i < ic.rls.parameter_count; ++i) ic.rls.feature_scale[i] = i == 0u ? 5.0f : (i == 1u ? 2.0f : 1.0f);
    ic.rls.initial[0] = active.inertia_kg_m2;
    ic.rls.initial[1] = active.viscous_nm_s_rad;
    ic.rls.initial[2] = active.coulomb_nm;
    ic.rls.initial[3] = active.gravity_sin_nm;
    ic.rls.initial[4] = active.gravity_cos_nm;
    ic.rls.minimum[0] = 0.003f; ic.rls.maximum[0] = 0.08f;
    ic.rls.minimum[1] = 0.0f; ic.rls.maximum[1] = 0.1f;
    ic.rls.minimum[2] = 0.0f; ic.rls.maximum[2] = 0.2f;
    ic.rls.minimum[3] = -0.4f; ic.rls.maximum[3] = 0.4f;
    ic.rls.minimum[4] = -0.4f; ic.rls.maximum[4] = 0.4f;
    active.friction_velocity_rad_s = (float)truth.eps;
    if (!pitch) { active.gravity_sin_nm = 0.0f; active.gravity_cos_nm = 0.0f; }
    if (!gimbal_identification_init(&identification, &ic)) return 0;
    gimbal_smc_default_config(&cc);
    cc.inertia_kg_m2 = active.inertia_kg_m2; cc.viscous_nm_s_rad = active.viscous_nm_s_rad;
    cc.lambda_per_s = 8.0f; cc.reaching_per_s = 8.0f;
    cc.robust_rad_s2 = 1.0f; cc.boundary_rad_s = 0.12f; cc.torque_limit_nm = 1.45f;
    if (!gimbal_smc_init(&controller, &cc)) return 0;
    reference(0.0, pitch, &q, &v, &unused);
    for (i = 0u; i < STEPS; ++i) {
        const double t = i * DT;
        double target, target_v, target_a, wire, delayed;
        gimbal_identification_sample_t sample;
        gimbal_identification_output_t estimate;
        gimbal_smc_input_t input;
        gimbal_smc_output_t output;
        gimbal_can_frame_t frame;
        gimbal_motor_status_t status;
        float applied;
        memset(&sample, 0, sizeof(sample));
        sample.timestamp_us = UINT32_MAX - 20000u + i * 1000u;
        sample.configuration_id = 42u;
        sample.angle_rad = (float)(q + 0.00005 * sin(2.0 * PI_D * 31.0 * t));
        sample.rate_rad_s = (float)(v + 0.0002 * sin(2.0 * PI_D * 37.0 * t));
        sample.gravity_angle_rad = sample.angle_rad + (float)tilt;
        sample.torque_nm = (float)(actuator + 0.0001 * sin(2.0 * PI_D * 53.0 * t));
        sample.enabled = i < 12000u || i >= 14000u; /* Pause only the estimator. */
        sample.valid = true; sample.saturated = previous_saturation != 0;
        sample.operating_conditions_valid = sample.torque_calibrated = sample.torque_time_aligned = true;
        if (i == 12000u) {
            memcpy(paused_theta, identification.estimator.theta, sizeof(paused_theta));
            memcpy(paused_p, identification.estimator.covariance, sizeof(paused_p));
        }
        gimbal_identification_update(&identification, &sample, &estimate);
        if (!sample.enabled) {
            if (estimate.ready || estimate.window_updated ||
                memcmp(paused_theta, identification.estimator.theta, sizeof(paused_theta)) ||
                memcmp(paused_p, identification.estimator.covariance, sizeof(paused_p))) return 0;
            ++result->pause_checks;
        }
        if (t >= 24.0 && !captured && estimate.ready) {
            result->candidate = estimate.model; result->apply_time = t; captured = 1;
            /* Explicit simulator decision, NOT a side effect of RLS update. */
            if (apply) {
                active = estimate.model;
                cc.inertia_kg_m2 = active.inertia_kg_m2;
                cc.viscous_nm_s_rad = active.viscous_nm_s_rad;
                if (!gimbal_smc_init(&controller, &cc)) return 0;
            }
        }
        reference(t, pitch, &target, &target_v, &target_a);
        memset(&input, 0, sizeof(input));
        input.angle_rad = sample.angle_rad; input.rate_rad_s = sample.rate_rad_s;
        input.reference_rad = (float)target; input.reference_rate_rad_s = (float)target_v;
        input.reference_accel_rad_s2 = (float)target_a; input.dt_s = (float)DT; input.enable = true;
        input.feedforward_nm = active.coulomb_nm * tanhf(input.rate_rad_s / active.friction_velocity_rad_s) +
            active.gravity_sin_nm * sinf(sample.gravity_angle_rad) + active.gravity_cos_nm * cosf(sample.gravity_angle_rad);
        gimbal_smc_update(&controller, &input, &output);
        if (!output.valid) return 0;
        if (dm) {
            unsigned int code;
            status = gimbal_dm4310_pack_torque(&dc, output.torque_nm, &frame, &applied);
            if (status < 0 || frame.id != 1u || frame.dlc != 8u) return 0;
            code = ((unsigned int)(frame.data[6] & 0x0fu) << 8) | frame.data[7];
            wire = code * 20.0 / 4095.0 - 10.0;
        } else {
            int16_t commands[4] = {0, 0, 0, 0};
            unsigned int bits;
            int command;
            status = gimbal_gm6020_torque_to_current(&gc, output.torque_nm, &commands[0], &applied);
            if (status < 0 || gimbal_gm6020_pack_current_group(1u, commands, &frame) < 0 || frame.id != 0x1feu) return 0;
            bits = ((unsigned int)frame.data[0] << 8) | frame.data[1];
            command = bits < 32768u ? (int)bits : (int)bits - 65536;
            wire = command * 3.0 / 16384.0 * 0.741;
        }
        previous_saturation = (output.flags & GIMBAL_SMC_AMPLITUDE_LIMIT) || status == GIMBAL_MOTOR_CLAMPED;
        delayed = delay[head]; delay[head] = wire; head = (head + 1u) % DELAY;
        if (i >= 30000u) { const double error = q - target; sum2 += error * error; peak = fmax(peak, fabs(error)); ++count; }
        for (sub = 0u; sub < 10u; ++sub) {
            const double dt = DT / 10.0;
            double acceleration;
            actuator += dt / 0.003 * (delayed - actuator);
            acceleration = (actuator - load(q, v, tilt, &truth)) / truth.j;
            q += v * dt + 0.5 * acceleration * dt * dt;
            v += acceleration * dt;
        }
        if (!isfinite(q) || !isfinite(v) || fabs(v) > 100.0) return 0;
    }
    result->updates = identification.estimator.accepted_updates;
    result->rms = sqrt(sum2 / count); result->peak = peak;
    if (!captured || result->apply_time > 27.0 || result->pause_checks != 2000u) return 0;
    result->model_error = fabs(result->candidate.inertia_kg_m2 / truth.j - 1.0);
    result->model_error = fmax(result->model_error, fabs(result->candidate.viscous_nm_s_rad / truth.b - 1.0));
    result->model_error = fmax(result->model_error, fabs(result->candidate.coulomb_nm / truth.fc - 1.0));
    if (pitch) {
        result->model_error = fmax(result->model_error, fabs(result->candidate.gravity_sin_nm / truth.a - 1.0));
        result->model_error = fmax(result->model_error, fabs(result->candidate.gravity_cos_nm / truth.c - 1.0));
    }
    return result->model_error < 0.12 && (!apply || (result->rms < 0.02 && result->peak < 0.06)) && (pitch || q > 4.0 * PI_D);
}

int main(int argc, char **argv)
{
    FILE *file;
    int pitch, dm;
    if (argc != 2 || (file = fopen(argv[1], "w")) == NULL) return 2;
    fprintf(file, "axis,motor,parameters,rms_rad,peak_rad,candidate_max_relative_error,ready_time_s,accepted_updates,pause_freeze_checks,J,B,Fc,A,C\n");
    for (pitch = 0; pitch <= 1; ++pitch) for (dm = 0; dm <= 1; ++dm) {
        result_t baseline, updated;
        int apply;
        for (apply = 0; apply <= 1; ++apply) {
            result_t *r = apply ? &updated : &baseline;
            if (!run(pitch, dm, apply, r)) {
                fprintf(stderr, "online synthetic FAIL axis=%s motor=%s apply=%d rms=%g model_error=%g time=%g updates=%u\n",
                        pitch ? "pitch" : "yaw", dm ? "DM4310" : "GM6020", apply, r->rms, r->model_error, r->apply_time, r->updates);
                fclose(file); return 1;
            }
            fprintf(file, "%s,%s,%s,%.10g,%.10g,%.10g,%.6f,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    pitch ? "pitch" : "yaw", dm ? "DM4310" : "GM6020", apply ? "accepted_candidate" : "initial_model",
                    r->rms, r->peak, r->model_error, r->apply_time, r->updates, r->pause_checks,
                    r->candidate.inertia_kg_m2, r->candidate.viscous_nm_s_rad, r->candidate.coulomb_nm,
                    r->candidate.gravity_sin_nm, r->candidate.gravity_cos_nm);
        }
        if (updated.rms >= baseline.rms) { fclose(file); return 1; }
        printf("%s/%s candidate RMS %.6g rad, initial %.6g rad; model error %.3g%%\n",
               pitch ? "pitch" : "yaw", dm ? "DM4310" : "GM6020", updated.rms, baseline.rms, updated.model_error * 100.0);
    }
    if (fclose(file) != 0) return 1;
    printf("PASS: four online candidates and four initial-model baselines; fixed-base synthetic only\n");
    return 0;
}
