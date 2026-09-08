/* Host-only model regression using the actual portable C control/motor code.
 * Mechanical parameters come from the input CSV; actuator dynamics, sensor
 * ripple, friction, disturbance and 1 N m limits below are declared assumptions.
 * This does not execute an MCU, CAN bus, full multibody robot or real motor. */
#include "gimbal_motor.h"
#include "gimbal_smc.h"

#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_D 3.14159265358979323846
#define PLANT_DT 0.0001
#define DURATION 6.0
#define DELAY_STEPS 10u
#define ACTUATOR_LAG 0.003
#define DAMPING 0.01
#define APPLICATION_TORQUE_LIMIT 1.0
#define MAX_PROFILES 64u
#define NAME_CAP 64u
#define HEADER "model,axis,inertia_kg_m2,gravity_sin_nm,gravity_cos_nm,min_rad,max_rad"

typedef struct {
    char model[NAME_CAP];
    char axis[NAME_CAP];
    double inertia, gravity_sin, gravity_cos, min_angle, max_angle;
} profile_t;

typedef struct { double angle, rate, torque; } state_t;

typedef struct {
    state_t value;
    double delay[DELAY_STEPS];
    unsigned int head;
} plant_t;

typedef struct {
    const profile_t *profile;
    unsigned int hz;
    double inertia_scale, gravity_ff_scale;
    int dm, terminal, sine;
} case_t;

typedef struct {
    double error_squared, evaluation_error_squared, peak_error, evaluation_peak;
    double peak_request, peak_wire, peak_actuator, total_variation;
    double min_angle, max_angle, final_error;
    unsigned int samples, evaluation_samples, saturated_samples, adapter_clamps;
} metrics_t;

static const char *motor_name(const case_t *c) { return c->dm ? "DM4310" : "GM6020"; }
static const char *controller_name(const case_t *c) { return c->terminal ? "regularized_terminal" : "linear"; }
static const char *reference_name(const case_t *c) { return c->sine ? "sine" : "hold"; }

static char *trim(char *text)
{
    char *end;
    while (isspace((unsigned char)*text)) ++text;
    end = text + strlen(text);
    while (end > text && isspace((unsigned char)end[-1])) --end;
    *end = '\0';
    return text;
}

static int parse_number(char *text, double *value)
{
    char *end;
    text = trim(text);
    if (*text == '\0') return 0;
    errno = 0;
    *value = strtod(text, &end);
    return errno != ERANGE && end != text && *end == '\0' &&
           isfinite(*value) && fabs(*value) <= FLT_MAX;
}

static int parse_name(char *text, char output[NAME_CAP])
{
    size_t i;
    text = trim(text);
    if (*text == '\0' || strlen(text) >= NAME_CAP) return 0;
    for (i = 0u; text[i] != '\0'; ++i) {
        if (!isalnum((unsigned char)text[i]) && text[i] != '_' &&
            text[i] != '-' && text[i] != '.') return 0;
    }
    strcpy(output, text);
    return 1;
}

static int read_profiles(const char *path, profile_t profiles[MAX_PROFILES], size_t *count)
{
    FILE *file = fopen(path, "r");
    char line[1024];
    unsigned int line_number = 0u;
    int ok = 1;
    *count = 0u;
    if (file == NULL) { perror(path); return 0; }
    if (fgets(line, sizeof(line), file) == NULL || strcmp(trim(line), HEADER) != 0) {
        fprintf(stderr, "%s: missing or incorrect CSV header\n", path);
        ok = 0;
    }
    line_number = 1u;
    while (ok && fgets(line, sizeof(line), file) != NULL) {
        char *fields[7], *cursor, *comma;
        profile_t candidate;
        size_t i;
        ++line_number;
        if (strchr(line, '\n') == NULL && !feof(file)) {
            fprintf(stderr, "%s:%u: line too long\n", path, line_number);
            ok = 0; break;
        }
        cursor = trim(line);
        if (*cursor == '\0') continue;
        if (*count >= MAX_PROFILES) {
            fprintf(stderr, "%s: more than %u profiles\n", path, MAX_PROFILES);
            ok = 0; break;
        }
        memset(&candidate, 0, sizeof(candidate));
        for (i = 0u; i < 7u; ++i) {
            fields[i] = cursor;
            comma = strchr(cursor, ',');
            if ((i < 6u && comma == NULL) || (i == 6u && comma != NULL)) {
                ok = 0; break;
            }
            if (comma != NULL) { *comma = '\0'; cursor = comma + 1; }
        }
        if (ok) {
            ok = parse_name(fields[0], candidate.model) && parse_name(fields[1], candidate.axis) &&
                 parse_number(fields[2], &candidate.inertia) &&
                 parse_number(fields[3], &candidate.gravity_sin) &&
                 parse_number(fields[4], &candidate.gravity_cos) &&
                 parse_number(fields[5], &candidate.min_angle) &&
                 parse_number(fields[6], &candidate.max_angle) &&
                 candidate.inertia > 0.0 && (float)candidate.inertia > 0.0f &&
                 candidate.min_angle < candidate.max_angle;
        }
        if (!ok) {
            fprintf(stderr, "%s:%u: invalid fields, finite numbers or physical range\n", path, line_number);
            break;
        }
        for (i = 0u; i < *count; ++i) {
            if (strcmp(candidate.model, profiles[i].model) == 0 &&
                strcmp(candidate.axis, profiles[i].axis) == 0) {
                fprintf(stderr, "%s:%u: duplicate model/axis\n", path, line_number);
                ok = 0; break;
            }
        }
        if (ok) profiles[(*count)++] = candidate;
    }
    if (ferror(file)) { perror(path); ok = 0; }
    if (fclose(file) != 0) { perror(path); ok = 0; }
    if (ok && *count == 0u) { fprintf(stderr, "%s: empty profile table\n", path); ok = 0; }
    return ok;
}

static double gravity(const profile_t *p, double angle)
{ return p->gravity_sin * sin(angle) + p->gravity_cos * cos(angle); }

static void reference_at(const case_t *c, double time, double *q, double *v, double *a)
{
    if (c->sine) {
        *q = 0.1 * sin(PI_D * time);
        *v = 0.1 * PI_D * cos(PI_D * time);
        *a = -0.1 * PI_D * PI_D * sin(PI_D * time);
    } else { *q = *v = *a = 0.0; }
}

static double disturbance(double time)
{
    return time < 2.0 ? 0.0 : -0.03 + 0.02 * sin(2.0 * PI_D * 1.3 * (time - 2.0));
}

/* Independent command decoding intentionally does NOT use applied_joint_nm.
 * DM decoding checks all actual MIT p/v/kp/kd fields; GM decoding checks the
 * packed group's bytes, signed command and untouched slots before conversion. */
static int wire_torque(int dm, float requested, double *decoded, int *clamped)
{
    gimbal_can_frame_t frame;
    float adapter_applied;
    gimbal_motor_status_t status;
    if (dm) {
        const gimbal_dm4310_config_t cfg = {1u, 0x11u, 1u, 12.5f, 30.0f, 10.0f,
                                          {1.0f, 1.0f, 1.0f, 1}};
        unsigned int code;
        status = gimbal_dm4310_pack_torque(&cfg, requested, &frame, &adapter_applied);
        if (status < 0 || frame.id != 1u || frame.dlc != 8u ||
            frame.data[0] != 0x80u || frame.data[1] != 0u ||
            frame.data[2] != 0x80u || frame.data[3] != 0u ||
            frame.data[4] != 0u || frame.data[5] != 0u ||
            (frame.data[6] & 0xf0u) != 0u) return 0;
        code = ((unsigned int)(frame.data[6] & 0x0fu) << 8) | frame.data[7];
        *decoded = -10.0 + (20.0 / 4095.0) * (double)code;
    } else {
        const gimbal_gm6020_config_t cfg = {1u, 1u, 0.741f, 1.0f / 0.741f, 0.0f,
                                          {1.0f, 1.0f, 1.0f, 1}};
        int16_t commands[4] = {0, 0, 0, 0};
        unsigned int raw, i;
        int signed_code;
        status = gimbal_gm6020_torque_to_current(&cfg, requested, &commands[0], &adapter_applied);
        if (status < 0 || gimbal_gm6020_pack_current_group(1u, commands, &frame) < 0 ||
            frame.id != 0x1feu || frame.dlc != 8u) return 0;
        for (i = 2u; i < 8u; ++i) if (frame.data[i] != 0u) return 0;
        raw = ((unsigned int)frame.data[0] << 8) | frame.data[1];
        signed_code = raw <= 32767u ? (int)raw : (int)raw - 65536;
        if (signed_code != commands[0] || abs(signed_code) > 16384) return 0;
        *decoded = (double)signed_code * (3.0 / 16384.0) * 0.741;
    }
    *clamped = status == GIMBAL_MOTOR_CLAMPED;
    return isfinite(*decoded) && fabs(*decoded) <= APPLICATION_TORQUE_LIMIT + 1e-9;
}

static state_t rhs(state_t s, const case_t *c, double command, double time)
{
    state_t d;
    d.angle = s.rate;
    d.rate = (s.torque - DAMPING * s.rate - gravity(c->profile, s.angle) +
              disturbance(time)) / (c->profile->inertia * c->inertia_scale);
    d.torque = (command - s.torque) / ACTUATOR_LAG;
    return d;
}

static state_t advance(state_t s, state_t d, double step)
{
    s.angle += step * d.angle;
    s.rate += step * d.rate;
    s.torque += step * d.torque;
    return s;
}

static void plant_step(plant_t *p, const case_t *c, double command, double time)
{
    const double delayed = p->delay[p->head];
    state_t k1, k2, k3, k4;
    p->delay[p->head] = command;
    p->head = (p->head + 1u) % DELAY_STEPS;
    k1 = rhs(p->value, c, delayed, time);
    k2 = rhs(advance(p->value, k1, PLANT_DT * 0.5), c, delayed, time + PLANT_DT * 0.5);
    k3 = rhs(advance(p->value, k2, PLANT_DT * 0.5), c, delayed, time + PLANT_DT * 0.5);
    k4 = rhs(advance(p->value, k3, PLANT_DT), c, delayed, time + PLANT_DT);
    p->value.angle += PLANT_DT * (k1.angle + 2.0 * k2.angle + 2.0 * k3.angle + k4.angle) / 6.0;
    p->value.rate += PLANT_DT * (k1.rate + 2.0 * k2.rate + 2.0 * k3.rate + k4.rate) / 6.0;
    p->value.torque += PLANT_DT * (k1.torque + 2.0 * k2.torque + 2.0 * k3.torque + k4.torque) / 6.0;
}

/* Necessary trajectory feasibility test, not proof of closed-loop feasibility.
 * We sample the exact analytic inverse-dynamics demand every 0.1 ms. A sampled
 * demand above available torque proves this trajectory infeasible; a lower
 * value cannot guarantee all transients or inter-sample extrema are feasible. */
static const char *preflight(const case_t *c, double *required_peak, double *available)
{
    double positive, negative;
    int ignored;
    unsigned int i;
    const double amplitude = c->sine ? 0.1 : 0.0;
    *required_peak = 0.0;
    if (!wire_torque(c->dm, 1.0f, &positive, &ignored) ||
        !wire_torque(c->dm, -1.0f, &negative, &ignored)) return "MOTOR_ADAPTER_FAULT";
    *available = fmin(fabs(positive), fabs(negative));
    if (c->profile->min_angle > -amplitude || c->profile->max_angle < amplitude)
        return "REFERENCE_OUTSIDE_JOINT_RANGE";
    for (i = 0u; i <= 60000u; ++i) {
        const double time = (double)i * PLANT_DT;
        double q, v, a, required;
        reference_at(c, time, &q, &v, &a);
        required = c->profile->inertia * c->inertia_scale * a + DAMPING * v +
                   gravity(c->profile, q) - disturbance(time);
        if (!isfinite(required)) return "NONFINITE_INVERSE_DYNAMICS";
        *required_peak = fmax(*required_peak, fabs(required));
    }
    return *required_peak > *available ? "REQUIRED_TORQUE_EXCEEDS_AVAILABLE" : NULL;
}

static int write_metrics(FILE *file, const case_t *c, const metrics_t *m,
                         double required, double available, const char *status,
                         const char *reason)
{
    const double rms = m->samples != 0u ? sqrt(m->error_squared / m->samples) : NAN;
    const double eval_rms = m->evaluation_samples != 0u ?
                           sqrt(m->evaluation_error_squared / m->evaluation_samples) : NAN;
    return fprintf(file,
        "%s,%s,%s,%s,%s,%u,%.3g,%.3g,%.12g,%.12g,%.12g,%.12g,%s,%s,%u,%u,"
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
        "%.9g,%.9g,%.9g\n",
        c->profile->model, c->profile->axis, motor_name(c), controller_name(c),
        reference_name(c), c->hz, c->inertia_scale, c->gravity_ff_scale,
        c->profile->inertia, c->profile->inertia * c->inertia_scale, required, available,
        status, reason, m->samples, m->evaluation_samples, rms, eval_rms,
        m->samples != 0u ? m->peak_error : NAN,
        m->evaluation_samples != 0u ? m->evaluation_peak : NAN,
        m->samples != 0u ? m->peak_request : NAN,
        m->samples != 0u ? m->peak_wire : NAN,
        m->samples != 0u ? m->peak_actuator : NAN,
        m->samples != 0u ? (double)m->saturated_samples / m->samples : NAN,
        m->samples != 0u ? (double)m->adapter_clamps / m->samples : NAN,
        m->samples != 0u ? m->total_variation : NAN,
        m->samples != 0u ? m->total_variation / ((double)m->samples / c->hz) : NAN,
        m->samples != 0u ? m->min_angle : NAN,
        m->samples != 0u ? m->max_angle : NAN,
        m->samples != 0u ? m->final_error : NAN,
        c->profile->min_angle, c->profile->max_angle,
        c->profile->gravity_sin, c->profile->gravity_cos) >= 0;
}

/* Result: 0 PASS, 1 FAIL (including I/O), 2 INFEASIBLE. */
static int run_case(FILE *metrics_file, FILE *trace, const case_t *c)
{
    const unsigned int ticks = (unsigned int)(DURATION * c->hz);
    const unsigned int substeps = 10000u / c->hz;
    const double dt = 1.0 / c->hz;
    const int save_trace = c->hz == 1000u && c->inertia_scale == 1.0 && c->gravity_ff_scale == 1.0;
    const char *reason;
    double required = 0.0, available = 0.0, previous_wire = 0.0;
    gimbal_smc_config_t config;
    gimbal_smc_t controller;
    gimbal_smc_input_t input;
    gimbal_smc_output_t output;
    metrics_t m;
    plant_t plant;
    unsigned int tick;
    int result = 0;
    memset(&m, 0, sizeof(m));
    memset(&plant, 0, sizeof(plant));
    memset(&input, 0, sizeof(input));
    reason = preflight(c, &required, &available);
    if (reason != NULL) {
        result = strcmp(reason, "REFERENCE_OUTSIDE_JOINT_RANGE") == 0 ||
                 strcmp(reason, "REQUIRED_TORQUE_EXCEEDS_AVAILABLE") == 0 ? 2 : 1;
        if (!write_metrics(metrics_file, c, &m, required, available,
                            result == 2 ? "INFEASIBLE" : "FAIL", reason)) return 1;
        if (result == 1 || (c->hz == 1000u && !c->terminal && c->gravity_ff_scale == 1.0))
            printf("%s %s/%s %s %s Jx%.1f: %s; required %.6g available %.6g Nm\n",
                   result == 2 ? "INFEASIBLE" : "FAIL", c->profile->model,
                   c->profile->axis, motor_name(c), reference_name(c), c->inertia_scale,
                   reason, required, available);
        return result;
    }
    /* A small position perturbation tests hold acquisition without inventing a
     * large reference step. Pick the side that stays inside the declared joint. */
    if (!c->sine) {
        plant.value.angle = c->profile->max_angle >= 0.03 ? 0.03 :
                            (c->profile->min_angle <= -0.03 ? -0.03 : 0.0);
    }
    m.min_angle = m.max_angle = plant.value.angle;
    gimbal_smc_default_config(&config);
    config.inertia_kg_m2 = (float)c->profile->inertia;
    config.viscous_nm_s_rad = (float)DAMPING;
    config.torque_limit_nm = (float)APPLICATION_TORQUE_LIMIT;
    config.rate_lpf_hz = 80.0f;
    config.terminal_gain = c->terminal ? 1.5f : 0.0f;
    if (!gimbal_smc_init(&controller, &config)) {
        write_metrics(metrics_file, c, &m, required, available, "FAIL", "CONTROLLER_CONFIG");
        return 1;
    }
    input.enable = true;
    input.dt_s = (float)dt;
    reason = "OK";
    for (tick = 0u; tick < ticks; ++tick) {
        const double time = (double)tick * dt;
        double q, v, a, wire, error;
        const double angle_noise = 0.0001 * (sin(2.0 * PI_D * 31.0 * time) +
                                            0.5 * sin(2.0 * PI_D * 73.0 * time));
        const double rate_noise = 0.002 * (sin(2.0 * PI_D * 43.0 * time) +
                                          0.5 * sin(2.0 * PI_D * 91.0 * time));
        int clamped;
        unsigned int substep;
        reference_at(c, time, &q, &v, &a);
        input.angle_rad = (float)(plant.value.angle + angle_noise);
        input.rate_rad_s = (float)(plant.value.rate + rate_noise);
        input.reference_rad = (float)q;
        input.reference_rate_rad_s = (float)v;
        input.reference_accel_rad_s2 = (float)a;
        /* ff=1 uses exact model gravity and true state: an ideal identification
         * condition. ff=.8 deliberately underestimates the gravity coefficient. */
        input.feedforward_nm = (float)(c->gravity_ff_scale * gravity(c->profile, plant.value.angle));
        (void)gimbal_smc_update(&controller, &input, &output);
        if (!output.valid || !isfinite(output.torque_nm)) {
            result = 1; reason = "CONTROLLER_INVALID"; break;
        }
        if (fabs((double)output.torque_nm) > APPLICATION_TORQUE_LIMIT + 1e-9) {
            result = 1; reason = "REQUEST_TORQUE_LIMIT"; break;
        }
        if (!wire_torque(c->dm, output.torque_nm, &wire, &clamped)) {
            result = 1; reason = "WIRE_FORMAT_OR_TORQUE_LIMIT"; break;
        }
        error = plant.value.angle - q;
        m.error_squared += error * error;
        m.peak_error = fmax(m.peak_error, fabs(error));
        m.peak_request = fmax(m.peak_request, fabs((double)output.torque_nm));
        m.peak_wire = fmax(m.peak_wire, fabs(wire));
        m.total_variation += fabs(wire - previous_wire);
        m.saturated_samples += (output.flags & GIMBAL_SMC_AMPLITUDE_LIMIT) != 0u;
        m.adapter_clamps += clamped != 0;
        ++m.samples;
        if (time >= 1.0) {
            m.evaluation_error_squared += error * error;
            m.evaluation_peak = fmax(m.evaluation_peak, fabs(error));
            ++m.evaluation_samples;
        }
        /* Save representative 1 kHz controllers at 100 Hz only. All limit,
         * performance and total-variation metrics still use every control tick. */
        if (save_trace && tick % 10u == 0u &&
            fprintf(trace, "%s,%s,%s,%s,%s,%u,%.7f,%.9g,%.9g,%.9g,%.9g,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u\n",
                    c->profile->model, c->profile->axis, motor_name(c), controller_name(c),
                    reference_name(c), c->hz, time, plant.value.angle, q, error,
                    plant.value.rate, v, a, (double)output.torque_nm, wire,
                    plant.value.torque, gravity(c->profile, plant.value.angle),
                    (double)input.feedforward_nm, disturbance(time),
                    (unsigned int)output.flags) < 0) {
            result = 1; reason = "TRACE_IO"; break;
        }
        previous_wire = wire;
        for (substep = 0u; substep < substeps; ++substep) {
            plant_step(&plant, c, wire, time + (double)substep * PLANT_DT);
            if (!isfinite(plant.value.angle) || !isfinite(plant.value.rate) ||
                !isfinite(plant.value.torque)) {
                result = 1; reason = "PLANT_NONFINITE"; break;
            }
            if (fabs(plant.value.torque) > APPLICATION_TORQUE_LIMIT + 1e-9) {
                result = 1; reason = "ACTUATOR_TORQUE_LIMIT"; break;
            }
            m.min_angle = fmin(m.min_angle, plant.value.angle);
            m.max_angle = fmax(m.max_angle, plant.value.angle);
            m.peak_actuator = fmax(m.peak_actuator, fabs(plant.value.torque));
            if (plant.value.angle < c->profile->min_angle ||
                plant.value.angle > c->profile->max_angle) {
                result = 1; reason = "JOINT_RANGE"; break;
            }
        }
        if (result != 0) break;
    }
    {
        double q, v, a;
        reference_at(c, (double)m.samples * dt, &q, &v, &a);
        m.final_error = plant.value.angle - q;
    }
    if (result == 0 && (m.evaluation_samples == 0u ||
        sqrt(m.evaluation_error_squared / m.evaluation_samples) >= 0.02 ||
        m.evaluation_peak >= 0.06)) {
        result = 1; reason = "TRACKING_ENVELOPE";
    }
    if (!write_metrics(metrics_file, c, &m, required, available,
                        result == 0 ? "PASS" : "FAIL", reason)) result = 1;
    if (result != 0) {
        printf("FAIL %s/%s %s %s %s %uHz Jx%.1f gravity_ff=%.1f: %s, "
               "samples=%u rms>=1s=%.6g peak>=1s=%.6g wire_peak=%.6g\n",
               c->profile->model, c->profile->axis, motor_name(c), controller_name(c),
               reference_name(c), c->hz, c->inertia_scale, c->gravity_ff_scale, reason,
               m.samples, m.evaluation_samples != 0u ?
               sqrt(m.evaluation_error_squared / m.evaluation_samples) : NAN,
               m.evaluation_peak, m.peak_wire);
    }
    return result;
}

int main(int argc, char **argv)
{
    static profile_t profiles[MAX_PROFILES];
    const unsigned int rates[] = {500u, 1000u, 2000u};
    const double scales[] = {0.5, 1.0, 2.0};
    const double gravity_scales[] = {1.0, 0.8};
    size_t count, p, h, j, g;
    unsigned int passed = 0u, failed = 0u, infeasible = 0u;
    int dm, terminal, sine, io_error = 0;
    FILE *metrics, *trace;
    if (argc != 4 || strcmp(argv[1], argv[2]) == 0 || strcmp(argv[1], argv[3]) == 0 ||
        strcmp(argv[2], argv[3]) == 0) {
        fprintf(stderr, "Usage: %s profiles.csv metrics.csv trace.csv (distinct paths)\n", argv[0]);
        return 1;
    }
    if (!read_profiles(argv[1], profiles, &count)) return 1;
    metrics = fopen(argv[2], "w");
    if (metrics == NULL) { perror(argv[2]); return 1; }
    trace = fopen(argv[3], "w");
    if (trace == NULL) { perror(argv[3]); fclose(metrics); return 1; }
    fputs("model,axis,motor,controller,reference,hz,inertia_scale,gravity_ff_scale,"
          "nominal_inertia_kg_m2,plant_inertia_kg_m2,required_peak_nm,available_wire_nm,"
          "status,reason,samples,evaluation_samples,rms_rad,rms_after_1s_rad,peak_rad,"
          "peak_after_1s_rad,peak_request_nm,peak_wire_nm,peak_actuator_nm,saturation_ratio,"
          "adapter_clamp_ratio,wire_total_variation_nm,wire_total_variation_nm_per_s,"
          "min_angle_rad,max_angle_rad,final_error_rad,joint_min_rad,joint_max_rad,"
          "gravity_sin_nm,gravity_cos_nm\n", metrics);
    fputs("model,axis,motor,controller,reference,hz,time_s,angle_rad,reference_rad,error_rad,"
          "rate_rad_s,reference_rate_rad_s,reference_accel_rad_s2,requested_nm,wire_nm,"
          "actuator_nm,gravity_nm,feedforward_nm,disturbance_nm,flags\n", trace);
    puts("MODEL-BASED SYNTHETIC REGRESSION: not hardware or full coupled robot validation.");
    printf("Profiles=%lu; 144 cases/profile. J_nom=CSV, J_plant=J_nom*x; other joint frozen.\n",
           (unsigned long)count);
    puts("Assumptions: direct drive; DM4310 MIT TMAX=10 Nm, GM6020 Kt=.741 Nm/A; "
         "both application limits=1 Nm, not measured/rated capability claims.");
    puts("B=.01; command delay=1 ms; actuator lag=3 ms; load after 2s=-.03+.02*sin(2*pi*1.3*(t-2)); "
         "synthetic gyro/angle ripple follows original synthetic harness.");
    puts("Controller lambda=12 k=10 eta=20 phi=.1; terminal alpha=1.5 r=.5 delta=.01; "
         "LPF=80 Hz; ideal gravity feedforward coefficient=1 or assumed mismatch=.8.");
    puts("Thresholds after 1s: RMS<.02 rad and peak<.06 rad; all samples checked for faults, "
         "actual wire torque and joint limits. TV is total variation, not a chattering metric.");
    puts("Representative traces: 1 kHz controller, Jx1, ff1, logged at 100 Hz. "
         "Unrun INFEASIBLE rows have zero samples and NaN performance fields.");
    puts("Trace gravity_nm denotes G(q), the compensating torque in J*qdd=tau-B*qdot-G(q)+d; "
         "the physical gravitational torque is -G(q).");
    for (p = 0u; p < count; ++p) for (dm = 0; dm < 2; ++dm)
    for (h = 0u; h < sizeof(rates) / sizeof(rates[0]); ++h)
    for (j = 0u; j < sizeof(scales) / sizeof(scales[0]); ++j)
    for (sine = 0; sine < 2; ++sine) for (terminal = 0; terminal < 2; ++terminal)
    for (g = 0u; g < sizeof(gravity_scales) / sizeof(gravity_scales[0]); ++g) {
        const case_t c = {&profiles[p], rates[h], scales[j], gravity_scales[g], dm, terminal, sine};
        const int result = run_case(metrics, trace, &c);
        if (result == 0) ++passed;
        else if (result == 2) ++infeasible;
        else ++failed;
    }
    if (ferror(metrics) || ferror(trace)) io_error = 1;
    if (fclose(metrics) != 0) io_error = 1;
    if (fclose(trace) != 0) io_error = 1;
    printf("Summary: PASS=%u FAIL=%u INFEASIBLE=%u IO_ERROR=%d. "
           "Exit 0=all pass; 1=failure; 2=infeasible without failures.\n",
           passed, failed, infeasible, io_error);
    return failed != 0u || io_error ? 1 : (infeasible != 0u ? 2 : 0);
}
