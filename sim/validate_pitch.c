/* Host-only pitch regression; executes the portable C controller, gravity
 * helper and actual motor packers. Source-derived single-axis models remain
 * synthetic: no MCU timing, physical stop, full robot or real motor is modeled.
 * Run: validate_pitch profiles.csv metrics.csv trace.csv
 *      validate_pitch --self-test profiles.csv
 */
#include "gimbal_smc.h"
#include "gimbal_motor.h"
#include "gimbal_pitch.h"

#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_D 3.14159265358979323846
#define HZ 1000u
#define PLANT_DT 0.0001
#define DURATION 16.0
#define STARTUP_END 1.0
#define DELAY_STEPS 10u
#define ACTUATOR_LAG 0.003
#define DAMPING 0.01
#define COULOMB 0.02
#define FRICTION_RATE 0.02
#define SOFT_MARGIN 0.08
#define MAX_PROFILES 64u
#define NAME_CAP 64u
#define HEADER "model,axis,inertia_kg_m2,gravity_sin_nm,gravity_cos_nm,min_rad,max_rad"

typedef struct {
    char model[NAME_CAP], axis[NAME_CAP];
    double inertia, gravity_sin, gravity_cos, min_angle, max_angle;
} profile_t;

typedef struct { double angle, rate, torque; } state_t;
typedef struct {
    state_t value;
    double delay[DELAY_STEPS];
    unsigned int head;
} plant_t;

typedef enum { STARTUP, UP, DOWN, HOLD, PHASE_COUNT } phase_t;
static const char *const phase_names[] = {"startup", "up", "down", "hold"};
static const char *const conditions[] = {
    "nominal", "gravity_under20", "gravity_over20", "payload_unmodeled",
    "tilted_correct", "tilted_wrong_frame"
};

typedef struct {
    const profile_t *profile;
    int dm;
    unsigned int condition;
    double torque_limit, ff_scale, payload_mass, payload_arm;
    double base_tilt, ff_tilt;
} case_t;

typedef struct {
    unsigned int count;
    double squared_error, peak_error, signed_error;
} phase_metrics_t;

typedef struct {
    phase_metrics_t phases[PHASE_COUNT];
    double peak_request, peak_wire, peak_actuator, total_variation;
    double min_angle, max_angle, hard_margin, reference_interval_margin;
    unsigned int samples, saturation_samples, adapter_clamps;
} metrics_t;

static const char *motor_name(const case_t *c) { return c->dm ? "DM4310" : "GM6020"; }
static const char *role_name(const case_t *c) { return c->condition == 5u ? "diagnostic" : "acceptance"; }

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

static case_t make_case(const profile_t *p, int dm, unsigned int condition)
{
    case_t c;
    memset(&c, 0, sizeof(c));
    c.profile = p;
    c.dm = dm;
    c.condition = condition;
    c.torque_limit = 1.0;
    c.ff_scale = condition == 1u ? 0.8 : (condition == 2u ? 1.2 : 1.0);
    if (condition == 3u) { c.payload_mass = 0.1; c.payload_arm = 0.08; }
    if (condition >= 4u) c.base_tilt = PI_D / 12.0;
    if (condition == 4u) c.ff_tilt = c.base_tilt;
    return c;
}

static double plant_inertia(const case_t *c)
{
    return c->profile->inertia + c->payload_mass * c->payload_arm * c->payload_arm;
}

/* The declared added point mass is oriented to add -m*g*r*cos(q) to G(q).
 * A fixed base rotation about the SAME pitch axis shifts the gravity angle.
 * This is not an arbitrary three-dimensional moving-base approximation. */
static double plant_cos(const case_t *c)
{ return c->profile->gravity_cos - c->payload_mass * 9.81 * c->payload_arm; }

static double gravity(const case_t *c, double angle)
{
    return c->profile->gravity_sin * sin(angle + c->base_tilt) +
           plant_cos(c) * cos(angle + c->base_tilt);
}

static double friction(double rate)
{ return DAMPING * rate + COULOMB * tanh(rate / FRICTION_RATE); }

static double disturbance(double time)
{ return time < 2.0 ? 0.0 : -0.03 + 0.02 * sin(2.0 * PI_D * 1.3 * (time - 2.0)); }

static void quintic(double start, double end, double elapsed, double duration,
                    double *q, double *v, double *a)
{
    const double x = elapsed / duration;
    const double x2 = x * x, x3 = x2 * x;
    const double delta = end - start;
    *q = start + delta * x3 * (10.0 - 15.0 * x + 6.0 * x2);
    *v = delta * 30.0 * x2 * (1.0 - x) * (1.0 - x) / duration;
    *a = delta * 60.0 * x * (1.0 - x) * (1.0 - 2.0 * x) / (duration * duration);
}

/* All moving segments join with continuous position, zero endpoint speed and
 * acceleration. Startup [0,1) is excluded from tracking criteria in advance;
 * protocol, nonfinite, torque and physical joint checks still include it.
 * [1,3): zero->low; [4,7): low->high; [8,11): high->low;
 * [12,14): low->zero; other intervals hold their endpoint. */
static phase_t reference_at(const case_t *c, double time,
                            double *q, double *v, double *a)
{
    const double lo = c->profile->min_angle + SOFT_MARGIN;
    const double hi = c->profile->max_angle - SOFT_MARGIN;
    *q = *v = *a = 0.0;
    if (time < 1.0) return STARTUP;
    if (time < 3.0) { quintic(0.0, lo, time - 1.0, 2.0, q, v, a); return DOWN; }
    if (time < 4.0) { *q = lo; return HOLD; }
    if (time < 7.0) { quintic(lo, hi, time - 4.0, 3.0, q, v, a); return UP; }
    if (time < 8.0) { *q = hi; return HOLD; }
    if (time < 11.0) { quintic(hi, lo, time - 8.0, 3.0, q, v, a); return DOWN; }
    if (time < 12.0) { *q = lo; return HOLD; }
    if (time < 14.0) { quintic(lo, 0.0, time - 12.0, 2.0, q, v, a); return UP; }
    return HOLD;
}

/* Actual wire bytes are independently decoded. The adapter-applied output is
 * deliberately not the oracle. Both motors use direct drive, efficiency=1. */
static int wire_torque(const case_t *c, float requested, double *decoded, int *clamped)
{
    gimbal_can_frame_t frame;
    float adapter_applied;
    gimbal_motor_status_t status;
    if (c->dm) {
        const gimbal_dm4310_config_t cfg = {
            1u, 0x11u, 1u, 12.5f, 30.0f, 10.0f,
            {1.0f, 1.0f, (float)c->torque_limit, 1}
        };
        unsigned int code;
        status = gimbal_dm4310_pack_torque(&cfg, requested, &frame, &adapter_applied);
        if (status < 0 || frame.id != 1u || frame.dlc != 8u ||
            frame.data[0] != 0x80u || frame.data[1] != 0u ||
            frame.data[2] != 0x80u || frame.data[3] != 0u ||
            frame.data[4] != 0u || frame.data[5] != 0u ||
            (frame.data[6] & 0xf0u) != 0u) return 0;
        code = ((unsigned int)(frame.data[6] & 0x0fu) << 8) | frame.data[7];
        *decoded = -10.0 + 20.0 * (double)code / 4095.0;
    } else {
        const gimbal_gm6020_config_t cfg = {
            1u, 1u, 0.741f, (float)(c->torque_limit / 0.741), 0.0f,
            {1.0f, 1.0f, (float)c->torque_limit, 1}
        };
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
    return isfinite(*decoded) && fabs(*decoded) <= c->torque_limit + 1e-9;
}

static state_t rhs(state_t s, const case_t *c, double command, double time)
{
    state_t d;
    d.angle = s.rate;
    d.rate = (s.torque - friction(s.rate) - gravity(c, s.angle) + disturbance(time)) /
             plant_inertia(c);
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
    p->value.angle += PLANT_DT * (k1.angle + 2.0*k2.angle + 2.0*k3.angle + k4.angle) / 6.0;
    p->value.rate += PLANT_DT * (k1.rate + 2.0*k2.rate + 2.0*k3.rate + k4.rate) / 6.0;
    p->value.torque += PLANT_DT * (k1.torque + 2.0*k2.torque + 2.0*k3.torque + k4.torque) / 6.0;
}

/* A sampled inverse-dynamics violation proves infeasibility; passing is only
 * necessary and does not certify transients, inter-sample extrema or hardware. */
static const char *preflight(const case_t *c, double *required_peak, double *available)
{
    double positive, negative;
    unsigned int i;
    int ignored;
    const double lo = c->profile->min_angle + SOFT_MARGIN;
    const double hi = c->profile->max_angle - SOFT_MARGIN;
    *required_peak = *available = NAN;
    if (!(lo < 0.0 && hi > 0.0 && lo < hi)) return "INVALID_REFERENCE_RANGE";
    if (!wire_torque(c, (float)c->torque_limit, &positive, &ignored) ||
        !wire_torque(c, (float)-c->torque_limit, &negative, &ignored))
        return "MOTOR_ADAPTER_FAULT";
    *available = fmin(fabs(positive), fabs(negative));
    *required_peak = 0.0;
    for (i = 0u; i <= (unsigned int)(DURATION / PLANT_DT); ++i) {
        const double time = (double)i * PLANT_DT;
        double q, v, a, required;
        (void)reference_at(c, time, &q, &v, &a);
        required = plant_inertia(c)*a + friction(v) + gravity(c, q) - disturbance(time);
        if (!isfinite(q) || !isfinite(v) || !isfinite(a) || !isfinite(required))
            return "NONFINITE_INVERSE_DYNAMICS";
        if (q < c->profile->min_angle || q > c->profile->max_angle)
            return "REFERENCE_OUTSIDE_JOINT_RANGE";
        *required_peak = fmax(*required_peak, fabs(required));
    }
    return *required_peak > *available ? "REQUIRED_TORQUE_EXCEEDS_AVAILABLE" : NULL;
}

static double rms(const phase_metrics_t *p)
{ return p->count != 0u ? sqrt(p->squared_error / p->count) : NAN; }

static void observe_state(metrics_t *m, const case_t *c, state_t s)
{
    const double hard = fmin(s.angle - c->profile->min_angle, c->profile->max_angle - s.angle);
    m->min_angle = fmin(m->min_angle, s.angle);
    m->max_angle = fmax(m->max_angle, s.angle);
    m->hard_margin = fmin(m->hard_margin, hard);
    m->reference_interval_margin = fmin(m->reference_interval_margin, hard - SOFT_MARGIN);
    m->peak_actuator = fmax(m->peak_actuator, fabs(s.torque));
}

static int write_metrics(FILE *file, const case_t *c, const metrics_t *m,
                         double required, double available, const char *status,
                         const char *reason)
{
    unsigned int phase;
    if (fprintf(file, "%s,pitch,%s,%s,%s,%s,%s,%u,%.9g,%.9g,%.12g,%.12g,"
        "%.12g,%.12g,%.12g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u",
        c->profile->model, motor_name(c), conditions[c->condition], role_name(c), status, reason,
        HZ, DURATION, STARTUP_END, c->profile->inertia, plant_inertia(c),
        c->profile->gravity_sin, c->profile->gravity_cos, plant_cos(c), c->ff_scale,
        c->base_tilt, c->ff_tilt, c->payload_mass, c->payload_arm,
        c->torque_limit, required, available, c->profile->min_angle, c->profile->max_angle,
        c->profile->min_angle + SOFT_MARGIN, c->profile->max_angle - SOFT_MARGIN,
        DAMPING, COULOMB, FRICTION_RATE, DELAY_STEPS*PLANT_DT, ACTUATOR_LAG,
        12.0, 10.0, 20.0, 0.1, 80.0, m->samples) < 0) return 0;
    for (phase = STARTUP; phase < PHASE_COUNT; ++phase) {
        const phase_metrics_t *p = &m->phases[phase];
        if (fprintf(file, ",%u,%.9g,%.9g,%.9g", p->count, rms(p),
                    p->count != 0u ? p->peak_error : NAN,
                    p->count != 0u ? p->signed_error / p->count : NAN) < 0) return 0;
    }
    return fprintf(file, ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
        m->samples != 0u ? m->peak_request : NAN,
        m->samples != 0u ? m->peak_wire : NAN,
        m->samples != 0u ? m->peak_actuator : NAN,
        m->samples != 0u ? (double)m->saturation_samples / m->samples : NAN,
        m->samples != 0u ? (double)m->adapter_clamps / m->samples : NAN,
        m->samples != 0u ? m->total_variation / ((double)m->samples/HZ) : NAN,
        m->samples != 0u ? m->min_angle : NAN,
        m->samples != 0u ? m->max_angle : NAN,
        m->samples != 0u ? m->hard_margin : NAN,
        m->samples != 0u ? m->reference_interval_margin : NAN) >= 0;
}

/* Return 0 PASS; 1 operational/input fault; 2 INFEASIBLE; 3 tracking FAIL.
 * Diagnostic tracking failures are reported but do not fail acceptance. All
 * diagnostic operational faults (including hard-stop crossings) fail the run. */
static int run_case(FILE *metrics_file, FILE *trace, const case_t *c)
{
    gimbal_smc_config_t cfg;
    gimbal_smc_t controller;
    gimbal_smc_input_t input;
    gimbal_smc_output_t output;
    gimbal_pitch_gravity_model_t ff_model;
    metrics_t m;
    plant_t plant;
    double required, available, previous_wire = 0.0;
    unsigned int tick, phase;
    int result = 0;
    const char *reason = preflight(c, &required, &available);
    memset(&m, 0, sizeof(m));
    memset(&plant, 0, sizeof(plant));
    memset(&input, 0, sizeof(input));
    m.hard_margin = m.reference_interval_margin = DBL_MAX;
    if (reason != NULL) {
        result = strcmp(reason, "REQUIRED_TORQUE_EXCEEDS_AVAILABLE") == 0 ||
                 strcmp(reason, "REFERENCE_OUTSIDE_JOINT_RANGE") == 0 ? 2 : 1;
        if (!write_metrics(metrics_file, c, &m, required, available,
                           result == 2 ? "INFEASIBLE" : "FAIL", reason)) return 1;
        printf("%s %s %s %s: %s\n", result == 2 ? "INFEASIBLE" : "FAIL",
               c->profile->model, motor_name(c), conditions[c->condition], reason);
        return result;
    }
    gimbal_smc_default_config(&cfg);
    cfg.inertia_kg_m2 = (float)c->profile->inertia;
    cfg.viscous_nm_s_rad = (float)DAMPING;
    cfg.torque_limit_nm = (float)c->torque_limit;
    cfg.rate_lpf_hz = 80.0f;
    cfg.wrap_angle = false;
    if (!gimbal_smc_init(&controller, &cfg)) {
        (void)write_metrics(metrics_file, c, &m, required, available, "FAIL", "CONTROLLER_CONFIG");
        return 1;
    }
    ff_model.sin_nm = (float)(c->ff_scale * c->profile->gravity_sin);
    ff_model.cos_nm = (float)(c->ff_scale * c->profile->gravity_cos);
    input.enable = true;
    input.dt_s = 1.0f / HZ;
    reason = "OK";
    observe_state(&m, c, plant.value);
    for (tick = 0u; tick < (unsigned int)(DURATION * HZ); ++tick) {
        const double time = (double)tick / HZ;
        const double angle_noise = 0.0001 * (sin(2.0*PI_D*31.0*time) +
                                             0.5*sin(2.0*PI_D*73.0*time));
        const double rate_noise = 0.002 * (sin(2.0*PI_D*43.0*time) +
                                           0.5*sin(2.0*PI_D*91.0*time));
        double q, v, a, wire, error;
        phase_metrics_t *pm;
        const phase_t current_phase = reference_at(c, time, &q, &v, &a);
        const float gravity_angle = (float)(plant.value.angle + angle_noise + c->ff_tilt);
        unsigned int substep;
        int clamped;
        input.angle_rad = (float)(plant.value.angle + angle_noise);
        input.rate_rad_s = (float)(plant.value.rate + rate_noise);
        input.reference_rad = (float)q;
        input.reference_rate_rad_s = (float)v;
        input.reference_accel_rad_s2 = (float)a;
        if (!gimbal_pitch_gravity_torque(&ff_model, gravity_angle, &input.feedforward_nm)) {
            result = 1; reason = "GRAVITY_HELPER_FAULT"; break;
        }
        (void)gimbal_smc_update(&controller, &input, &output);
        if (!output.valid || !isfinite(output.torque_nm) ||
            !isfinite(output.unsaturated_torque_nm) || !isfinite(output.sliding_rad_s)) {
            result = 1; reason = "CONTROLLER_INVALID"; break;
        }
        if (fabs((double)output.torque_nm) > c->torque_limit + 1e-9) {
            result = 1; reason = "REQUEST_TORQUE_LIMIT"; break;
        }
        if (!wire_torque(c, output.torque_nm, &wire, &clamped)) {
            result = 1; reason = "WIRE_FORMAT_OR_TORQUE_LIMIT"; break;
        }
        error = plant.value.angle - q;
        pm = &m.phases[current_phase];
        ++pm->count;
        pm->squared_error += error * error;
        pm->signed_error += error;
        pm->peak_error = fmax(pm->peak_error, fabs(error));
        ++m.samples;
        m.peak_request = fmax(m.peak_request, fabs((double)output.torque_nm));
        m.peak_wire = fmax(m.peak_wire, fabs(wire));
        m.total_variation += fabs(wire - previous_wire);
        m.saturation_samples += (output.flags & GIMBAL_SMC_AMPLITUDE_LIMIT) != 0u;
        m.adapter_clamps += clamped != 0;
        /* Trace is decimated to 100 Hz; all statistics use all 1 kHz ticks and
         * physical limits are checked at every 0.1 ms integration substep. */
        if (tick % 10u == 0u && fprintf(trace,
            "%s,pitch,%s,%s,%s,%s,%.7f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
            "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u\n",
            c->profile->model, motor_name(c), conditions[c->condition], role_name(c),
            phase_names[current_phase], time, plant.value.angle, q, error,
            plant.value.rate, v, a, (double)input.angle_rad, (double)gravity_angle,
            gravity(c, plant.value.angle), (double)input.feedforward_nm,
            (double)output.torque_nm, wire, plant.value.torque, disturbance(time),
            (unsigned int)output.flags) < 0) {
            result = 1; reason = "TRACE_IO"; break;
        }
        previous_wire = wire;
        for (substep = 0u; substep < 10u; ++substep) {
            plant_step(&plant, c, wire, time + substep*PLANT_DT);
            if (!isfinite(plant.value.angle) || !isfinite(plant.value.rate) ||
                !isfinite(plant.value.torque)) {
                result = 1; reason = "PLANT_NONFINITE"; break;
            }
            observe_state(&m, c, plant.value);
            if (fabs(plant.value.torque) > c->torque_limit + 1e-9) {
                result = 1; reason = "ACTUATOR_TORQUE_LIMIT"; break;
            }
            if (plant.value.angle < c->profile->min_angle ||
                plant.value.angle > c->profile->max_angle) {
                result = 1; reason = "JOINT_RANGE"; break;
            }
        }
        if (result != 0) break;
    }
    if (result == 0) for (phase = UP; phase < PHASE_COUNT; ++phase) {
        if (m.phases[phase].count == 0u || !isfinite(rms(&m.phases[phase])) ||
            rms(&m.phases[phase]) >= 0.02 || m.phases[phase].peak_error >= 0.06) {
            result = 3; reason = "DIRECTIONAL_TRACKING_ENVELOPE"; break;
        }
    }
    if (!write_metrics(metrics_file, c, &m, required, available,
                       result == 0 ? "PASS" : "FAIL", reason)) return 1;
    printf("%s %s %s %s %s: up/down/hold RMS %.6g/%.6g/%.6g rad; "
           "peak wire %.6g Nm; hard margin %.6g rad%s\n",
           result == 0 ? "PASS" : "FAIL", role_name(c), c->profile->model, motor_name(c),
           conditions[c->condition], rms(&m.phases[UP]), rms(&m.phases[DOWN]),
           rms(&m.phases[HOLD]), m.peak_wire, m.hard_margin,
           result != 0 ? reason : "");
    return result;
}

static int self_test(const profile_t *profiles, size_t count)
{
    size_t i;
    unsigned int tested = 0u;
    int dm;
    for (i = 0u; i < count; ++i) {
        if (strcmp(profiles[i].model, "dynamicx_standard3") != 0 ||
            strcmp(profiles[i].axis, "pitch") != 0) continue;
        for (dm = 0; dm < 2; ++dm) {
            case_t c = make_case(&profiles[i], dm, 0u);
            double required, available;
            const char *reason;
            c.torque_limit = 0.25;
            reason = preflight(&c, &required, &available);
            if (reason == NULL || strcmp(reason, "REQUIRED_TORQUE_EXCEEDS_AVAILABLE") != 0 ||
                !isfinite(required) || !isfinite(available) || required <= available) {
                fprintf(stderr, "Negative test failed for %s: %s\n", motor_name(&c),
                        reason != NULL ? reason : "incorrectly declared feasible");
                return 1;
            }
            printf("NEGATIVE PASS %s: INFEASIBLE at 0.25 Nm; required %.9g, available %.9g Nm\n",
                   motor_name(&c), required, available);
            ++tested;
        }
    }
    if (tested != 2u) {
        fputs("Negative test requires the archived dynamicx_standard3/pitch profile.\n", stderr);
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    static profile_t profiles[MAX_PROFILES];
    size_t count, p;
    int dm, io_error = 0;
    unsigned int condition, accepted = 0u, acceptance_failed = 0u;
    unsigned int diagnostic_passed = 0u, diagnostic_failed = 0u, operational_failed = 0u;
    FILE *metrics_file, *trace;
    if (argc == 3 && strcmp(argv[1], "--self-test") == 0) {
        if (!read_profiles(argv[2], profiles, &count)) return 1;
        return self_test(profiles, count);
    }
    if (argc != 4 || strcmp(argv[1], argv[2]) == 0 || strcmp(argv[1], argv[3]) == 0 ||
        strcmp(argv[2], argv[3]) == 0) {
        fprintf(stderr, "Usage: %s profiles.csv metrics.csv trace.csv (distinct paths)\n"
                        "       %s --self-test profiles.csv\n", argv[0], argv[0]);
        return 1;
    }
    if (!read_profiles(argv[1], profiles, &count)) return 1;
    metrics_file = fopen(argv[2], "w");
    if (metrics_file == NULL) { perror(argv[2]); return 1; }
    trace = fopen(argv[3], "w");
    if (trace == NULL) { perror(argv[3]); fclose(metrics_file); return 1; }
    fputs("model,axis,motor,condition,role,status,reason,hz,duration_s,startup_excluded_s,"
          "nominal_inertia_kg_m2,plant_inertia_kg_m2,nominal_gravity_sin_nm,nominal_gravity_cos_nm,"
          "plant_gravity_cos_nm,gravity_ff_scale,base_tilt_rad,feedforward_tilt_rad,"
          "payload_mass_kg,payload_arm_m,torque_limit_nm,required_peak_nm,available_wire_nm,"
          "joint_min_rad,joint_max_rad,reference_min_rad,reference_max_rad,viscous_nm_s_rad,"
          "coulomb_nm,friction_smoothing_rad_s,command_delay_s,actuator_lag_s,lambda_per_s,"
          "reaching_per_s,robust_rad_s2,boundary_rad_s,rate_lpf_hz,samples,"
          "startup_samples,startup_rms_rad,startup_peak_rad,startup_bias_rad,"
          "up_samples,up_rms_rad,up_peak_rad,up_bias_rad,"
          "down_samples,down_rms_rad,down_peak_rad,down_bias_rad,"
          "hold_samples,hold_rms_rad,hold_peak_rad,hold_bias_rad,"
          "peak_request_nm,peak_wire_nm,peak_actuator_nm,saturation_ratio,adapter_clamp_ratio,"
          "wire_total_variation_nm_per_s,min_angle_rad,max_angle_rad,min_hard_margin_rad,"
          "min_reference_interval_margin_rad\n", metrics_file);
    fputs("model,axis,motor,condition,role,phase,time_s,angle_rad,reference_rad,error_rad,"
          "rate_rad_s,reference_rate_rad_s,reference_accel_rad_s2,measured_joint_angle_rad,"
          "measured_gravity_angle_rad,gravity_nm,feedforward_nm,requested_nm,wire_nm,"
          "actuator_nm,disturbance_nm,flags\n", trace);
    puts("PITCH MODEL REGRESSION: 1 kHz C core + noisy-angle gravity helper + actual motor bytes.");
    puts("Default linear SMC; 16 s smooth bidirectional reference; hard limit inset=0.08 rad.");
    puts("Synthetic assumptions: B=.01, smooth Coulomb=.02*tanh(rate/.02) Nm (no stiction), "
         "delay=1 ms, lag=3 ms, noise/disturbance as declared in source; 1 Nm application limit.");
    puts("Tilt is a static +15 deg base rotation about the pitch axis; added point mass=.1 kg at .08 m.");
    puts("Startup [0,1) excluded from tracking only. EACH up/down/hold RMS<.02 rad and peak<.06 rad.");
    puts("No physical state clamping. Wrong-frame diagnostic retains PASS/FAIL but is not acceptance.");
    for (p = 0u; p < count; ++p) {
        if (strcmp(profiles[p].axis, "pitch") != 0) continue;
        for (dm = 0; dm < 2; ++dm) for (condition = 0u; condition < 6u; ++condition) {
            const case_t c = make_case(&profiles[p], dm, condition);
            const int result = run_case(metrics_file, trace, &c);
            if (condition == 5u) {
                if (result == 0) ++diagnostic_passed;
                else ++diagnostic_failed;
                if (result == 1 || result == 2) ++operational_failed;
            } else {
                if (result == 0) ++accepted;
                else ++acceptance_failed;
            }
        }
    }
    if (ferror(metrics_file) || ferror(trace)) io_error = 1;
    if (fclose(metrics_file) != 0) io_error = 1;
    if (fclose(trace) != 0) io_error = 1;
    printf("Summary: acceptance PASS=%u FAIL/INFEASIBLE=%u; diagnostic PASS=%u FAIL/INFEASIBLE=%u; "
           "diagnostic operational failures=%u; IO_ERROR=%d\n", accepted, acceptance_failed,
           diagnostic_passed, diagnostic_failed, operational_failed, io_error);
    return accepted == 0u || acceptance_failed != 0u || operational_failed != 0u || io_error ? 1 : 0;
}
