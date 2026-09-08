/* Synthetic single-axis regression. This program calls the production C core.
 * The plant is deliberately motor-independent; see assumptions.md. */
#include "gimbal_smc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_D 3.14159265358979323846
#define PLANT_DT 0.0001
#define DELAY_CAPACITY 10u

typedef struct {
    const char *name;
    unsigned int hz;
    double duration;
    double inertia;
    double damping;
    double initial_angle;
    double load_step;
    double periodic_load;
    double actuator_tau;
    unsigned int delay_substeps;
    int sine_reference;
    int measurement_noise;
} scenario_t;

typedef struct {
    double angle;
    double rate;
    double torque;
    double delay[DELAY_CAPACITY];
    unsigned int delay_head;
} plant_t;

typedef struct {
    double squared_error;
    double evaluation_squared_error;
    double absolute_error_integral;
    double peak_error;
    double evaluation_peak_error;
    double peak_torque;
    double torque_total_variation;
    double final_error;
    unsigned int samples;
    unsigned int evaluation_samples;
    unsigned int saturated_samples;
} metrics_t;

static double absolute_max(double previous, double value)
{
    return fabs(value) > previous ? fabs(value) : previous;
}

static void reference_at(const scenario_t *scenario, double time,
                         double *angle, double *rate, double *accel)
{
    const double amplitude = 0.25;
    const double frequency = 2.0 * PI_D * 0.7;
    if (scenario->sine_reference) {
        *angle = amplitude * sin(frequency * time);
        *rate = amplitude * frequency * cos(frequency * time);
        *accel = -amplitude * frequency * frequency * sin(frequency * time);
    } else {
        *angle = *rate = *accel = 0.0;
    }
}

static double load_at(const scenario_t *scenario, double time)
{
    return time >= 2.0 ? scenario->load_step + scenario->periodic_load *
           sin(2.0 * PI_D * 1.3 * (time - 2.0)) : 0.0;
}

/* RK4 integration of J*w_dot = actuator_torque - B*w + load, with a
 * first-order actuator. The control command is held between controller ticks.
 * The delay FIFO advances at 0.1 ms, independently of the control frequency. */
static void plant_step(plant_t *plant, const scenario_t *scenario,
                        double command, double time)
{
    double delayed = command;
    double q1, q2, q3, q4, v1, v2, v3, v4;
    const double h = PLANT_DT;
    const double b = scenario->damping;
    const double j = scenario->inertia;
    const double tau = scenario->actuator_tau;
    if (scenario->delay_substeps != 0u) {
        delayed = plant->delay[plant->delay_head];
        plant->delay[plant->delay_head] = command;
        plant->delay_head = (plant->delay_head + 1u) % scenario->delay_substeps;
    }
    if (tau == 0.0) {
        plant->torque = delayed;
    }
    q1 = tau > 0.0 ? (delayed - plant->torque) / tau : 0.0;
    v1 = (plant->torque - b * plant->rate + load_at(scenario, time)) / j;
    q2 = tau > 0.0 ? (delayed - (plant->torque + 0.5 * h * q1)) / tau : 0.0;
    v2 = (plant->torque + 0.5 * h * q1 - b * (plant->rate + 0.5 * h * v1) +
          load_at(scenario, time + 0.5 * h)) / j;
    q3 = tau > 0.0 ? (delayed - (plant->torque + 0.5 * h * q2)) / tau : 0.0;
    v3 = (plant->torque + 0.5 * h * q2 - b * (plant->rate + 0.5 * h * v2) +
          load_at(scenario, time + 0.5 * h)) / j;
    q4 = tau > 0.0 ? (delayed - (plant->torque + h * q3)) / tau : 0.0;
    v4 = (plant->torque + h * q3 - b * (plant->rate + h * v3) +
          load_at(scenario, time + h)) / j;
    plant->angle += h * (plant->rate + 2.0 * (plant->rate + 0.5 * h * v1) +
                        2.0 * (plant->rate + 0.5 * h * v2) + plant->rate + h * v3) / 6.0;
    plant->rate += h * (v1 + 2.0 * v2 + 2.0 * v3 + v4) / 6.0;
    plant->torque += h * (q1 + 2.0 * q2 + 2.0 * q3 + q4) / 6.0;
}

static int run_case(FILE *trace, const scenario_t *scenario, int terminal)
{
    gimbal_smc_config_t config;
    gimbal_smc_t controller;
    gimbal_smc_input_t input;
    gimbal_smc_output_t output;
    plant_t plant;
    metrics_t metrics;
    unsigned int tick, substep;
    const unsigned int substeps = 10000u / scenario->hz;
    const unsigned int ticks = (unsigned int)(scenario->duration * scenario->hz);
    const double dt = 1.0 / (double)scenario->hz;
    const char *variant = terminal ? "regularized_terminal" : "linear";
    double previous_command = 0.0;
    double rms, evaluation_rms, saturated_ratio;
    int passed;

    memset(&plant, 0, sizeof(plant));
    memset(&metrics, 0, sizeof(metrics));
    memset(&input, 0, sizeof(input));
    plant.angle = scenario->initial_angle;
    gimbal_smc_default_config(&config);
    config.viscous_nm_s_rad = 0.01f;
    config.terminal_gain = terminal ? 1.5f : 0.0f;
    /* Same rate filter, linear gains and actuation limits for both variants. */
    config.rate_lpf_hz = 80.0f;
    if (!gimbal_smc_init(&controller, &config) ||
        substeps == 0u || scenario->delay_substeps > DELAY_CAPACITY) {
        fprintf(stderr, "Invalid simulation configuration\n");
        return 0;
    }
    input.enable = true;
    input.dt_s = (float)dt;

    for (tick = 0u; tick < ticks; ++tick) {
        const double time = (double)tick * dt;
        double target, target_rate, target_accel, error;
        double angle_noise = 0.0, rate_noise = 0.0;
        reference_at(scenario, time, &target, &target_rate, &target_accel);
        if (scenario->measurement_noise) {
            /* Deterministic continuous-time ripple: identical at shared time
             * points for every sample rate, not a measured sensor spectrum. */
            angle_noise = 0.0001 * (sin(2.0 * PI_D * 31.0 * time) +
                                     0.5 * sin(2.0 * PI_D * 73.0 * time));
            rate_noise = 0.002 * (sin(2.0 * PI_D * 43.0 * time) +
                                   0.5 * sin(2.0 * PI_D * 91.0 * time));
        }
        input.angle_rad = (float)(plant.angle + angle_noise);
        input.rate_rad_s = (float)(plant.rate + rate_noise);
        input.reference_rad = (float)target;
        input.reference_rate_rad_s = (float)target_rate;
        input.reference_accel_rad_s2 = (float)target_accel;
        gimbal_smc_update(&controller, &input, &output);
        if (!output.valid || !isfinite(plant.angle) || !isfinite(plant.rate) ||
            fabs(output.torque_nm) > 0.500001 || fabs(plant.angle) > 2.0 ||
            fabs(plant.rate) > 20.0) {
            fprintf(stderr, "FAIL %s %s %u Hz at %.6f s: invalid/diverging/over-limit\n",
                    scenario->name, variant, scenario->hz, time);
            return 0;
        }
        error = plant.angle - target; /* Metric uses true state, not noisy input. */
        metrics.squared_error += error * error;
        metrics.absolute_error_integral += fabs(error) * dt;
        metrics.peak_error = absolute_max(metrics.peak_error, error);
        metrics.peak_torque = absolute_max(metrics.peak_torque, output.torque_nm);
        metrics.torque_total_variation += fabs(output.torque_nm - previous_command);
        metrics.saturated_samples += (output.flags & GIMBAL_SMC_AMPLITUDE_LIMIT) != 0u;
        ++metrics.samples;
        if (time >= 1.0) {
            metrics.evaluation_squared_error += error * error;
            metrics.evaluation_peak_error = absolute_max(metrics.evaluation_peak_error, error);
            ++metrics.evaluation_samples;
        }
        if (fprintf(trace, "%s,%s,%u,%.7f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u\n",
                    scenario->name, variant, scenario->hz, time, plant.angle, target,
                    error, plant.rate, target_rate, target_accel,
                    (double)output.torque_nm, plant.torque, load_at(scenario, time),
                    (double)output.sliding_rad_s, (unsigned int)output.flags) < 0) {
            perror("writing trace");
            return 0;
        }
        previous_command = output.torque_nm;
        for (substep = 0u; substep < substeps; ++substep) {
            plant_step(&plant, scenario, output.torque_nm, time + substep * PLANT_DT);
        }
    }
    {
        double target, rate, accel;
        reference_at(scenario, scenario->duration, &target, &rate, &accel);
        metrics.final_error = plant.angle - target;
    }
    rms = sqrt(metrics.squared_error / metrics.samples);
    evaluation_rms = sqrt(metrics.evaluation_squared_error / metrics.evaluation_samples);
    saturated_ratio = (double)metrics.saturated_samples / metrics.samples;
    /* Broad, predeclared engineering regression envelopes, not a comparative
     * performance claim and not a hardware acceptance specification. */
    passed = isfinite(plant.angle) && isfinite(plant.rate) &&
             isfinite(metrics.final_error) && evaluation_rms < 0.015 &&
             metrics.evaluation_peak_error < 0.050 &&
             (scenario->sine_reference || fabs(metrics.final_error) < 0.010);
    printf("%s,%s,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
           scenario->name, variant, scenario->hz, rms, evaluation_rms,
           metrics.peak_error, metrics.evaluation_peak_error,
           metrics.absolute_error_integral, metrics.peak_torque, saturated_ratio,
           metrics.torque_total_variation / scenario->duration,
           metrics.final_error, passed ? "PASS" : "FAIL");
    return passed;
}

/* Isolated diagnostic of the published target-difference convention. It is
 * not a reconstruction of the original controller, plant, or motor output.
 * There is intentionally no original output/dead-zone/terminal law here. */
static int derivative_diagnostic(const char *trace_path)
{
    const unsigned int frequencies[] = {500u, 1000u, 2000u};
    char *path = malloc(strlen(trace_path) + sizeof(".derivative.csv"));
    FILE *file;
    unsigned int fi;
    int passed = 1;
    if (path == NULL) {
        return 0;
    }
    strcpy(path, trace_path);
    strcat(path, ".derivative.csv");
    file = fopen(path, "w");
    if (file == NULL) {
        perror(path);
        free(path);
        return 0;
    }
    fputs("hz,time_s,reference_deg,analytic_rate_deg_s,analytic_accel_deg_s2,"
          "native_rate_used_raw,native_accel_raw,current_backward_rate_deg_s,"
          "current_backward_accel_deg_s2\n", file);
    puts("Derivative convention diagnostic (numeric RMS, units differ for native raw fields):");
    puts("hz,native_rate_vs_deg_s_rms,backward_rate_error_deg_s_rms,"
         "native_accel_vs_deg_s2_rms,backward_accel_error_deg_s2_rms,status");
    for (fi = 0u; fi < sizeof(frequencies) / sizeof(frequencies[0]); ++fi) {
        const unsigned int hz = frequencies[fi];
        const double dt = 1.0 / hz;
        const double amplitude = 10.0;
        const double frequency = 2.0 * PI_D;
        double previous_target = 0.0, previous_increment = 0.0, previous_rate = 0.0;
        double raw_rate_mse = 0.0, rate_mse = 0.0, raw_accel_mse = 0.0, accel_mse = 0.0;
        unsigned int tick, samples = 0u;
        int case_pass;
        for (tick = 0u; tick < 2u * hz; ++tick) {
            const double time = tick * dt;
            const double target = amplitude * sin(frequency * time);
            const double analytic_rate = amplitude * frequency * cos(frequency * time);
            const double analytic_accel = -amplitude * frequency * frequency * sin(frequency * time);
            const double increment = target - previous_target;
            const double raw_rate_used = previous_increment;
            const double raw_accel = increment - previous_increment;
            const double rate = increment / dt;
            const double accel = (rate - previous_rate) / dt;
            if (tick >= 2u) { /* Exclude unavailable prehistory only. */
                raw_rate_mse += (raw_rate_used - analytic_rate) * (raw_rate_used - analytic_rate);
                rate_mse += (rate - analytic_rate) * (rate - analytic_rate);
                raw_accel_mse += (raw_accel - analytic_accel) * (raw_accel - analytic_accel);
                accel_mse += (accel - analytic_accel) * (accel - analytic_accel);
                ++samples;
            }
            fprintf(file, "%u,%.7f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    hz, time, target, analytic_rate, analytic_accel,
                    raw_rate_used, raw_accel, rate, accel);
            previous_target = target;
            previous_increment = increment;
            previous_rate = rate;
        }
        case_pass = sqrt(rate_mse / samples) < 0.5 && sqrt(accel_mse / samples) < 4.0;
        printf("%u,%.6f,%.6f,%.6f,%.6f,%s\n", hz,
               sqrt(raw_rate_mse / samples), sqrt(rate_mse / samples),
               sqrt(raw_accel_mse / samples), sqrt(accel_mse / samples),
               case_pass ? "PASS" : "FAIL");
        passed = passed && case_pass;
    }
    if (ferror(file)) {
        passed = 0;
    }
    if (fclose(file) != 0) {
        passed = 0;
    }
    printf("Derivative trace: %s\n", path);
    free(path);
    return passed;
}

int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "simulation.csv";
    const scenario_t scenarios[] = {
        {"hold_load", 1000u, 5.0, 0.01, 0.01, 0.35, -0.06, 0.0, 0.0, 0u, 0, 0},
        {"sine_tracking", 1000u, 6.0, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0u, 1, 0},
        {"heavy_disturbance", 1000u, 5.0, 0.02, 0.015, 0.35, -0.06, 0.02, 0.003, 10u, 0, 1},
        {"sample_rate", 500u, 6.0, 0.01, 0.01, 0.0, -0.04, 0.0, 0.003, 10u, 1, 1},
        {"sample_rate", 1000u, 6.0, 0.01, 0.01, 0.0, -0.04, 0.0, 0.003, 10u, 1, 1},
        {"sample_rate", 2000u, 6.0, 0.01, 0.01, 0.0, -0.04, 0.0, 0.003, 10u, 1, 1}
    };
    FILE *trace;
    size_t i;
    int terminal, passed = 1;
    if (argc > 2) {
        fprintf(stderr, "Usage: %s [trace.csv]\n", argv[0]);
        return EXIT_FAILURE;
    }
    trace = fopen(path, "w");
    if (trace == NULL) {
        perror(path);
        return EXIT_FAILURE;
    }
    fputs("scenario,controller,hz,time_s,angle_rad,reference_rad,error_rad,rate_rad_s,"
          "reference_rate_rad_s,reference_accel_rad_s2,command_nm,applied_nm,load_nm,"
          "sliding_rad_s,flags\n", trace);
    puts("SYNTHETIC / DEMO: no motor identification or hardware measurements.");
    puts("Core: J_nom=0.01 B_nom=0.01 lambda=12 k=10 eta=20 phi=0.1; "
         "terminal alpha=1.5 r=0.5 delta=0.01; torque_limit=0.5 Nm; LPF=80 Hz.");
    puts("scenario,controller,hz,rms_rad,rms_after_1s_rad,peak_rad,peak_after_1s_rad,"
         "iae_rad_s,peak_command_nm,saturation_ratio,command_total_variation_nm_s,"
         "final_error_rad,status");
    for (i = 0u; i < sizeof(scenarios) / sizeof(scenarios[0]); ++i) {
        for (terminal = 0; terminal < 2; ++terminal) {
            if (!run_case(trace, &scenarios[i], terminal)) {
                passed = 0;
            }
        }
    }
    if (ferror(trace)) {
        passed = 0;
    }
    if (fclose(trace) != 0) {
        passed = 0;
    }
    printf("Closed-loop trace: %s\n", path);
    if (!derivative_diagnostic(path)) {
        passed = 0;
    }
    puts(passed ? "Synthetic regression PASS (12 closed-loop cases + 3 derivative diagnostics)" :
                  "Synthetic regression FAIL");
    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
