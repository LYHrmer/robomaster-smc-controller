#!/usr/bin/env python3
"""Plot deterministic synthetic traces with csv + Matplotlib, without pandas.

Example on the development host:
  /usr/bin/python3 -s sim/plot_results.py build/simulation.csv --output-dir sim/results
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


COLORS = {"linear": "#2864a4", "regularized_terminal": "#cc5a24"}
LABELS = {"linear": "Linear SMC", "regularized_terminal": "Regularized terminal SMC"}


def load_traces(path):
    groups = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            key = (row["scenario"], row["controller"], int(row["hz"]))
            numeric = {name: float(value) for name, value in row.items()
                       if name not in ("scenario", "controller")}
            if not all(math.isfinite(value) for value in numeric.values()):
                raise ValueError("Non-finite trace entry")
            groups.setdefault(key, []).append(numeric)
    if not groups:
        raise ValueError("Trace file is empty")
    return groups


def save_figure(fig, output_dir, name):
    fig.savefig(output_dir / (name + ".png"), dpi=220, bbox_inches="tight")
    fig.savefig(output_dir / (name + ".svg"), bbox_inches="tight")
    plt.close(fig)


def write_metrics(groups, output_dir):
    result = {}
    for key, rows in groups.items():
        dt = 1.0 / key[2]
        evaluation = [row for row in rows if row["time_s"] >= 1.0]
        if not evaluation:
            raise ValueError("Trace needs at least one sample after 1 s")
        previous = 0.0
        variation = 0.0
        for row in rows:
            variation += abs(row["command_nm"] - previous)
            previous = row["command_nm"]
        stats = {
            "scenario": key[0], "controller": key[1], "hz": key[2],
            "rms_rad": math.sqrt(sum(r["error_rad"] ** 2 for r in rows) / len(rows)),
            "rms_after_1s_rad": math.sqrt(sum(r["error_rad"] ** 2 for r in evaluation) / len(evaluation)),
            "peak_rad": max(abs(r["error_rad"]) for r in rows),
            "peak_after_1s_rad": max(abs(r["error_rad"]) for r in evaluation),
            "iae_rad_s": sum(abs(r["error_rad"]) for r in rows) * dt,
            "peak_command_nm": max(abs(r["command_nm"]) for r in rows),
            "saturation_ratio": sum(bool(int(r["flags"]) & 64) for r in rows) / len(rows),
            "command_total_variation_nm_s": variation / (len(rows) * dt),
        }
        result[key] = stats
    with (output_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(next(iter(result.values()))))
        writer.writeheader()
        writer.writerows(result.values())
    return result


def overview(groups, output_dir):
    cases = [("hold_load", "Position recovery + load step"),
             ("sine_tracking", "Analytic sine tracking"),
             ("heavy_disturbance", "2x inertia + load + delay + ripple")]
    fig, axes = plt.subplots(3, 3, figsize=(15, 10), constrained_layout=True)
    for row_index, (case, title) in enumerate(cases):
        first = groups[(case, "linear", 1000)]
        axes[row_index, 0].plot([r["time_s"] for r in first],
                               [r["reference_rad"] for r in first],
                               color="#333333", linestyle="--", linewidth=1.2, label="Reference")
        for controller, color in COLORS.items():
            rows = groups[(case, controller, 1000)]
            t = [r["time_s"] for r in rows]
            axes[row_index, 0].plot(t, [r["angle_rad"] for r in rows],
                                   color=color, label=LABELS[controller])
            axes[row_index, 1].plot(t, [r["error_rad"] * 1000 for r in rows], color=color)
            axes[row_index, 2].plot(t, [r["command_nm"] for r in rows], color=color)
        axes[row_index, 0].set_title(title + " | 1000 Hz")
        axes[row_index, 1].set_title("True error after initial recovery")
        axes[row_index, 1].set_xlim(1.0, first[-1]["time_s"])
        # Scale this panel to the evaluation window, not the excluded transient.
        steady_errors = [abs(r["error_rad"]) * 1000
                         for controller in COLORS
                         for r in groups[(case, controller, 1000)] if r["time_s"] >= 1.0]
        bound = max(0.5, max(steady_errors) * 1.15)
        axes[row_index, 1].set_ylim(-bound, bound)
        axes[row_index, 2].set_title("Command torque (limit +/-0.5 N m)")
        for axis, ylabel in zip(axes[row_index], ("Angle [rad]", "Error [mrad]", "Torque [N m]")):
            axis.set_xlabel("Time [s]")
            axis.set_ylabel(ylabel)
            axis.grid(alpha=0.22)
            if case != "sine_tracking":
                axis.axvline(2.0, color="#777777", linestyle=":", linewidth=1)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("SYNTHETIC / DEMO: shared torque limits; additional terminal gain changes bandwidth\n"
                 "Model regression only - no GM6020 / DM4310 hardware performance claim", fontsize=13)
    save_figure(fig, output_dir, "closed_loop_comparison")


def frequency_comparison(groups, metrics, output_dir):
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5), constrained_layout=True)
    for controller, color in COLORS.items():
        hz = sorted(key[2] for key in metrics if key[:2] == ("sample_rate", controller))
        axes[0, 0].plot(hz, [metrics[("sample_rate", controller, f)]["rms_after_1s_rad"] * 1000 for f in hz],
                        "o-", color=color, label=LABELS[controller])
        axes[0, 1].plot(hz, [metrics[("sample_rate", controller, f)]["command_total_variation_nm_s"] for f in hz],
                        "o-", color=color)
        rows = groups[("sample_rate", controller, 500)]
        axes[1, 0].plot([r["time_s"] for r in rows if r["time_s"] >= 1.0],
                        [r["error_rad"] * 1000 for r in rows if r["time_s"] >= 1.0], color=color)
        tail = [r for r in rows if 5.7 <= r["time_s"] <= 5.9]
        axes[1, 1].plot([r["time_s"] for r in tail], [r["command_nm"] for r in tail], color=color)
    axes[0, 0].set(xlabel="Control rate [Hz]", ylabel="RMSE after 1 s [mrad]",
                   title="Tracking error: true state vs analytic target")
    axes[0, 1].set(xlabel="Control rate [Hz]", ylabel="Command TV / duration [N m/s]",
                   title="Total variation includes intentional tracking torque")
    axes[1, 0].set(xlabel="Time [s]", ylabel="Error [mrad]", title="500 Hz error including the load step")
    axes[1, 1].set(xlabel="Time [s]", ylabel="Command [N m]", title="500 Hz command ripple detail")
    axes[0, 0].legend(fontsize=8)
    for axis in axes.flat:
        axis.grid(alpha=0.22)
    fig.suptitle("SYNTHETIC sample-rate check: fixed 1 ms command delay + 3 ms actuator lag\n"
                 "These settings reduce terminal-SMC error while increasing command variation", fontsize=12)
    save_figure(fig, output_dir, "sample_rate_tradeoff")


def derivative_plot(path, output_dir):
    diagnostic = Path(str(path) + ".derivative.csv")
    with diagnostic.open(newline="", encoding="utf-8") as stream:
        rows = [{name: float(value) for name, value in row.items()}
                for row in csv.DictReader(stream)
                if int(row["hz"]) == 1000 and 0.02 <= float(row["time_s"]) <= 1.0]
    t = [r["time_s"] for r in rows]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    comparisons = [(0, "analytic_rate_deg_s", "current_backward_rate_deg_s", "Rate [deg/s]"),
                   (1, "analytic_accel_deg_s2", "current_backward_accel_deg_s2", "Acceleration [deg/s^2]")]
    for index, analytic, backward, label in comparisons:
        axes[index, 0].plot(t, [r[analytic] for r in rows], color="#333333", label="Analytic derivative")
        axes[index, 0].plot(t, [r[backward] for r in rows], "--", color=COLORS["linear"], label="Current backward difference / dt")
        axes[index, 0].set_ylabel(label)
    axes[0, 1].plot(t, [r["native_rate_used_raw"] for r in rows], color=COLORS["regularized_terminal"])
    axes[0, 1].set_ylabel("Previous target increment [deg/sample]")
    axes[1, 1].plot(t, [r["native_accel_raw"] for r in rows], color=COLORS["regularized_terminal"])
    axes[1, 1].set_ylabel("Target second difference [deg/sample^2]")
    axes[0, 0].set_title("Physical derivatives with explicit seconds")
    axes[0, 1].set_title("Published difference convention: separate units")
    axes[0, 0].legend(fontsize=8)
    for axis in axes.flat:
        axis.set_xlabel("Time [s]")
        axis.grid(alpha=0.22)
    fig.suptitle("SYNTHETIC isolated target-difference diagnostic | 1000 Hz, 10 deg, 1 Hz sine\n"
                 "This is not a reproduction of the original full controller or its hardware", fontsize=12)
    save_figure(fig, output_dir, "target_derivative_diagnostic")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace_csv", type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()
    output_dir = args.output_dir or args.trace_csv.parent / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "svg.fonttype": "none"})
    groups = load_traces(args.trace_csv)
    metrics = write_metrics(groups, output_dir)
    overview(groups, output_dir)
    frequency_comparison(groups, metrics, output_dir)
    derivative_plot(args.trace_csv, output_dir)
    print(f"Wrote 3 PNG + 3 SVG figures and metrics.csv to {output_dir.resolve()}")


if __name__ == "__main__":
    main()
