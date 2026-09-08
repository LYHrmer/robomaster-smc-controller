#!/usr/bin/env python3
"""Plot existing validation CSVs; plotting success is not controller validation.

Usage: python3 sim/plot_open_models.py metrics.csv trace.csv output_directory
Requires Matplotlib. Uses every status in the summary and never relabels failures.
"""
import argparse
import collections
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite_max(rows, key):
    values = [float(row[key]) for row in rows if math.isfinite(float(row[key]))]
    return max(values) if values else None


def save_figure(figure, output, basename):
    figure.savefig(output / f"{basename}.png", dpi=300, bbox_inches="tight")
    figure.savefig(output / f"{basename}.svg", bbox_inches="tight")
    figure.savefig(output / f"{basename}.pdf", bbox_inches="tight")
    plt.close(figure)


def plot_summary(rows, output):
    groups = collections.defaultdict(list)
    for row in rows:
        groups[(row["model"], row["axis"], row["motor"])].append(row)
    keys = sorted(groups)
    figure, axes = plt.subplots(1, 2, figsize=(13, max(4.5, 0.55 * len(keys) + 1.8)), sharey=True)
    for axis, field, threshold, title in zip(
        axes,
        ("rms_after_1s_rad", "peak_after_1s_rad"),
        (0.02, 0.06),
        ("Worst RMS error after 1 s", "Worst peak error after 1 s"),
    ):
        for index, key in enumerate(keys):
            cases = groups[key]
            statuses = collections.Counter(row["status"] for row in cases)
            value = finite_max(cases, field)
            color = "#2B6C9B" if statuses["FAIL"] == 0 and statuses["INFEASIBLE"] == 0 else "#B64C45"
            axis.barh(index, value or 0.0, color=color, height=0.65)
            status_note = ""
            if statuses["FAIL"]:
                status_note += f" FAIL={statuses['FAIL']}"
            if statuses["INFEASIBLE"]:
                status_note += f" INFEASIBLE={statuses['INFEASIBLE']}"
            label = (f"{value:.4f}" if value is not None else "not simulated") + status_note
            axis.text((value or 0.0) + threshold * 0.02, index, label, va="center", fontsize=8)
        axis.axvline(threshold, color="#AD3A34", linestyle="--", linewidth=1.2, label=f"Gate: {threshold:g} rad")
        maximum = max((finite_max(groups[key], field) or 0.0 for key in keys), default=0.0)
        axis.set_xlim(0, max(threshold * 1.3, maximum * 1.5))
        axis.set_title(title)
        axis.set_xlabel("Error (rad)")
        axis.grid(axis="x", alpha=0.25)
        axis.legend(loc="lower right", fontsize=8)
    axes[0].set_yticks(range(len(keys)), [f"{m} / {a}\n{motor}" for m, a, motor in keys])
    axes[0].invert_yaxis()
    figure.suptitle("Open mechanical models: synthetic single-axis regression", fontsize=14)
    figure.text(0.5, 0.005, "Worst across 500/1000/2000 Hz, J x0.5/1/2, hold/sine, linear/terminal, gravity ff x1/0.8.\n"
                "Frozen other joint; assumed motor dynamics and 1 N m limits. No hardware measurements.",
                ha="center", va="bottom", fontsize=8)
    figure.tight_layout(rect=(0, 0.06, 1, 0.96))
    save_figure(figure, output, "open_models_summary")


def plot_traces(rows, output, reference):
    chosen = [row for row in rows if row["reference"] == reference]
    if not chosen:
        return
    profiles = sorted({(row["model"], row["axis"]) for row in chosen})
    groups = collections.defaultdict(list)
    for row in chosen:
        groups[(row["model"], row["axis"], row["motor"], row["controller"])].append(row)
    figure, axes = plt.subplots(len(profiles), 2, figsize=(12, max(3.5, 2.6 * len(profiles))), squeeze=False)
    colors = {"GM6020": "#286D9E", "DM4310": "#B36C23"}
    for index, (model, joint) in enumerate(profiles):
        for motor in ("GM6020", "DM4310"):
            for controller in ("linear", "regularized_terminal"):
                samples = sorted(groups.get((model, joint, motor, controller), []), key=lambda row: float(row["time_s"]))
                if not samples:
                    continue
                times = [float(row["time_s"]) for row in samples]
                line = "-" if controller == "linear" else "--"
                label = f"{motor} / {'linear' if controller == 'linear' else 'terminal'}"
                for axis, field in zip(axes[index], ("error_rad", "wire_nm")):
                    axis.plot(times, [float(row[field]) for row in samples], color=colors[motor],
                              linestyle=line, linewidth=1.0, alpha=0.9, label=label)
        axes[index, 0].set_ylabel("Error (rad)")
        axes[index, 0].set_title(f"{model} / {joint}: tracking error", fontsize=10)
        axes[index, 1].set_title(f"{model} / {joint}: decoded wire torque", fontsize=10)
        axes[index, 1].set_ylabel("Decoded wire torque (N m)")
        for axis in axes[index]:
            axis.axvline(1.0, color="#666666", linewidth=0.7, linestyle=":")
            axis.axvline(2.0, color="#AD3A34", linewidth=0.7, linestyle=":")
            axis.grid(alpha=0.25)
            axis.set_xlabel("Time (s)")
    axes[0, 0].legend(fontsize=7, ncol=2)
    figure.suptitle(f"{reference.capitalize()}: 1 kHz control, nominal J, exact model gravity feedforward", fontsize=13)
    figure.text(0.5, 0.004, "Display sampled at 100 Hz; metrics and limits checked at full simulation rates. "
                "Load step and periodic disturbance start at 2 s. Synthetic results.",
                ha="center", va="bottom", fontsize=8)
    figure.tight_layout(rect=(0, 0.03, 1, 0.96))
    save_figure(figure, output, f"open_models_{reference}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics", type=Path)
    parser.add_argument("trace", type=Path)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()
    rows = read_csv(args.metrics)
    traces = read_csv(args.trace)
    if not rows:
        parser.error("metrics CSV has no cases")
    required = {"model", "axis", "motor", "status", "rms_after_1s_rad", "peak_after_1s_rad", "peak_wire_nm"}
    if not required.issubset(rows[0]):
        parser.error("metrics CSV has incorrect columns")
    statuses = collections.Counter(row["status"] for row in rows)
    if set(statuses) - {"PASS", "FAIL", "INFEASIBLE"}:
        parser.error("unknown validation status")
    args.output_directory.mkdir(parents=True, exist_ok=True)
    summary = {
        "qualification": "Synthetic single-axis model regression; no hardware measurements",
        "cases": len(rows),
        "status_counts": dict(statuses),
        "all_cases_pass": statuses["PASS"] == len(rows),
        "max_rms_after_1s_rad": finite_max(rows, "rms_after_1s_rad"),
        "max_peak_after_1s_rad": finite_max(rows, "peak_after_1s_rad"),
        "max_wire_torque_nm": finite_max(rows, "peak_wire_nm"),
        "sources": {"metrics": str(args.metrics), "trace": str(args.trace)},
    }
    (args.output_directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    plot_summary(rows, args.output_directory)
    for reference in ("hold", "sine"):
        plot_traces(traces, args.output_directory, reference)
    print(json.dumps(summary, indent=2))
    print(f"Figures written to {args.output_directory}")


if __name__ == "__main__":
    main()
