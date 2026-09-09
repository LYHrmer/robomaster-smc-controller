#!/usr/bin/env python3
"""Plot pitch validation CSVs without changing their PASS/FAIL classification.

Usage: python3 sim/plot_pitch.py metrics.csv trace.csv output_directory
Requires Matplotlib; controller validation is performed by validate_pitch.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PHASES = ("up", "down", "hold")
CONDITIONS = (
    "nominal", "gravity_under20", "gravity_over20", "payload_unmodeled",
    "tilted_correct", "tilted_wrong_frame",
)
CONDITION_LABELS = {
    "nominal": "Nominal",
    "gravity_under20": "Gravity FF x0.8",
    "gravity_over20": "Gravity FF x1.2",
    "payload_unmodeled": "Unmodeled payload",
    "tilted_correct": "15 deg tilt / correct frame",
    "tilted_wrong_frame": "15 deg tilt / wrong frame",
}
MODEL_LABELS = {"rmoss_rmua19": "RMUA19", "dynamicx_standard3": "Standard3"}
COLORS = {"tilted_correct": "#166B89", "tilted_wrong_frame": "#C76130"}


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or any(None in row or None in row.values() for row in rows):
        raise ValueError(f"Empty or malformed CSV: {path}")
    return rows


def row_id(row):
    return {key: row[key] for key in ("model", "motor", "condition", "role", "status")}


def extreme(rows, fields, operation=max):
    candidates = [(float(row[field]), field, row) for row in rows for field in fields
                  if math.isfinite(float(row[field]))]
    if not candidates:
        return None
    value, field, row = operation(candidates, key=lambda item: item[0])
    return {"value": value, "field": field, "case": row_id(row)}


def source_record(path):
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def save_figure(figure, output, basename):
    for suffix in ("png", "svg", "pdf"):
        figure.savefig(output / f"{basename}.{suffix}", dpi=300, bbox_inches="tight")
    plt.close(figure)


def plot_directional_summary(rows, output):
    acceptance = sorted((row for row in rows if row["role"] == "acceptance"),
                        key=lambda row: (row["model"], row["motor"],
                                         CONDITIONS.index(row["condition"])))
    figure, axes = plt.subplots(1, 2, figsize=(12.8, max(6, 0.34 * len(acceptance) + 2.8)), sharey=True)
    labels = [f"{MODEL_LABELS.get(row['model'], row['model'])} / {row['motor']}  |  "
              f"{CONDITION_LABELS[row['condition']]}"
              + (f" [{row['status']}]" if row["status"] != "PASS" else "") for row in acceptance]
    cmap = plt.get_cmap("Blues").copy()
    cmap.set_bad("#DDDDDD")
    cmap.set_over("#B04436")
    mesh = None
    for axis, metric, gate in zip(axes, ("rms", "peak"), (0.02, 0.06)):
        values = np.array([[float(row[f"{phase}_{metric}_rad"]) for phase in PHASES]
                           for row in acceptance])
        normalized = np.ma.masked_invalid(values / gate)
        mesh = axis.imshow(normalized, cmap=cmap, vmin=0, vmax=1, aspect="auto")
        for i in range(len(acceptance)):
            for j in range(len(PHASES)):
                value = values[i, j]
                axis.text(j, i, f"{value * 1000:.2f}" if math.isfinite(value) else "unrun",
                          ha="center", va="center", fontsize=9,
                          color="white" if math.isfinite(value) and value / gate > 0.6 else "#172936")
        axis.set_xticks(range(3), ["Upward", "Downward", "Hold"])
        axis.set_title(f"{metric.upper()} error (mrad)\nEach-direction gate: < {gate * 1000:g} mrad",
                       fontsize=11, pad=12)
        axis.tick_params(axis="both", length=0)
        for i in range(1, len(acceptance)):
            if (acceptance[i]["model"], acceptance[i]["motor"]) != \
               (acceptance[i - 1]["model"], acceptance[i - 1]["motor"]):
                axis.axhline(i - 0.5, color="#4A5964", linewidth=1.0)
        for spine in axis.spines.values():
            spine.set_visible(False)
    axes[0].set_yticks(range(len(labels)), labels, fontsize=9)
    for label, row in zip(axes[0].get_yticklabels(), acceptance):
        if row["status"] != "PASS":
            label.set_color("#A12B28")
    figure.subplots_adjust(left=0.37, right=0.97, top=0.86, bottom=0.15, wspace=0.13)
    color_axis = figure.add_axes([0.44, 0.088, 0.45, 0.017])
    figure.colorbar(mesh, cax=color_axis, orientation="horizontal", extend="max",
                    label="Fraction of each metric's gate (1.0 = threshold)")
    figure.suptitle("Pitch acceptance: errors by motion direction", fontsize=16, y=0.97)
    figure.text(0.67, 0.918, f"{len(acceptance)} acceptance cases | 1 kHz linear SMC | 16 s per case",
                ha="center", fontsize=10)
    figure.text(0.5, 0.012,
                "Synthetic single-axis models. Startup [0,1) excluded from tracking only; "
                "up / down / hold each include all 5 s of their phase.\n"
                "Wrong-frame diagnostics are reported separately. No hardware or soft-limit certification.",
                ha="center", fontsize=9)
    save_figure(figure, output, "pitch_directional_summary")


def plot_tilt_comparison(rows, output):
    selected = [row for row in rows if row["model"] == "dynamicx_standard3" and
                row["condition"] in COLORS]
    if not selected:
        raise ValueError("Missing Standard3 correct/wrong gravity-frame traces")
    groups = collections.defaultdict(list)
    for row in selected:
        groups[(row["motor"], row["condition"])].append(row)
    figure, axes = plt.subplots(3, 2, figsize=(13, 9.2), sharex=True)
    for column, motor in enumerate(("GM6020", "DM4310")):
        for condition in ("tilted_correct", "tilted_wrong_frame"):
            samples = sorted(groups[(motor, condition)], key=lambda row: float(row["time_s"]))
            if not samples:
                raise ValueError(f"Missing {motor}/{condition} trace")
            times = [float(row["time_s"]) for row in samples]
            label = "Correct gravity angle" if condition == "tilted_correct" else "Wrong frame (diagnostic)"
            axes[0, column].plot(times, [float(row["angle_rad"]) for row in samples],
                                 color=COLORS[condition], linewidth=1.3, label=label)
            axes[1, column].plot(times, [1000 * float(row["error_rad"]) for row in samples],
                                 color=COLORS[condition], linewidth=1.0)
            axes[2, column].plot(times, [1000 * (float(row["gravity_nm"]) - float(row["feedforward_nm"]))
                                        for row in samples], color=COLORS[condition], linewidth=1.1)
            if condition == "tilted_correct":
                axes[0, column].plot(times, [float(row["reference_rad"]) for row in samples],
                                     color="#27323B", linestyle="--", linewidth=0.9, label="Reference")
        axes[0, column].set_title(motor, fontsize=12)
        axes[0, column].legend(fontsize=8, loc="lower right")
        for axis in axes[:, column]:
            axis.axvspan(0, 1, color="#BBBBBB", alpha=0.25)
            for start, end in ((3, 4), (7, 8), (11, 12), (14, 16)):
                axis.axvspan(start, end, color="#728778", alpha=0.07)
            axis.grid(alpha=0.22)
            axis.set_xlim(0, 16)
        axes[2, column].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Joint angle (rad)")
    axes[1, 0].set_ylabel("Position error (mrad)")
    axes[2, 0].set_ylabel("G(actual) - feedforward (mN m)")
    figure.suptitle("Standard3 pitch: a static 15 deg tilt exposes gravity-frame error", fontsize=15)
    figure.text(0.5, 0.013,
                "Correct: noisy measured joint angle + base tilt. Diagnostic: the same joint angle with tilt omitted.\n"
                "100 Hz display, 1 kHz metrics; gray = startup, pale green = holds. "
                "Both raw statuses may pass the gate; wrong coordinates remain a model error.",
                ha="center", fontsize=9)
    figure.tight_layout(rect=(0, 0.065, 1, 0.955))
    save_figure(figure, output, "pitch_gravity_frame_comparison")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics", type=Path)
    parser.add_argument("trace", type=Path)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()
    rows, traces = read_csv(args.metrics), read_csv(args.trace)
    statuses = {row["status"] for row in rows}
    if statuses - {"PASS", "FAIL", "INFEASIBLE"}:
        raise ValueError("Unknown raw validation status")
    if {row["role"] for row in rows} != {"acceptance", "diagnostic"}:
        raise ValueError("Both acceptance and diagnostic roles must be present")
    if any(row["condition"] not in CONDITIONS for row in rows):
        raise ValueError("Unknown pitch scenario")
    args.output_directory.mkdir(parents=True, exist_ok=True)
    summary = {
        "qualification": "Synthetic single-axis pitch validation; no hardware, three-axis or soft-limit certification",
        "case_count": len(rows),
        "raw_status_counts": dict(collections.Counter(row["status"] for row in rows)),
        "gates_rad": {"each_direction_rms_exclusive": 0.02, "each_direction_peak_exclusive": 0.06},
        "tracking_startup_exclusion_s": [0, 1],
        "note": "Diagnostic PASS does not establish a correct coordinate transform or acceptable deployment.",
        "sources": {"metrics": source_record(args.metrics), "trace": source_record(args.trace)},
        "groups": {},
    }
    for role in ("acceptance", "diagnostic"):
        group = [row for row in rows if row["role"] == role]
        summary["groups"][role] = {
            "cases": len(group),
            "status_counts": dict(collections.Counter(row["status"] for row in group)),
            "max_directional_rms_rad": extreme(group, [f"{p}_rms_rad" for p in PHASES]),
            "max_directional_peak_rad": extreme(group, [f"{p}_peak_rad" for p in PHASES]),
            "max_wire_torque_nm": extreme(group, ["peak_wire_nm"]),
            "max_saturation_ratio": extreme(group, ["saturation_ratio"]),
            "min_hard_margin_rad": extreme(group, ["min_hard_margin_rad"], min),
            "min_reference_interval_margin_rad": extreme(group, ["min_reference_interval_margin_rad"], min),
        }
    summary["standard3_wrong_over_correct_rms_ratio"] = {}
    for motor in ("GM6020", "DM4310"):
        lookup = {row["condition"]: row for row in rows
                  if row["model"] == "dynamicx_standard3" and row["motor"] == motor}
        summary["standard3_wrong_over_correct_rms_ratio"][motor] = {
            p: float(lookup["tilted_wrong_frame"][f"{p}_rms_rad"]) /
               float(lookup["tilted_correct"][f"{p}_rms_rad"]) for p in PHASES
        }
    (args.output_directory / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    plot_directional_summary(rows, args.output_directory)
    plot_tilt_comparison(traces, args.output_directory)
    print(json.dumps({"cases": len(rows), "groups": {role: data["status_counts"]
                                                     for role, data in summary["groups"].items()}}, indent=2))
    print(f"Figures written to {args.output_directory}")


if __name__ == "__main__":
    main()
