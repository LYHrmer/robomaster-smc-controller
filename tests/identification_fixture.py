#!/usr/bin/env python3
"""Generate reproducible synthetic *offline* identification data.

This is an independent analytical rigid-axis fixture, not motor measurements
and not a simulation of the SMC feedback loop.  Position, velocity and
acceleration are evaluated analytically; torque follows the declared plant.
The validation trajectories use frequencies/phases absent from training.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


FIELDS = (
    "time_s", "angle_rad", "rate_rad_s", "gravity_angle_rad", "torque_nm",
    "enabled", "valid", "saturated", "segment",
)


@dataclass(frozen=True)
class Truth:
    inertia_kg_m2: float
    viscous_nm_s_rad: float
    coulomb_nm: float
    friction_velocity_rad_s: float
    gravity_sin_nm: float
    gravity_cos_nm: float


TRUTH = {
    "yaw": Truth(0.018, 0.008, 0.045, 0.09, 0.0, 0.0),
    "pitch": Truth(0.012, 0.011, 0.025, 0.075, 0.055, 0.19),
}


def metadata(axis: str) -> dict:
    """The torque scale is known by construction, not inferred from CAN."""
    return {
        "schema_version": 1,
        "axis": axis,
        "configuration": "synthetic_fixed_plane_v1",
        "data_kind": "synthetic",
        "torque_source": "synthetic_truth",
        "torque_time_aligned": True,
        "base_motion": "fixed",
        "angle_continuous": True,
        "gravity_frame": "fixed_plane",
        "friction_velocity_rad_s": TRUTH[axis].friction_velocity_rad_s,
    }


def state(axis: str, split: str, time_s: float) -> tuple[float, float, float, float]:
    """Return angle, its exact derivatives, and the gravity phase in rad."""
    if split not in ("train", "validation"):
        raise ValueError("split must be train or validation")
    validation = split == "validation"
    if axis == "yaw":
        offset = 4.0 * math.pi if not validation else -5.0 * math.pi
        drift = 0.85 if not validation else -0.72
        amplitudes = (1.2, 0.45, 0.12)
        frequencies = (0.17, 0.59, 1.13) if not validation else (0.23, 0.71, 1.37)
        phases = (0.1, 0.7, -0.3) if not validation else (1.1, -0.5, 0.4)
        base_angle = 0.0
    elif axis == "pitch":
        offset, drift = (-0.05 if not validation else 0.04), 0.0
        amplitudes = (0.38, 0.13, 0.045)
        frequencies = (0.13, 0.47, 1.03) if not validation else (0.19, 0.61, 1.29)
        phases = (0.2, -0.7, 0.3) if not validation else (0.8, 0.5, -1.2)
        # Each recording has a fixed base. The second recording deliberately
        # changes its constant tilt to distinguish gravity from encoder angle.
        base_angle = 0.18 if not validation else -0.16
    else:
        raise ValueError("axis must be yaw or pitch")
    angle = offset + drift * time_s
    rate, acceleration = drift, 0.0
    for amplitude, frequency, phase in zip(amplitudes, frequencies, phases):
        omega = 2.0 * math.pi * frequency
        argument = omega * time_s + phase
        angle += amplitude * math.sin(argument)
        rate += amplitude * omega * math.cos(argument)
        acceleration -= amplitude * omega * omega * math.sin(argument)
    return angle, rate, acceleration, angle + base_angle


def plant_torque(truth: Truth, rate: float, acceleration: float, gravity: float) -> float:
    return (
        truth.inertia_kg_m2 * acceleration
        + truth.viscous_nm_s_rad * rate
        + truth.coulomb_nm * math.tanh(rate / truth.friction_velocity_rad_s)
        + truth.gravity_sin_nm * math.sin(gravity)
        + truth.gravity_cos_nm * math.cos(gravity)
    )


def rows(axis: str, split: str, *, duration_s: float = 16.0,
         dt_s: float = 0.002, truth: Truth | None = None) -> list[dict]:
    if duration_s <= 0.0 or dt_s <= 0.0:
        raise ValueError("duration_s and dt_s must be positive")
    truth = TRUTH[axis] if truth is None else truth
    result = []
    for index in range(round(duration_s / dt_s) + 1):
        time_s = index * dt_s
        angle, rate, acceleration, gravity = state(axis, split, time_s)
        result.append(dict(zip(FIELDS, (
            time_s, angle, rate, gravity,
            plant_torque(truth, rate, acceleration, gravity),
            1, 1, 0, 0,
        ))))
    return result


def write_csv(path: Path, samples: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for sample in samples:
            # Round-trip precision keeps numerical integration as the primary
            # approximation; no differentiated measurement creates the truth.
            writer.writerow({key: format(value, ".17g") if isinstance(value, float)
                             else value for key, value in sample.items()})


def generate(output_dir: Path, *, duration_s: float = 16.0,
             dt_s: float = 0.002) -> dict[str, dict[str, Path]]:
    generated = {}
    for axis in TRUTH:
        directory = output_dir / axis
        directory.mkdir(parents=True, exist_ok=True)
        paths = {key: directory / (key + suffix) for key, suffix in (
            ("train", ".csv"), ("validation", ".csv"),
            ("metadata", ".json"), ("truth", ".json"),
        )}
        for split in ("train", "validation"):
            write_csv(paths[split], rows(axis, split, duration_s=duration_s, dt_s=dt_s))
        paths["metadata"].write_text(json.dumps(metadata(axis), indent=2) + "\n")
        details = {
            "parameters": asdict(TRUTH[axis]),
            "duration_s": duration_s,
            "sample_period_s": dt_s,
            "data_kind": "synthetic",
            "generation": "analytical multifrequency trajectories; exact plant torque",
            "closed_loop": False,
            "noise": "none; identification still numerically integrates sampled signals",
            "base_tilt_train_rad": 0.18 if axis == "pitch" else 0.0,
            "base_tilt_validation_rad": -0.16 if axis == "pitch" else 0.0,
            "limitations": [
                "No actuator lag, CAN delay, sensor noise, stiction or folding coupling.",
                "Recovery of known synthetic parameters does not establish unbiased closed-loop LS.",
                "Parameters are fixture values, not measured or recommended motor gains.",
            ],
        }
        paths["truth"].write_text(json.dumps(details, indent=2) + "\n")
        generated[axis] = paths
    return generated


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=16.0)
    parser.add_argument("--dt-s", type=float, default=0.002)
    args = parser.parse_args()
    generated = generate(args.output_dir, duration_s=args.duration_s, dt_s=args.dt_s)
    print(json.dumps({axis: {key: str(path) for key, path in paths.items()}
                      for axis, paths in generated.items()}, indent=2))


if __name__ == "__main__":
    main()
