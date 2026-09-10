#!/usr/bin/env python3
"""Reproduce synthetic identification and exercise its headers with actual C.

The deliberately coarse model is an illustrative baseline, not a tuned original
controller. All thresholds apply only to the declared synthetic experiments.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PARAMETERS = ("inertia_kg_m2", "viscous_nm_s_rad", "coulomb_nm",
              "friction_velocity_rad_s", "gravity_sin_nm", "gravity_cos_nm")


def execute(arguments):
    completed = subprocess.run([str(item) for item in arguments], cwd=ROOT,
                               text=True, capture_output=True, check=False)
    if completed.returncode:
        raise RuntimeError("command failed: " + " ".join(map(str, arguments)) +
                           "\n" + completed.stdout + completed.stderr)
    return completed.stdout


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "build-identification")
    parser.add_argument("--cc", default="cc", help="host C99 compiler")
    args = parser.parse_args(argv)
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    execute([sys.executable, ROOT / "tests/identification_fixture.py",
             "--output-dir", directory])
    runs = []
    for axis in ("yaw", "pitch"):
        case = directory / axis
        header = case / "candidate.h"
        execute([sys.executable, ROOT / "tools/identify_gimbal.py",
                 "--train", case / "train.csv", "--validation", case / "validation.csv",
                 "--metadata", case / "metadata.json", "--output", case / "report.json",
                 "--header", header, "--max-validation-rmse-nm", "0.003"])
        executable = case / "validate_identified_model"
        execute([args.cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-Werror",
                 '-DIDENTIFIED_MODEL_HEADER="' + str(header) + '"',
                 "-I", ROOT / "include", ROOT / "sim/validate_identified_model.c",
                 ROOT / "src/gimbal_smc.c", ROOT / "src/gimbal_motor.c", "-lm", "-o", executable])
        truth = json.loads((case / "truth.json").read_text())["parameters"]
        for motor in ("dm", "gm"):
            for condition in ("nominal", "stress"):
                for model in ("coarse", "identified"):
                    record = json.loads(execute([executable, axis, motor, condition, model,
                                                *(truth[key] for key in PARAMETERS)]))
                    runs.append(record)
                    print(f'{axis:5} {record["motor"]:7} {condition:7} {model:10} '
                          f'RMS={record["rms_rad"]:.6f} rad peak={record["peak_rad"]:.6f} rad')
    comparisons = []
    for coarse, identified in zip(runs[::2], runs[1::2]):
        if identified["rms_rad"] >= coarse["rms_rad"]:
            raise RuntimeError("identified candidate failed the declared coarse-baseline comparison")
        comparisons.append({"axis": coarse["axis"], "motor": coarse["motor"],
                            "condition": coarse["condition"],
                            "rms_ratio_identified_to_coarse": identified["rms_rad"] / coarse["rms_rad"]})
    manifest = {"schema_version": 1, "data_kind": "synthetic", "status": "passed",
                "scope": "fixed base and configuration; actual C core and motor packers; no hardware",
                "baseline": "hand-defined coarse model: J*0.7, B*0.5, no gravity/friction feedforward; same gains",
                "plant": {"control_period_s": 0.001, "command_delay_s": 0.003,
                          "actuator_lag_s": 0.003, "duration_s": 18, "score_start_s": 1,
                          "stress": "J*1.2, gravity*1.1, 0.01 N m sinusoidal disturbance"},
                "gates": {"identified_rms_rad_max": 0.035, "identified_peak_rad_max": 0.10,
                          "saturation_events_max": 100, "yaw_final_angle_rad_min": 12.566370614359172,
                          "identified_rms_less_than_coarse": True},
                "runs": runs, "comparisons": comparisons,
                "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in (ROOT / "src/gimbal_smc.c", ROOT / "src/gimbal_motor.c",
                                               ROOT / "sim/validate_identified_model.c", Path(__file__).resolve(),
                                               ROOT / "tools/identify_gimbal.py", ROOT / "tests/identification_fixture.py")}}
    (directory / "summary.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    print(f"PASS: {len(comparisons)} identified-model cases and their coarse baselines; {directory / 'summary.json'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f"validation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
