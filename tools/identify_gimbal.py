#!/usr/bin/env python3
"""Offline, fixed-configuration gimbal identification; Python 3.9+ and NumPy.

This tool estimates candidate physical parameters, not controller gains.  It
uses non-overlapping trapezoidal integral windows, requires an independent
validation record, and never enables or communicates with an actuator.
"""

import argparse
import csv
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


CSV_COLUMNS = (
    "time_s", "angle_rad", "rate_rad_s", "gravity_angle_rad", "torque_nm",
    "enabled", "valid", "saturated", "segment",
)
PARAMETER_NAMES = (
    "inertia_kg_m2", "viscous_nm_s_rad", "coulomb_nm",
    "gravity_sin_nm", "gravity_cos_nm",
)


class IdentificationError(ValueError):
    """Data, excitation, physical parameter, or acceptance check failed."""


@dataclass
class Dataset:
    path: str
    sha256: str
    columns: Dict[str, np.ndarray]

    @property
    def row_count(self) -> int:
        return len(self.columns["time_s"])


@dataclass
class WindowSet:
    axis: str
    design: np.ndarray
    target: np.ndarray
    durations_s: np.ndarray
    start_times_s: np.ndarray
    end_times_s: np.ndarray
    discarded_sample_count: int
    segment_break_count: int
    gap_break_count: int


@dataclass
class FitResult:
    parameters: Dict[str, float]
    metrics: dict


def _positive(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise IdentificationError(name + " must be a finite positive number")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise IdentificationError(name + " must be a finite positive number")
    return number


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _json_object(pairs: list) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise IdentificationError("duplicate metadata key: " + key)
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise IdentificationError("metadata contains a non-finite number: " + value)


def read_dataset(path: Path) -> Dataset:
    """Read the exact CSV schema; reject malformed rows, NaN/Inf, and bad time.

    Invalid/disabled/saturated rows still need finite numbers and increasing
    timestamps. They remain in the record as explicit integration boundaries.
    """
    source = Path(path)
    raw = source.read_bytes()
    try:
        reader = csv.reader(io.StringIO(raw.decode("utf-8-sig")), strict=True)
        header = next(reader, None)
        if header != list(CSV_COLUMNS):
            raise IdentificationError("CSV header must be: " + ",".join(CSV_COLUMNS))
        values: List[List[float]] = []
        last_time = None
        for line, row in enumerate(reader, 2):
            if len(row) != len(CSV_COLUMNS):
                raise IdentificationError("CSV row %d has the wrong field count" % line)
            fields = [field.strip() for field in row]
            try:
                numbers = [float(field) for field in fields]
            except ValueError as error:
                raise IdentificationError("CSV row %d contains a nonnumeric field" % line) from error
            if not all(math.isfinite(number) for number in numbers):
                raise IdentificationError("CSV row %d contains NaN or infinity" % line)
            for index in (5, 6, 7):
                if fields[index] not in ("0", "1"):
                    raise IdentificationError("CSV row %d flags must be literal 0 or 1" % line)
            if not numbers[8].is_integer():
                raise IdentificationError("CSV row %d segment must be an integer" % line)
            if last_time is not None and numbers[0] <= last_time:
                raise IdentificationError("CSV timestamps must be strictly increasing (row %d)" % line)
            last_time = numbers[0]
            values.append(numbers)
    except (UnicodeError, csv.Error) as error:
        raise IdentificationError("invalid UTF-8 CSV: " + str(error)) from error
    if len(values) < 2:
        raise IdentificationError("CSV requires at least two samples")
    matrix = np.asarray(values, dtype=np.float64)
    return Dataset(str(source.resolve()), _sha256(raw), {
        name: matrix[:, index] for index, name in enumerate(CSV_COLUMNS)
    })


def _read_metadata(path: Path) -> Tuple[dict, str]:
    raw = Path(path).read_bytes()
    try:
        metadata = json.loads(raw.decode("utf-8"), object_pairs_hook=_json_object,
                              parse_constant=_reject_json_constant)
    except (UnicodeError, json.JSONDecodeError) as error:
        raise IdentificationError("invalid metadata JSON: " + str(error)) from error
    if not isinstance(metadata, dict):
        raise IdentificationError("metadata must be a JSON object")
    # JSON's numeric grammar also accepts exponents that overflow a double.
    def check_finite(value: object) -> None:
        if isinstance(value, float) and not math.isfinite(value):
            raise IdentificationError("metadata numbers must remain finite")
        if isinstance(value, dict):
            for item in value.values():
                check_finite(item)
        elif isinstance(value, list):
            for item in value:
                check_finite(item)
    check_finite(metadata)
    if type(metadata.get("schema_version")) is not int or metadata["schema_version"] != 1:
        raise IdentificationError("metadata schema_version must be integer 1")
    if metadata.get("axis") not in ("yaw", "pitch"):
        raise IdentificationError("metadata axis must be yaw or pitch")
    if not isinstance(metadata.get("configuration"), str) or not metadata["configuration"].strip():
        raise IdentificationError("metadata configuration must be a nonempty string")
    if metadata.get("data_kind") not in ("synthetic", "hardware"):
        raise IdentificationError("metadata data_kind must be synthetic or hardware")
    if metadata.get("torque_source") not in ("synthetic_truth", "calibrated_feedback"):
        raise IdentificationError("torque_source requires synthetic_truth or calibrated_feedback")
    if metadata["torque_source"] == "synthetic_truth" and metadata["data_kind"] != "synthetic":
        raise IdentificationError("synthetic_truth is only valid for synthetic data")
    for name in ("torque_time_aligned", "angle_continuous"):
        if metadata.get(name) is not True:
            raise IdentificationError("metadata " + name + " must be true")
    if metadata.get("base_motion") != "fixed":
        raise IdentificationError("V1 requires base_motion=fixed")
    if metadata.get("gravity_frame") != "fixed_plane":
        raise IdentificationError("V1 requires gravity_frame=fixed_plane")
    metadata["friction_velocity_rad_s"] = _positive(
        metadata.get("friction_velocity_rad_s"), "friction_velocity_rad_s")
    return metadata, _sha256(raw)


def read_metadata(path: Path) -> dict:
    """Validate the metadata assumptions; this does not calibrate sensors."""
    return _read_metadata(path)[0]


def build_windows(dataset: Dataset, *, axis: str,
                  friction_velocity_rad_s: float,
                  window_s: float = 0.05, max_gap_s: float = 0.02) -> WindowSet:
    """Build mean-torque equations from disjoint integration intervals.

    Columns are delta(rate)/T, delta(angle)/T, mean(tanh(rate/eps)),
    and, for pitch, mean(sin(gravity_angle)), mean(cos(gravity_angle)).
    Integral means use trapezoidal quadrature. Only endpoints can be shared;
    a trailing incomplete window is discarded. Window duration is at least
    window_s (within rounding tolerance) and may exceed it by one sample gap.
    """
    if axis not in ("yaw", "pitch"):
        raise IdentificationError("axis must be yaw or pitch")
    epsilon = _positive(friction_velocity_rad_s, "friction_velocity_rad_s")
    duration = _positive(window_s, "window_s")
    max_gap = _positive(max_gap_s, "max_gap_s")
    c = dataset.columns
    good = (c["enabled"] == 1) & (c["valid"] == 1) & (c["saturated"] == 0)
    design, targets, durations, starts, ends = [], [], [], [], []
    start: Optional[int] = None
    segment_breaks = 0
    gap_breaks = 0
    with np.errstate(over="raise", invalid="raise", divide="raise"):
        try:
            for index in range(dataset.row_count):
                if not good[index]:
                    start = None
                    continue
                if start is None:
                    start = index
                    continue
                if c["segment"][index] != c["segment"][index - 1]:
                    segment_breaks += 1
                    start = index
                    continue
                gap = c["time_s"][index] - c["time_s"][index - 1]
                if gap > max_gap:
                    gap_breaks += 1
                    start = index
                    continue
                elapsed = c["time_s"][index] - c["time_s"][start]
                if elapsed < duration * (1.0 - 1e-12):
                    continue
                selection = slice(start, index + 1)
                dt = np.diff(c["time_s"][selection])

                def mean_integral(samples: np.ndarray) -> float:
                    return float(np.sum((0.5 * samples[:-1] + 0.5 * samples[1:]) * dt) / elapsed)

                row = [
                    (c["rate_rad_s"][index] - c["rate_rad_s"][start]) / elapsed,
                    (c["angle_rad"][index] - c["angle_rad"][start]) / elapsed,
                    mean_integral(np.tanh(c["rate_rad_s"][selection] / epsilon)),
                ]
                if axis == "pitch":
                    gravity = c["gravity_angle_rad"][selection]
                    row.extend([mean_integral(np.sin(gravity)), mean_integral(np.cos(gravity))])
                design.append(row)
                targets.append(mean_integral(c["torque_nm"][selection]))
                durations.append(float(elapsed))
                starts.append(float(c["time_s"][start]))
                ends.append(float(c["time_s"][index]))
                start = index
        except FloatingPointError as error:
            raise IdentificationError("non-finite integral regression: " + str(error)) from error
    parameter_count = 3 if axis == "yaw" else 5
    matrix = np.asarray(design, dtype=np.float64).reshape((-1, parameter_count))
    target = np.asarray(targets, dtype=np.float64)
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(target)):
        raise IdentificationError("integral regression must remain finite")
    return WindowSet(axis, matrix, target, np.asarray(durations), np.asarray(starts),
                     np.asarray(ends), int(np.count_nonzero(~good)), segment_breaks, gap_breaks)


def _excitation(windows: WindowSet, max_condition: float) -> Tuple[np.ndarray, np.ndarray, dict]:
    limit = _positive(max_condition, "max_condition")
    if limit < 1.0:
        raise IdentificationError("max_condition must be at least 1")
    matrix = windows.design
    count, parameter_count = matrix.shape
    required = max(20, 5 * parameter_count)
    if count < required:
        raise IdentificationError("insufficient windows: %d available, %d required" % (count, required))
    # Scale first by each column's largest element to avoid squaring huge values.
    maxima = np.max(np.abs(matrix), axis=0)
    if np.any(maxima == 0.0):
        raise IdentificationError("rank deficient regression: an unexcited column is zero")
    scaled = matrix / maxima
    relative_norms = np.linalg.norm(scaled, axis=0)
    scales = maxima * relative_norms
    if not np.all(np.isfinite(scales)) or np.any(scales <= 0.0):
        raise IdentificationError("non-finite regression column scales")
    normalized = scaled / relative_norms
    singular = np.linalg.svd(normalized, compute_uv=False)
    tolerance = np.finfo(np.float64).eps * max(normalized.shape) * singular[0]
    rank = int(np.count_nonzero(singular > tolerance))
    if rank != parameter_count:
        raise IdentificationError("rank deficient regression: %d of %d" % (rank, parameter_count))
    condition = float(singular[0] / singular[-1])
    if not math.isfinite(condition) or condition > limit:
        raise IdentificationError("regression condition number %.6g exceeds %.6g" % (condition, limit))
    metrics = {
        "window_count": count,
        "duration_s": float(np.sum(windows.durations_s)),
        "window_duration_min_s": float(np.min(windows.durations_s)),
        "window_duration_max_s": float(np.max(windows.durations_s)),
        "rank": rank,
        "condition_number": condition,
        "singular_values": singular.tolist(),
        "column_l2_scales": scales.tolist(),
        "discarded_sample_count": windows.discarded_sample_count,
        "segment_break_count": windows.segment_break_count,
        "gap_break_count": windows.gap_break_count,
    }
    return normalized, scales, metrics


def _check_parameters(parameters: Dict[str, float]) -> None:
    if parameters["inertia_kg_m2"] <= 0.0:
        raise IdentificationError("identified inertia must be positive")
    if parameters["viscous_nm_s_rad"] < 0.0 or parameters["coulomb_nm"] < 0.0:
        raise IdentificationError("identified viscous and Coulomb friction must be nonnegative")
    for name, value in parameters.items():
        if not math.isfinite(value) or abs(value) > np.finfo(np.float32).max:
            raise IdentificationError(name + " is not representable as finite float32")
        converted = float(np.float32(value))
        if value != 0.0 and converted == 0.0:
            raise IdentificationError(name + " underflows float32")


def _residual_metrics(windows: WindowSet, coefficients: np.ndarray) -> dict:
    with np.errstate(over="raise", invalid="raise"):
        try:
            residual = windows.design @ coefficients - windows.target
            peak = float(np.max(np.abs(residual)))
            rmse = 0.0 if peak == 0.0 else float(peak * np.sqrt(np.mean((residual / peak) ** 2)))
            bias = float(np.mean(residual))
        except FloatingPointError as error:
            raise IdentificationError("non-finite regression residual") from error
    if not all(math.isfinite(x) for x in (peak, rmse, bias)):
        raise IdentificationError("non-finite regression residual")
    return {"mean_torque_rmse_nm": rmse, "mean_torque_bias_nm": bias,
            "mean_torque_peak_abs_error_nm": peak}


def fit(windows: WindowSet, *, max_condition: float = 1e6) -> FitResult:
    """Column-normalized SVD/lstsq fit; reject rather than clip bad parameters."""
    normalized, scales, metrics = _excitation(windows, max_condition)
    coefficients = np.linalg.lstsq(normalized, windows.target, rcond=None)[0] / scales
    names = PARAMETER_NAMES[:windows.design.shape[1]]
    parameters = {name: float(value) for name, value in zip(names, coefficients)}
    parameters.setdefault("gravity_sin_nm", 0.0)
    parameters.setdefault("gravity_cos_nm", 0.0)
    _check_parameters(parameters)
    metrics.update(_residual_metrics(windows, coefficients))
    return FitResult(parameters, metrics)


def evaluate(windows: WindowSet, parameters: Dict[str, float], *, max_condition: float = 1e6) -> dict:
    """Check independent excitation and fixed-model mean-torque residuals."""
    _, _, metrics = _excitation(windows, max_condition)
    _check_parameters(parameters)
    coefficients = np.asarray([parameters[name] for name in PARAMETER_NAMES[:windows.design.shape[1]]])
    metrics.update(_residual_metrics(windows, coefficients))
    return metrics


def _header(parameters: Dict[str, float], report: dict) -> str:
    prefix = report["symbol_prefix"]
    # Preserve case so distinct C prefixes such as yaw / YAW cannot collide.
    guard = prefix + "_IDENTIFIED_PARAMETERS_H_INCLUDED"
    lines = [
        "/* Offline identification candidate only: validate before hardware use.",
        " * Fixed base/configuration; no SMC gains are changed or selected.",
        " * Data kind: " + report["data_kind"] + ".",
        " * Train SHA256: " + report["provenance"]["train_sha256"],
        " * Validation SHA256: " + report["provenance"]["validation_sha256"],
        " */", "#ifndef " + guard, "#define " + guard, "",
    ]
    for name in ("inertia_kg_m2", "viscous_nm_s_rad", "coulomb_nm", "friction_velocity_rad_s",
                 "gravity_sin_nm", "gravity_cos_nm"):
        value = float(np.float32(parameters[name]))
        lines.append("static const float %s_%s = %.9ef;" % (prefix, name, value))
    lines.extend(["", "#endif", ""])
    return "\n".join(lines)


def _write_outputs(outputs: Dict[Path, str]) -> None:
    """Stage all valid artifacts before replacing existing successful outputs."""
    staged: Dict[Path, Path] = {}
    original: Dict[Path, Optional[bytes]] = {}
    committed: List[Path] = []
    try:
        for destination, content in outputs.items():
            destination.parent.mkdir(parents=True, exist_ok=True)
            original[destination] = destination.read_bytes() if destination.exists() else None
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                             dir=str(destination.parent), delete=False) as stream:
                staged[destination] = Path(stream.name)
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
        for destination, temporary in staged.items():
            os.replace(str(temporary), str(destination))
            committed.append(destination)
    except OSError:
        # Roll back a partial multi-file commit; no invalid result is published.
        for destination in reversed(committed):
            previous = original[destination]
            if previous is None:
                destination.unlink()
            else:
                destination.write_bytes(previous)
        raise
    finally:
        for temporary in staged.values():
            if temporary.exists():
                temporary.unlink()


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", required=True, type=Path)
    parser.add_argument("--validation", required=True, type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="candidate JSON report")
    parser.add_argument("--header", type=Path, help="optional C float constants; no gains")
    parser.add_argument("--symbol-prefix", default="smc_ident",
                        help="C constant prefix, ASCII letter then letters/digits/underscores")
    parser.add_argument("--window-s", type=float, default=0.05)
    parser.add_argument("--max-gap-s", type=float, default=0.02)
    parser.add_argument("--max-condition", type=float, default=1e6)
    parser.add_argument("--max-validation-rmse-nm", required=True, type=float,
                        help="maximum RMS residual of window-mean torque, not pointwise torque")
    args = parser.parse_args(argv)
    try:
        threshold = _positive(args.max_validation_rmse_nm, "max_validation_rmse_nm")
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", args.symbol_prefix) is None:
            raise IdentificationError("symbol-prefix must start with an ASCII letter and contain only letters, digits or underscores")
        destinations = [args.output.resolve()]
        if args.header is not None:
            destinations.append(args.header.resolve())
        protected = {args.train.resolve(), args.validation.resolve(), args.metadata.resolve(), Path(__file__).resolve()}
        if len(set(destinations)) != len(destinations) or any(path in protected for path in destinations):
            raise IdentificationError("output paths must be distinct and must not overwrite inputs or this tool")
        metadata, metadata_hash = _read_metadata(args.metadata)
        train, validation = read_dataset(args.train), read_dataset(args.validation)
        if train.sha256 == validation.sha256:
            raise IdentificationError("training and validation content hashes must differ")
        settings = {"axis": metadata["axis"], "friction_velocity_rad_s": metadata["friction_velocity_rad_s"],
                    "window_s": args.window_s, "max_gap_s": args.max_gap_s}
        train_windows = build_windows(train, **settings)
        validation_windows = build_windows(validation, **settings)
        result = fit(train_windows, max_condition=args.max_condition)
        result.parameters["friction_velocity_rad_s"] = metadata["friction_velocity_rad_s"]
        _check_parameters(result.parameters)
        validation_metrics = evaluate(validation_windows, result.parameters, max_condition=args.max_condition)
        if validation_metrics["mean_torque_rmse_nm"] > threshold:
            raise IdentificationError("validation mean-torque RMSE %.6g exceeds %.6g Nm" %
                                      (validation_metrics["mean_torque_rmse_nm"], threshold))
        exported = {name: float(np.float32(value)) for name, value in result.parameters.items()}
        exported_settings = dict(settings, friction_velocity_rad_s=exported["friction_velocity_rad_s"])
        exported_train_windows = build_windows(train, **exported_settings)
        exported_validation_windows = build_windows(validation, **exported_settings)
        exported_train_metrics = evaluate(exported_train_windows, exported, max_condition=args.max_condition)
        exported_validation_metrics = evaluate(exported_validation_windows, exported, max_condition=args.max_condition)
        if exported_validation_metrics["mean_torque_rmse_nm"] > threshold:
            raise IdentificationError("float32 export validation mean-torque RMSE %.6g exceeds %.6g Nm" %
                                      (exported_validation_metrics["mean_torque_rmse_nm"], threshold))
        report = {
            "schema_version": 1, "status": "candidate_only", "axis": metadata["axis"],
            "configuration": metadata["configuration"], "data_kind": metadata["data_kind"],
            "symbol_prefix": args.symbol_prefix,
            "model": "tau = J*qdd + B*qd + Fc*tanh(qd/epsilon) + A*sin(qg) + C*cos(qg)",
            "parameters": result.parameters,
            "parameter_units": {"inertia_kg_m2": "kg*m^2", "viscous_nm_s_rad": "N*m*s/rad",
                                "coulomb_nm": "N*m", "friction_velocity_rad_s": "rad/s",
                                "gravity_sin_nm": "N*m", "gravity_cos_nm": "N*m"},
            "provenance": {"train_sha256": train.sha256, "validation_sha256": validation.sha256,
                           "metadata_sha256": metadata_hash, "tool_sha256": _sha256(Path(__file__).read_bytes())},
            "metadata": metadata,
            "settings": {"window_s": args.window_s, "max_gap_s": args.max_gap_s,
                         "max_condition": args.max_condition, "max_validation_rmse_nm": threshold},
            "data": {"train_row_count": train.row_count, "validation_row_count": validation.row_count},
            "metrics": {"train": result.metrics, "validation": validation_metrics},
            "export_float32": {
                "parameters": exported,
                "metrics": {"train": exported_train_metrics, "validation": exported_validation_metrics},
                "validation": "Same independent window-mean torque threshold after float32 rounding, rebuilding friction regressors with rounded epsilon. Accumulation remains float64; MCU arithmetic is not emulated.",
            },
            "assumptions_and_limits": [
                "Fixed base, fixed configuration, continuous same-frame angle/rate; pitch gravity is fixed-plane.",
                "Torque must be time-aligned joint-side synthetic truth or calibrated feedback, not a command proxy.",
                "Metadata declares calibration and provenance; this tool cannot independently verify those declarations.",
                "Non-overlapping trapezoidal windows discard faults, disabled/saturated spans, gaps and segment boundaries.",
                "Adjacent windows share endpoint measurements: endpoint noise can correlate their residuals even though integration intervals do not overlap.",
                "Acceptance measures window-mean torque residuals, not instantaneous residuals or closed-loop tracking.",
                "Full-rank normalized regression is a finite-record excitation check, not a persistent-excitation proof.",
                "Integral least squares does not remove errors-in-variables or closed-loop noise bias.",
                "Different hashes do not prove statistical independence; sessions and validation policy must be independent.",
                "This V1 does not identify moving-base dynamics, folding transitions, axis coupling, or actuator delays.",
                "No confidence intervals, stability guarantee, or automatic SMC gain tuning are provided.",
                "Candidate physical parameters require independent control simulation and staged hardware validation.",
            ],
        }
        outputs = {args.output: json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"}
        if args.header is not None:
            outputs[args.header] = _header(exported, report)
        _write_outputs(outputs)
        print("candidate_only: %s; validation window-mean torque RMSE %.6g Nm" %
              (args.output, validation_metrics["mean_torque_rmse_nm"]))
        return 0
    except (IdentificationError, OSError, np.linalg.LinAlgError) as error:
        print("identification failed: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
