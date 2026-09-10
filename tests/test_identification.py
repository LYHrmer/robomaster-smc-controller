#!/usr/bin/env python3
"""Black-box acceptance tests for offline, candidate-only identification.

Synthetic inverse-dynamics recovery is not a test of unbiased closed-loop LS.
The fixture deliberately separates plant truth from the identification tool.
"""

from __future__ import annotations

from dataclasses import asdict, replace
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import identification_fixture as fixture


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "identify_gimbal.py"


class IdentificationAcceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.workspace = tempfile.TemporaryDirectory(prefix="smc-identification-tests-")
        cls.base = Path(cls.workspace.name)
        cls.paths = fixture.generate(cls.base / "fixture")
        spec = importlib.util.spec_from_file_location("smc_identification_under_test", TOOL)
        cls.identify = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls.identify
        spec.loader.exec_module(cls.identify)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.workspace.cleanup()

    def setUp(self) -> None:
        self.case = Path(tempfile.mkdtemp(prefix="case-", dir=self.base))
        self.report = self.case / "report.json"
        self.header = self.case / "identified.h"

    def invoke(self, axis="pitch", *, train=None, validation=None, metadata=None,
               threshold="0.001", output=None, header=None,
               symbol_prefix=None) -> subprocess.CompletedProcess:
        paths = self.paths[axis]
        command = [sys.executable, str(TOOL),
                   "--train", str(train or paths["train"]),
                   "--validation", str(validation or paths["validation"]),
                   "--metadata", str(metadata or paths["metadata"]),
                   "--output", str(output or self.report),
                   "--header", str(header or self.header),
                   "--window-s", "0.05", "--max-gap-s", "0.02",
                   "--max-validation-rmse-nm", threshold]
        if symbol_prefix is not None:
            command.extend(["--symbol-prefix", symbol_prefix])
        environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        return subprocess.run(command, capture_output=True, text=True,
                              env=environment, timeout=60, check=False)

    def assert_success(self, process) -> dict:
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        self.assertTrue(self.header.is_file())
        result = json.loads(self.report.read_text())
        self.assertEqual(result["schema_version"], 1)
        self.assertEqual(result["status"], "candidate_only")
        return result

    def assert_rejected(self, process) -> None:
        self.assertNotEqual(process.returncode, 0, process.stdout + process.stderr)
        self.assertFalse(self.header.exists(), "Rejected data exported controller parameters")
        self.assertTrue((process.stderr + process.stdout).strip())

    def altered_csv(self, axis, split, change, *, name="altered.csv") -> Path:
        with self.paths[axis][split].open(newline="") as stream:
            data = list(csv.DictReader(stream))
        change(data)
        path = self.case / name
        fixture.write_csv(path, data)
        return path

    def altered_metadata(self, axis="pitch", **changes) -> Path:
        data = fixture.metadata(axis)
        data.update(changes)
        path = self.case / "metadata.json"
        path.write_text(json.dumps(data))
        return path

    def independent_mean_torque_rmse(self, axis: str, parameters: dict) -> float:
        """Dense midpoint quadrature of *analytic* held-out states.

        Does not reuse the estimator's CSV integration or window builder.
        Test windows include acceleration, reversals and several gravity phases.
        """
        estimated = fixture.Truth(**{key: parameters[key] for key in asdict(fixture.TRUTH[axis])})
        residuals = []
        for start in (0.173, 0.647, 1.259, 2.413, 3.031, 4.719, 6.283, 8.117, 11.371, 14.227):
            error = 0.0
            for sample in range(200):
                time_s = start + (sample + 0.5) * 0.05 / 200.0
                _, rate, acceleration, gravity = fixture.state(axis, "validation", time_s)
                error += (fixture.plant_torque(estimated, rate, acceleration, gravity)
                          - fixture.plant_torque(fixture.TRUTH[axis], rate, acceleration, gravity))
            residuals.append(error / 200.0)
        return math.sqrt(sum(value * value for value in residuals) / len(residuals))

    def test_recover_parameters_and_predict_independent_heldout_windows(self):
        for axis in ("yaw", "pitch"):
            with self.subTest(axis=axis):
                result = self.assert_success(self.invoke(axis))
                self.assertEqual(result["axis"], axis)
                values = result["parameters"]
                for key, truth in asdict(fixture.TRUTH[axis]).items():
                    self.assertAlmostEqual(values[key], truth,
                                           delta=max(abs(truth) * 0.006, 2e-5), msg=key)
                self.assertLess(self.independent_mean_torque_rmse(axis, values), 0.0001)
                for split in ("train", "validation"):
                    self.assertGreaterEqual(result["metrics"][split]["window_count"], 20)
                    self.assertLess(result["metrics"][split]["mean_torque_rmse_nm"], 0.0001)
                self.assertNotEqual(result["provenance"]["train_sha256"],
                                    result["provenance"]["validation_sha256"])
                self.assertLess(result["export_float32"]["metrics"]["validation"]
                                ["mean_torque_rmse_nm"], 0.0001)

    def test_generated_header_compiles_as_c99(self):
        compiler = shutil.which("cc")
        if compiler is None:
            self.skipTest("A C compiler is required to check the exported header")
        self.assert_success(self.invoke())
        source = self.case / "header_check.c"
        constants = ["smc_ident_" + name for name in asdict(fixture.TRUTH["pitch"])]
        source.write_text(
            '#include <math.h>\n#include "identified.h"\n'
            'int main(void) { const float p[] = {' + ",".join(constants) + '};\n'
            'for (unsigned i=0; i<sizeof(p)/sizeof(p[0]); ++i) '
            'if (!isfinite(p[i])) return 1;\nreturn p[0] > 0.0f ? 0 : 2; }\n')
        binary = self.case / "header_check"
        compiled = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                   str(source), "-lm", "-o", str(binary)],
                                  capture_output=True, text=True, check=False)
        self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
        self.assertEqual(subprocess.run([str(binary)], check=False).returncode, 0)

    def test_two_axis_headers_can_share_one_c_translation_unit(self):
        compiler = shutil.which("cc")
        if compiler is None:
            self.skipTest("A C compiler is required to check exported headers")
        # Case-different prefixes also catch include guards that accidentally
        # normalize distinct legal C identifiers into the same macro.
        prefixes = {"yaw": "Axis", "pitch": "axis"}
        for axis,prefix in prefixes.items():
            result = self.invoke(axis, symbol_prefix=prefix,
                                 output=self.case / (axis + ".json"),
                                 header=self.case / (axis + ".h"))
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(json.loads((self.case / (axis + ".json")).read_text())
                             ["symbol_prefix"], prefix)
        source = self.case / "two_axes.c"
        names = [prefix + "_" + name for prefix in prefixes.values()
                 for name in asdict(fixture.TRUTH["pitch"])]
        source.write_text('#include <math.h>\n#include "yaw.h"\n#include "pitch.h"\n'
                          'int main(void) { const float p[] = {' + ",".join(names) + '};\n'
                          'for (unsigned i=0; i<sizeof(p)/sizeof(p[0]); ++i) '
                          'if (!isfinite(p[i])) return 1;\n'
                          'return Axis_inertia_kg_m2 > axis_inertia_kg_m2 ? 0 : 2; }\n')
        binary = self.case / "two_axes"
        process = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                  str(source), "-lm", "-o", str(binary)],
                                 capture_output=True, text=True, check=False)
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        self.assertEqual(subprocess.run([str(binary)], check=False).returncode, 0)

    def test_invalid_c_symbol_prefix_is_rejected(self):
        for prefix in ("", "_private", "3axis", "pitch-yaw", "偏航", "bad name"):
            with self.subTest(prefix=prefix):
                self.assert_rejected(self.invoke(symbol_prefix=prefix))

    def test_continuous_yaw_fixture_really_crosses_multiple_turns(self):
        samples = fixture.rows("yaw", "train")
        angles = [row["angle_rad"] for row in samples]
        self.assertGreater(max(angles) - min(angles), 4.0 * math.pi)
        self.assertGreater(max(angles), 6.0 * math.pi)
        self.assertLess(max(abs(b-a) for a,b in zip(angles, angles[1:])), 0.02)
        self.assert_success(self.invoke("yaw"))

    def test_wrong_pitch_gravity_frame_does_not_generalize(self):
        def replace_gravity(data):
            for row in data:
                row["gravity_angle_rad"] = row["angle_rad"]
        train = self.altered_csv("pitch", "train", replace_gravity, name="wrong_train.csv")
        validation = self.altered_csv("pitch", "validation", replace_gravity, name="wrong_validation.csv")
        loose_report = self.case / "wrong_frame_diagnostic.json"
        loose_header = self.case / "wrong_frame_diagnostic.h"
        diagnostic = self.invoke(train=train, validation=validation, threshold="1",
                                 output=loose_report, header=loose_header)
        self.assertEqual(diagnostic.returncode, 0, diagnostic.stdout + diagnostic.stderr)
        metrics = json.loads(loose_report.read_text())["metrics"]
        self.assertLess(metrics["train"]["mean_torque_rmse_nm"], 0.0001)
        self.assertGreater(metrics["validation"]["mean_torque_rmse_nm"], 0.003)
        self.assert_rejected(self.invoke(train=train, validation=validation, threshold="0.003"))

    def test_limitation_common_torque_scale_is_not_identifiable_from_residuals(self):
        # Both recordings can consistently misreport the absolute torque scale.
        # A low residual then validates scaled coefficients, not physical Nm or J.
        scale = 1.2
        for axis in ("yaw", "pitch"):
            with self.subTest(axis=axis):
                def scale_torque(data):
                    for row in data:
                        row["torque_nm"] = scale * float(row["torque_nm"])
                paths = {
                    split: self.altered_csv(axis, split, scale_torque,
                                           name=axis + "_scaled_" + split + ".csv")
                    for split in ("train", "validation")
                }
                result = self.assert_success(self.invoke(axis, **paths))
                self.assertLess(result["metrics"]["validation"]["mean_torque_rmse_nm"], 0.0001)
                for name, truth in asdict(fixture.TRUTH[axis]).items():
                    expected = truth if name == "friction_velocity_rad_s" else scale * truth
                    self.assertAlmostEqual(result["parameters"][name], expected,
                                           delta=max(abs(expected) * 0.006, 2e-5), msg=name)
                self.assertGreater(abs(result["parameters"]["inertia_kg_m2"]
                                       - fixture.TRUTH[axis].inertia_kg_m2),
                                   0.15 * fixture.TRUTH[axis].inertia_kg_m2)

    def test_limitation_common_gravity_zero_offset_is_absorbed_by_coefficients(self):
        # A consistent angular zero error in both recordings rotates the sine /
        # cosine coefficients. It is unobservable from torque residuals alone;
        # this successful candidate must not be described as a zero calibration.
        offset = 0.37
        def shift_gravity(data):
            for row in data:
                row["gravity_angle_rad"] = float(row["gravity_angle_rad"]) + offset
        paths = {
            split: self.altered_csv("pitch", split, shift_gravity,
                                   name="shifted_gravity_" + split + ".csv")
            for split in ("train", "validation")
        }
        result = self.assert_success(self.invoke(**paths))
        parameters = result["parameters"]
        truth = fixture.TRUTH["pitch"]
        expected = asdict(truth)
        expected["gravity_sin_nm"] = (truth.gravity_sin_nm * math.cos(offset)
                                      + truth.gravity_cos_nm * math.sin(offset))
        expected["gravity_cos_nm"] = (-truth.gravity_sin_nm * math.sin(offset)
                                      + truth.gravity_cos_nm * math.cos(offset))
        for name, value in expected.items():
            self.assertAlmostEqual(parameters[name], value,
                                   delta=max(abs(value) * 0.006, 2e-5), msg=name)
        self.assertAlmostEqual(math.hypot(parameters["gravity_sin_nm"], parameters["gravity_cos_nm"]),
                               math.hypot(truth.gravity_sin_nm, truth.gravity_cos_nm), delta=0.0002)
        self.assertGreater(abs(parameters["gravity_sin_nm"] - truth.gravity_sin_nm), 0.01)
        self.assertLess(result["metrics"]["train"]["mean_torque_rmse_nm"], 0.0001)
        self.assertLess(result["metrics"]["validation"]["mean_torque_rmse_nm"], 0.0001)

    def test_constant_state_is_rank_deficient(self):
        def make_constant(data):
            for row in data:
                row.update(angle_rad="0", rate_rad_s="0", gravity_angle_rad="0.18", torque_nm="0.2")
        train = self.altered_csv("pitch", "train", make_constant)
        self.assert_rejected(self.invoke(train=train))

    def test_nonmonotonic_or_duplicate_timestamp_is_rejected(self):
        for time_s in ("0.001", "0.002"):
            with self.subTest(time_s=time_s):
                train = self.altered_csv("pitch", "train", lambda data: data[2].update(time_s=time_s))
                self.assert_rejected(self.invoke(train=train))

    def test_nonfinite_numeric_fields_are_rejected_even_on_invalid_rows(self):
        for column in fixture.FIELDS[:5]:
            for invalid in ("nan", "inf", "-inf"):
                with self.subTest(column=column, value=invalid):
                    def corrupt(data):
                        data[100][column] = invalid
                        data[100]["valid"] = "0"
                    train = self.altered_csv("pitch", "train", corrupt)
                    self.assert_rejected(self.invoke(train=train))

    def test_flags_and_segment_have_strict_types(self):
        changes = [(name,value) for name in ("enabled", "valid", "saturated")
                   for value in ("2", "-1", "true", "0.5")]
        changes += [("segment", "1.5"), ("segment", "nan")]
        for column,value in changes:
            with self.subTest(column=column, value=value):
                train = self.altered_csv("pitch", "train", lambda data: data[100].update({column:value}))
                self.assert_rejected(self.invoke(train=train))

    def test_required_column_is_not_silently_defaulted(self):
        path = self.case / "missing_torque.csv"
        text = self.paths["pitch"]["train"].read_text()
        path.write_text(text.replace("torque_nm", "unscaled_current", 1))
        self.assert_rejected(self.invoke(train=path))

    def test_unqualified_metadata_is_rejected(self):
        changes = (
            {"schema_version": 2}, {"torque_source": "uncalibrated_current"},
            {"torque_time_aligned": False}, {"base_motion": "moving"},
            {"gravity_frame": "encoder_without_base_pose"}, {"angle_continuous": False},
            {"friction_velocity_rad_s": 0.0}, {"friction_velocity_rad_s": -0.1},
        )
        for change in changes:
            with self.subTest(change=change):
                self.assert_rejected(self.invoke(metadata=self.altered_metadata(**change)))

    def test_same_content_under_different_filename_is_not_validation(self):
        copied = self.case / "pretend_validation.csv"
        shutil.copyfile(self.paths["pitch"]["train"], copied)
        self.assert_rejected(self.invoke(validation=copied))

    def test_negative_physical_parameters_are_rejected_not_projected(self):
        for name in ("inertia_kg_m2", "viscous_nm_s_rad", "coulomb_nm"):
            with self.subTest(parameter=name):
                impossible = replace(fixture.TRUTH["pitch"],
                                     **{name: -getattr(fixture.TRUTH["pitch"], name)})
                paths = {}
                for split in ("train", "validation"):
                    paths[split] = self.case / (split + ".csv")
                    fixture.write_csv(paths[split], fixture.rows("pitch", split, truth=impossible))
                self.assert_rejected(self.invoke(**paths))

    def test_validation_error_gate_blocks_header_export(self):
        def add_unmodeled_load(data):
            for row in data:
                row["torque_nm"] = float(row["torque_nm"]) + 0.03
        validation = self.altered_csv("pitch", "validation", add_unmodeled_load)
        self.assert_rejected(self.invoke(validation=validation, threshold="0.005"))

    def test_failed_run_preserves_previously_reviewed_artifacts(self):
        self.report.write_text("previous reviewed report\n")
        self.header.write_text("/* previous reviewed parameters */\n")
        result = self.invoke(threshold="1e-12")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.report.read_text(), "previous reviewed report\n")
        self.assertEqual(self.header.read_text(), "/* previous reviewed parameters */\n")

    def test_float64_fit_cannot_bypass_failed_float32_export_validation(self):
        # A numerical stress case, not a proposed gimbal load: the gravity
        # coefficient is finite in float32, but its rounding loses ~0.1 Nm.
        truth = replace(fixture.TRUTH["pitch"], gravity_cos_nm=12345678.9)
        paths, windows = {}, {}
        for split in ("train", "validation"):
            paths[split] = self.case / (split + ".csv")
            fixture.write_csv(paths[split], fixture.rows("pitch", split, truth=truth))
            windows[split] = self.identify.build_windows(
                self.identify.read_dataset(paths[split]), axis="pitch",
                friction_velocity_rad_s=truth.friction_velocity_rad_s,
                window_s=0.05, max_gap_s=0.02)
        fit = self.identify.fit(windows["train"])
        metrics = self.identify.evaluate(windows["validation"], fit.parameters)
        self.assertLess(metrics["mean_torque_rmse_nm"], 0.001,
                        "Stress case must pass float64 before testing export rejection")
        self.report.write_text("previous reviewed report\n")
        self.header.write_text("/* previous reviewed parameters */\n")
        result = self.invoke(**paths, threshold="0.001")
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("float32", result.stdout + result.stderr)
        self.assertEqual(self.report.read_text(), "previous reviewed report\n")
        self.assertEqual(self.header.read_text(), "/* previous reviewed parameters */\n")

    def test_insufficient_data_does_not_export_parameters(self):
        path = self.case / "too_short.csv"
        fixture.write_csv(path, fixture.rows("pitch", "train", duration_s=0.02))
        self.assert_rejected(self.invoke(train=path))

    def test_integral_windows_never_cross_invalid_samples_gaps_or_segments(self):
        samples = fixture.rows("pitch", "train")
        excluded_times = []
        kept = []
        for index, sample in enumerate(samples):
            sample["segment"] = int(index >= 1000) + int(index >= 6000)
            if 3000 < index < 3100:
                continue  # Deliberately lose 0.2 s of data; never interpolate it.
            for field, first in (("valid", 2000), ("enabled", 4000), ("saturated", 5000)):
                if first <= index <= first + 10:
                    sample[field] = 1 if field == "saturated" else 0
                    excluded_times.append(sample["time_s"])
            kept.append(sample)
        path = self.case / "discontinuous_recording.csv"
        fixture.write_csv(path, kept)
        dataset = self.identify.read_dataset(path)
        windows = self.identify.build_windows(
            dataset, axis="pitch",
            friction_velocity_rad_s=fixture.TRUTH["pitch"].friction_velocity_rad_s,
            window_s=0.05, max_gap_s=0.02)
        self.assertGreater(len(windows.start_times_s), 100)
        self.assertGreaterEqual(windows.discarded_sample_count, len(excluded_times))
        self.assertGreaterEqual(windows.segment_break_count, 2)
        self.assertGreaterEqual(windows.gap_break_count, 1)
        for start,end in zip(windows.start_times_s, windows.end_times_s):
            self.assertFalse(any(start <= bad <= end for bad in excluded_times), (start,end))
            self.assertFalse(start <= 6.0 and end >= 6.2, (start,end))
            for boundary in (2.0, 12.0):
                self.assertFalse(start < boundary <= end, (start,end,boundary))
        # The usable parts still describe the same plant: rejection and
        # segmentation must not fabricate bias by joining disconnected samples.
        result = self.assert_success(self.invoke(train=path))
        self.assertLess(result["metrics"]["validation"]["mean_torque_rmse_nm"], 0.0001)


if __name__ == "__main__":
    unittest.main()
