#!/usr/bin/env python3
"""Offline, standard-library-only inertia extraction from two pinned RM models.

This is a deliberately bounded SDF/URDF-xacro adapter, not a general xacro engine.
Only arithmetic constants and a small, explicit macro recipe are supported.
Run without arguments to regenerate profiles.csv/details.json; --check is read-only.
"""
import argparse
import ast
import copy
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / "sim" / "open_models"
XACRO = "{http://www.ros.org/wiki/xacro}"
IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
ZERO = [[0.0] * 3 for _ in range(3)]
GRAVITY = [0.0, 0.0, -9.81]


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def scale(a, s):
    return [x * s for x in a]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def transpose(a):
    return [list(row) for row in zip(*a)]


def matmul(a, b):
    return [[dot(row, column) for column in transpose(b)] for row in a]


def matvec(a, v):
    return [dot(row, v) for row in a]


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def compose(a, b):
    ra, pa = a
    rb, pb = b
    return matmul(ra, rb), add(pa, matvec(ra, pb))


def numeric(expression, env):
    """Reject calls, subscripts, attributes, powers, and unknown identifiers."""
    def walk(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id in env:
            return float(env[node.id])
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            return walk(node.operand) * (-1 if isinstance(node.op, ast.USub) else 1)
        if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            left, right = walk(node.left), walk(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            return left / right
        raise ValueError("Unsupported constant expression: " + expression)
    value = walk(ast.parse(expression, mode="eval").body)
    if not math.isfinite(value):
        raise ValueError("Non-finite source constant")
    return value


def expand(value, env):
    def replace_arg(match):
        key = "arg:" + match.group(1)
        if key not in env:
            raise ValueError("Unknown xacro argument: " + key)
        return str(env[key])
    def replace_expression(match):
        expression = match.group(1).strip()
        if expression in env:
            return str(env[expression])
        return str(numeric(expression, env))
    value = re.sub(r"\$\(arg\s+([A-Za-z0-9_]+)\)", replace_arg, value)
    value = re.sub(r"\$\{([^{}]+)\}", replace_expression, value)
    if "$" in value:
        raise ValueError("Unsupported unresolved expression: " + value)
    return value


def vector(value, env=None, length=3):
    values = [numeric(part, {}) for part in expand(value, env or {}).split()]
    if len(values) != length:
        raise ValueError("Invalid vector: " + value)
    return values


def origin(element):
    if element is None:
        return copy.deepcopy(IDENTITY), [0.0] * 3
    return rpy_matrix(vector(element.get("rpy", "0 0 0"))), vector(element.get("xyz", "0 0 0"))


def tensor(values):
    return [[values["ixx"], values["ixy"], values["ixz"]],
            [values["ixy"], values["iyy"], values["iyz"]],
            [values["ixz"], values["iyz"], values["izz"]]]


def eigenvalues(matrix):
    """Symmetric 3x3 Jacobi eigensolver, used only to audit physical tensors."""
    a = copy.deepcopy(matrix)
    for _ in range(32):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(a[ij[0]][ij[1]]))
        if abs(a[p][q]) < 1e-20:
            break
        angle = 0.5 * math.atan2(2 * a[p][q], a[q][q] - a[p][p])
        c, s = math.cos(angle), math.sin(angle)
        rotation = copy.deepcopy(IDENTITY)
        rotation[p][p], rotation[q][q] = c, c
        rotation[p][q], rotation[q][p] = s, -s
        a = matmul(transpose(rotation), matmul(a, rotation))
    return sorted(a[i][i] for i in range(3))


class Sources:
    def __init__(self, root):
        self.root = root
        self.manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        self.metadata = {}
        for record in self.manifest["sources"]:
            path = root / record["local_path"]
            data = path.read_bytes()
            if hashlib.sha256(data).hexdigest() != record["sha256"]:
                raise ValueError("SHA256 mismatch: " + str(path))
            git_sha = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
            if git_sha != record["git_blob_sha"]:
                raise ValueError("Git blob mismatch: " + str(path))
            self.metadata[record["local_path"].removeprefix("sources/")] = record

    def read(self, path):
        if path not in self.metadata:
            raise ValueError("Unpinned source: " + path)
        return ET.parse(self.root / "sources" / path).getroot()

    def location(self, path, marker):
        lines = (self.root / "sources" / path).read_text(encoding="utf-8").splitlines()
        matches = [i + 1 for i, line in enumerate(lines) if marker in line]
        if len(matches) != 1:
            raise ValueError("Ambiguous source marker: " + marker)
        metadata = self.metadata[path]
        return {"path": path, "line": matches[0], "url": metadata["url"] + "#L" + str(matches[0])}


class Model:
    def __init__(self, name):
        self.name = name
        self.links = {}
        self.joints = {}
        self.corrections = []

    def add_link(self, name, mass, inertia_origin, inertia, location):
        if name in self.links:
            raise ValueError("Duplicate link: " + name)
        self.links[name] = {"mass": mass, "inertia_origin": inertia_origin,
                            "inertia": inertia, "source": location}

    def add_joint(self, name, parent, child, transform, axis, kind, limits, location):
        if child in (j["child"] for j in self.joints.values()):
            raise ValueError("Multiple parents: " + child)
        self.joints[name] = {"parent": parent, "child": child, "transform": transform,
                             "axis": axis, "type": kind, "limits": limits, "source": location}

    def validate(self):
        for name, body in self.links.items():
            eig = eigenvalues(body["inertia"])
            body["source_principal_inertia"] = eig
            if body["mass"] < 0:
                raise ValueError("Negative mass: " + name)
            if min(eig) < -1e-15 or eig[2] > eig[0] + eig[1] + 1e-12:
                expected = [[2.129e-9, 7.329e-9, -1.11e-9],
                            [7.329e-9, 2.198e-9, -8.04e-9],
                            [-1.11e-9, -8.04e-9, 2.197e-9]]
                if (self.name == "dynamicx_standard3" and name == "camera_optical_frame" and
                        body["mass"] == 0.001 and body["inertia"] == expected):
                    self.corrections.append({"link": name, "source": body["source"],
                        "reason": "Virtual optical frame tensor is non-PSD; retain its 1 g mass and full transform, approximate it as a point mass.",
                        "original_inertia": body["inertia"], "original_principal_inertia": eig,
                        "used_inertia": copy.deepcopy(ZERO),
                        "max_axis_inertia_change_bound_kg_m2": max(abs(v) for v in eig)})
                    body["inertia"] = copy.deepcopy(ZERO)
                else:
                    raise ValueError("Unapproved nonphysical inertia tensor: " + name + " " + repr(eig))

    def transforms(self):
        result = {"base_link": (copy.deepcopy(IDENTITY), [0.0] * 3),
                  "chassis": (copy.deepcopy(IDENTITY), [0.0] * 3)}
        pending = list(self.joints.values())
        while pending:
            ready = [j for j in pending if j["parent"] in result]
            if not ready:
                raise ValueError("Disconnected or cyclic model tree")
            for joint in ready:
                result[joint["child"]] = compose(result[joint["parent"]], joint["transform"])
                pending.remove(joint)
        return result

    def subtree(self, start):
        names = [start]
        for name in names:
            names.extend(j["child"] for j in self.joints.values() if j["parent"] == name)
        return names

    def profile(self, axis_name, joint_name):
        joint = self.joints[joint_name]
        transforms = self.transforms()
        r_joint, p_joint = transforms[joint["child"]]
        axis = matvec(r_joint, joint["axis"])
        length = math.sqrt(dot(axis, axis))
        if abs(length - 1.0) > 1e-10:
            raise ValueError("Joint axis is not unit length")
        contributions = []
        total, mass, first_moment = 0.0, 0.0, [0.0] * 3
        for name in self.subtree(joint["child"]):
            body = self.links[name]
            r_com, p_com = compose(transforms[name], body["inertia_origin"])
            r = sub(p_com, p_joint)
            rotated = matmul(r_com, matmul(body["inertia"], transpose(r_com)))
            center_part = dot(axis, matvec(rotated, axis))
            parallel_part = body["mass"] * (dot(r, r) - dot(axis, r) ** 2)
            total += center_part + parallel_part
            mass += body["mass"]
            first_moment = add(first_moment, scale(r, body["mass"]))
            contributions.append({"link": name, "source": body["source"],
                "mass_kg": body["mass"], "com_from_axis_origin_world_m": r,
                "com_orientation_world": r_com, "inertia_about_com_world": rotated,
                "center_projection_kg_m2": center_part,
                "parallel_axis_kg_m2": parallel_part,
                "axis_inertia_kg_m2": center_part + parallel_part})
        perpendicular = sub(first_moment, scale(axis, dot(axis, first_moment)))
        required_sin = -dot(axis, cross(cross(axis, first_moment), GRAVITY))
        required_cos = -dot(axis, cross(perpendicular, GRAVITY))
        # Eliminate negative zero in the generated CSV, not meaningful small terms.
        required_sin = 0.0 if abs(required_sin) < 1e-15 else required_sin
        required_cos = 0.0 if abs(required_cos) < 1e-15 else required_cos
        limits = [-0.5, 0.5] if axis_name == "yaw" else joint["limits"]
        profile = {"model": self.name, "axis": axis_name, "inertia_kg_m2": total,
            "gravity_sin_nm": required_sin, "gravity_cos_nm": required_cos,
            "min_rad": limits[0], "max_rad": limits[1]}
        details = {"joint": joint_name, "joint_source": joint["source"],
            "world_axis": axis, "world_joint_origin_m": p_joint,
            "source_joint_limits_rad": joint["limits"],
            "validation_limits_rad": limits,
            "validation_interval_note": "Yaw uses chosen +/-0.5 rad test interval; not a claim of source limits." if axis_name == "yaw" else "Pitch limits copied from source model.",
            "frozen_pose": "All joint positions and velocities zero; base fixed; descendant actuated joints locked.",
            "included_subtree_mass_kg": mass, "subtree_com_from_axis_world_m": scale(first_moment, 1.0 / mass),
            "actual_gravity_drive_sin_nm": -required_sin,
            "actual_gravity_drive_cos_nm": -required_cos,
            "required_hold_torque_sin_nm": required_sin,
            "required_hold_torque_cos_nm": required_cos,
            "contributions": contributions, "profile": profile}
        return profile, details


def urdf_model(sources):
    model = Model("dynamicx_standard3")
    model.add_link("base_link", 0.0, (IDENTITY, [0.0] * 3), ZERO, {"note": "Fixed base; inertia outside selected moving subtree."})
    prefix = "rm_description/"
    recipe_path = prefix + "urdf/standard3/standard3.urdf.xacro"
    recipe = sources.read(recipe_path)
    env = {"pi": math.pi}
    for node in recipe.findall(XACRO + "arg"):
        env["arg:" + node.get("name")] = node.get("default")
    for setting in ("load_chassis", "load_gimbal", "load_shooter"):
        if env["arg:" + setting] != "true":
            raise ValueError("Recipe requires all model parts enabled")

    def collect(root, environment, path, source_marker=None, instantiation=None):
        for node in root:
            if node.tag not in ("link", "joint"):
                continue
            expanded = copy.deepcopy(node)
            for child in expanded.iter():
                child.attrib.update({key: expand(value, environment) for key, value in child.attrib.items()})
            name = expanded.get("name")
            location = sources.location(path, source_marker or ('<' + node.tag + ' name="' + node.get("name") + '"'))
            if instantiation is not None:
                location["instantiation"] = instantiation
            if expanded.tag == "link":
                inertial = expanded.find("inertial")
                if inertial is None:
                    model.add_link(name, 0.0, (IDENTITY, [0.0] * 3), ZERO, location)
                else:
                    mass = float(inertial.find("mass").get("value"))
                    inertia = tensor({key: float(value) for key, value in inertial.find("inertia").attrib.items()})
                    model.add_link(name, mass, origin(inertial.find("origin")), inertia, location)
            else:
                limit = expanded.find("limit")
                limits = [float(limit.get("lower")), float(limit.get("upper"))] if limit is not None else None
                axis = expanded.find("axis")
                model.add_joint(name, expanded.find("parent").get("link"), expanded.find("child").get("link"),
                    origin(expanded.find("origin")), vector(axis.get("xyz")) if axis is not None else [0.0, 0.0, 1.0],
                    expanded.get("type"), limits, location)

    roots = {}
    for short in ("urdf/standard/gimbal.urdf.xacro", "urdf/standard/shooter.urdf.xacro"):
        path = prefix + short
        roots[short] = sources.read(path)
        for node in roots[short].findall(XACRO + "property"):
            env[node.get("name")] = numeric(node.get("value"), env)
        collect(roots[short], env, path)

    macros = [("friction_wheel", "urdf/common/friction.urdf.xacro", roots["urdf/standard/shooter.urdf.xacro"]),
              ("camera_sensor", "urdf/common/camera.urdf.xacro", recipe),
              ("camera_optical_frame", "urdf/common/camera.urdf.xacro", recipe),
              ("IMU", "urdf/common/imu.urdf.xacro", recipe)]
    for name, short_path, call_tree in macros:
        path = prefix + short_path
        templates = [n for n in sources.read(path).findall(XACRO + "macro") if n.get("name") == name]
        if len(templates) != 1:
            raise ValueError("Missing macro template: " + name)
        calls = list(call_tree.iter(XACRO + name))
        expected_count = 2 if name == "friction_wheel" else 1
        if len(calls) != expected_count:
            raise ValueError("Unexpected macro count: " + name)
        for call in calls:
            local_env = dict(env)
            local_env.update({key: expand(value, env) for key, value in call.attrib.items()})
            # This pinned upstream macro uses global wheel_offset_z rather than
            # its wheel_z_offset parameter; the standard recipe defines both equally.
            if name == "friction_wheel" and float(local_env["wheel_z_offset"]) != float(env["wheel_offset_z"]):
                raise ValueError("Upstream wheel z alias no longer agrees")
            call_path = prefix + "urdf/standard/shooter.urdf.xacro" if name == "friction_wheel" else recipe_path
            call_marker = '<xacro:' + name + ' '
            if name == "friction_wheel":
                call_marker += 'prefixs="' + call.get("prefixs") + '"'
            collect(templates[0], local_env, path, 'name="' + name + '" params=',
                    sources.location(call_path, call_marker))
    expected_subtree = {"yaw", "supply_frame", "pitch", "left_friction_wheel", "right_friction_wheel",
                        "camera_link", "camera_optical_frame", "gimbal_imu"}
    if set(model.subtree("yaw")) != expected_subtree:
        raise ValueError("Pinned URDF gimbal subtree differs from reviewed recipe")
    model.validate()
    return model, [("yaw", "yaw_joint"), ("pitch", "pitch_joint")]


def sdf_model(sources):
    model = Model("rmoss_rmua19")
    path = "rmoss_gz_resources/resource/models/rmua19_standard_robot/model.sdf"
    root = sources.read(path).find("model")
    link_nodes = {n.get("name"): n for n in root.findall("link")}
    joint_nodes = {n.get("name"): n for n in root.findall("joint")}
    selected_links = {"chassis", "gimbal_yaw", "gimbal_pitch", "speed_monitor"}
    # Validate the entire moving subtree, so future unexpected payloads fail
    # visibly instead of silently being omitted.
    discovered = {"gimbal_yaw"}
    while True:
        new = {j.findtext("child") for j in joint_nodes.values() if j.findtext("parent") in discovered}
        if new <= discovered:
            break
        discovered |= new
    if discovered != selected_links - {"chassis"}:
        raise ValueError("SDF gimbal subtree changed: " + repr(discovered))
    for name in sorted(selected_links):
        node = link_nodes[name]
        if name == "chassis":
            model.add_link(name, 0.0, (IDENTITY, [0.0] * 3), ZERO, {"note": "Fixed chassis; excluded from moving subtree."})
            continue
        link_pose = node.find("pose")
        if vector(link_pose.text, length=6) != [0.0] * 6:
            raise ValueError("Adapter only accepts child links coincident with joint frames")
        inertia = node.find("inertial")
        pose = vector(inertia.findtext("pose", "0 0 0 0 0 0"), length=6)
        model.add_link(name, float(inertia.findtext("mass")), (rpy_matrix(pose[3:]), pose[:3]),
            tensor({child.tag: float(child.text) for child in inertia.find("inertia")}),
            sources.location(path, '<link name="' + name + '"'))
    for name, node in joint_nodes.items():
        if node.findtext("child") not in selected_links - {"chassis"}:
            continue
        pose_node = node.find("pose")
        parent, child = node.findtext("parent"), node.findtext("child")
        if pose_node.get("relative_to") != parent:
            raise ValueError("Unsupported SDF joint pose reference")
        if link_nodes[child].find("pose").get("relative_to") != name:
            raise ValueError("Unsupported SDF child pose reference")
        pose = vector(pose_node.text, length=6)
        axis_node = node.find("axis/xyz")
        if axis_node is not None and axis_node.attrib:
            raise ValueError("Unsupported SDF expressed_in axis")
        limit = node.find("axis/limit")
        limits = [float(limit.findtext("lower")), float(limit.findtext("upper"))] if limit is not None else None
        model.add_joint(name, parent, child, (rpy_matrix(pose[3:]), pose[:3]),
            vector(axis_node.text) if axis_node is not None else [0.0, 0.0, 1.0],
            node.get("type"), limits, sources.location(path, '<joint name="' + name + '"'))
    model.validate()
    return model, [("yaw", "gimbal_yaw_joint"), ("pitch", "gimbal_pitch_joint")]


def derive(root):
    sources = Sources(root)
    profiles, model_details = [], []
    for builder in (sdf_model, urdf_model):
        model, joints = builder(sources)
        details = {"model": model.name, "corrections": model.corrections, "axes": []}
        for axis, joint in joints:
            profile, axis_details = model.profile(axis, joint)
            profiles.append(profile)
            details["axes"].append(axis_details)
        model_details.append(details)
    details = {"schema_version": 1, "gravity_world_m_s2": GRAVITY,
        "dynamics_convention": "J*qdd = command_torque - B*qdot - hold_gravity_torque(q) + disturbance",
        "hold_gravity_torque": "gravity_sin_nm*sin(q) + gravity_cos_nm*cos(q)",
        "inertia_scope": "Locked-subtree joint inertia at q=0, base fixed; excludes unmodeled rotor reflection and moving-joint coupling.",
        "classification": {"rmoss_rmua19": "Maintainer-built primitive-shape SDF using official robot drawings; not measured inertia.",
                           "dynamicx_standard3": "Author-published URDF model constants; measurement/CAD provenance not specified in the pinned sources; one explicit virtual-frame tensor correction."},
        "models": model_details}
    return profiles, details


def csv_text(profiles):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=["model", "axis", "inertia_kg_m2", "gravity_sin_nm",
                                              "gravity_cos_nm", "min_rad", "max_rad"], lineterminator="\n")
    writer.writeheader()
    for profile in profiles:
        writer.writerow({k: format(v, ".12g") if isinstance(v, float) else v for k, v in profile.items()})
    return stream.getvalue()


def rounded(value):
    if isinstance(value, float):
        return float(format(value, ".12g"))
    if isinstance(value, list):
        return [rounded(v) for v in value]
    if isinstance(value, dict):
        return {k: rounded(v) for k, v in value.items()}
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify source hashes and recomputed profiles/details without writing")
    parser.add_argument("--root", type=Path, default=ROOT, help="Directory containing manifest.json and pinned sources")
    args = parser.parse_args()
    profiles, details = derive(args.root)
    outputs = {"profiles.csv": csv_text(profiles), "details.json": json.dumps(rounded(details), ensure_ascii=False, indent=2) + "\n"}
    if args.check:
        for filename, generated in outputs.items():
            actual = (args.root / filename).read_text(encoding="utf-8")
            if filename.endswith(".json"):
                if json.loads(actual) != json.loads(generated):
                    raise ValueError("Recomputed details differ: " + filename)
            elif actual != generated:
                raise ValueError("Recomputed profiles differ: " + filename)
        print("PASS: 11 pinned sources, 2 models, 4 joint profiles; offline derivation matches committed outputs")
    else:
        for filename, generated in outputs.items():
            (args.root / filename).write_text(generated, encoding="utf-8")
        print(outputs["profiles.csv"], end="")
        print("Wrote profiles.csv and details.json under", args.root)


if __name__ == "__main__":
    main()
