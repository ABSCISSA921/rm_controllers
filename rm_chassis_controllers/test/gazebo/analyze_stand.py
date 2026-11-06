#!/usr/bin/env python3
"""Offline diagnosis of a completed startup attempt; never changes the plant."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[4]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--console", type=Path, required=True)
    args = parser.parse_args()
    directory = args.run.resolve()
    manifest = json.loads((directory / "manifest.json").read_text())
    a = np.genfromtxt(directory / "truth/physics.csv", delimiter=",", names=True)
    time = a["sim_sec"] + a["sim_nsec"] * 1e-9
    rotation = Rotation.from_quat(np.column_stack([a["base_" + key] for key in ("qx", "qy", "qz", "qw")]))
    rpy = rotation.as_euler("xyz")
    base = np.column_stack([a["base_" + key] for key in ("x", "y", "z")])
    speed = np.hypot(a["base_vx"], a["base_vy"])
    robot = ET.fromstring((directory / "expanded_series_legged2.urdf").read_text())
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    probe_path = WORKSPACE / "mujoco_platform/model/_build/vmc_probe.so"
    probe = ctypes.CDLL(str(probe_path))
    probe.evaluate_vmc.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_double)]
    errors, lengths, actual_lengths, serial_errors = [], [], [], []
    for side in ("left", "right"):
        hip, knee, wheel = (joints[side + "_" + part + "_joint"] for part in ("hip", "knee", "wheel"))
        if knee.find("parent").attrib["link"] != side + "_thigh":
            raise ValueError("This report is specifically for the actual serial Gazebo plant")
        hip_origin = np.fromstring(hip.find("origin").attrib["xyz"], sep=" ")
        hip_offset = np.fromstring(hip.find("origin").attrib["rpy"], sep=" ")[1]
        knee_offset = np.fromstring(knee.find("origin").attrib["rpy"], sep=" ")[1]
        first = -np.fromstring(knee.find("origin").attrib["xyz"], sep=" ")[2]
        second = -np.fromstring(wheel.find("origin").attrib["xyz"], sep=" ")[2]
        qh, qk = a[side + "_hip_joint_q"], a[side + "_knee_joint_q"]
        first_angle = qh + hip_offset
        second_angle = first_angle + knee_offset + qk
        serial = np.column_stack([-first * np.sin(first_angle) - second * np.sin(second_angle),
                                   -first * np.cos(first_angle) - second * np.cos(second_angle)])
        wheel_world = np.column_stack([a[side + "_wheel_" + key] for key in ("x", "y", "z")])
        physical = (rotation.inv().apply(wheel_world - base) - hip_origin)[:, [0, 2]]
        virtual = np.empty_like(physical)
        length = np.empty(len(time))
        out = (ctypes.c_double * 6)()
        for index, (h, k) in enumerate(zip(qh, qk)):
            probe.evaluate_vmc(h, k, out)
            length[index] = out[0]
            virtual[index] = [-out[0] * np.sin(out[1]), -out[0] * np.cos(out[1])]
        errors.append(np.linalg.norm(virtual - physical, axis=1))
        serial_errors.append(np.linalg.norm(serial - physical, axis=1))
        lengths.append(length)
        actual_lengths.append(np.linalg.norm(physical, axis=1))

    transitions = []
    standing_messages = []
    for line in args.console.read_text(errors="replace").splitlines():
        match = re.search(r", ([0-9.]+)\]: \[balance\] (.+)", line)
        if match:
            stamp, text = float(match.group(1)), match.group(2)
            if text.startswith("Enter "):
                transitions.append({"time": stamp, "mode": text.split()[1]})
            if "standing condition reached" in text:
                standing_messages.append(stamp)
    normal_durations = [right["time"] - left["time"] for left, right in zip(transitions, transitions[1:])
                        if left["mode"] == "NORMAL"]
    # A diagnostic common window, not a retrospectively claimed acceptance gate.
    selected = (time >= 5.) & (time <= 17.5)
    geometry = {}
    for i, side in enumerate(("left", "right")):
        geometry[side] = {
            "serial_fk_vs_truth_max_m": float(np.max(serial_errors[i])),
            "vmc_vs_truth_rms_m": float(np.sqrt(np.mean(errors[i][selected] ** 2))),
            "vmc_vs_truth_max_m": float(np.max(errors[i][selected])),
        }
    examples = []
    for target in ([standing_messages[0]] if standing_messages else []) + [6., 10., 17.]:
        i = int(np.argmin(abs(time - target)))
        examples.append({"physical_time": float(time[i]), "requested_report_time": target,
                         "base_height_m": float(base[i, 2]), "pitch_rad": float(rpy[i, 1]),
                         "left_vmc_length_m": float(lengths[0][i]),
                         "left_physical_hip_wheel_distance_m": float(actual_lengths[0][i]),
                         "left_point_disagreement_m": float(errors[0][i])})
    vmc_source = WORKSPACE / "src/rm_controllers/rm_chassis_controllers/src/bipedal_wheel_controller/vmc/VMC.cpp"
    baseline = subprocess.check_output(["git", "-C", str(WORKSPACE / "src/rm_controllers"), "show",
                                        "8e0265470a3cdef8730ff31b0628f95c2c2a971c:rm_chassis_controllers/src/bipedal_wheel_controller/vmc/VMC.cpp"])
    result = {
        "run": str(directory), "runtime_libraries": manifest.get("runtime_controller_libraries", []),
        "physics_samples": len(time), "physics_time_start": float(time[0]), "physics_time_end": float(time[-1]),
        "diagnostic_window_s": [5., 17.5], "mode_transitions_from_console_source_time": transitions,
        "max_normal_interval_s": max(normal_durations) if normal_durations else None,
        "normal_interval_at_least_5s": bool(normal_durations and max(normal_durations) >= 5.),
        "base_height_range_m": [float(base[selected, 2].min()), float(base[selected, 2].max())],
        "pitch_range_deg": np.rad2deg([rpy[selected, 1].min(), rpy[selected, 1].max()]).tolist(),
        "roll_peak_deg": float(np.rad2deg(abs(rpy[selected, 0])).max()),
        "horizontal_speed_rms_mps": float(np.sqrt(np.mean(speed[selected] ** 2))),
        "base_horizontal_excursion_m": float(np.linalg.norm(base[selected, :2] - base[selected, :2][0], axis=1).max()),
        "geometry": geometry, "examples": examples,
        "probe_sha256": hashlib.sha256(probe_path.read_bytes()).hexdigest(),
        "production_vmc_unchanged_from_A_head": baseline == vmc_source.read_bytes(),
        "force_diagnostic_abs_peak": {name: float(np.max(abs(a[name]))) for name in a.dtype.names if "GetForce" in name},
        "phase_note": "Geometry uses only one WorldUpdateEnd row; message/console source times shown separately. Force API is diagnostic only.",
    }
    (directory / "diagnosis.json").write_text(json.dumps(result, indent=2) + "\n")
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(time, base[:, 2], label="base height")
    axes[0].set_ylabel("height [m]")
    axes[1].plot(time, np.rad2deg(rpy[:, 1]), label="pitch")
    axes[1].plot(time, np.rad2deg(rpy[:, 0]), label="roll")
    axes[1].set_ylabel("angle [deg]")
    axes[2].plot(time, speed, label="base horizontal speed")
    axes[2].set_ylabel("speed [m/s]")
    axes[3].plot(time, errors[0], label="left VMC vs physical endpoint")
    axes[3].plot(time, serial_errors[0], label="left serial FK vs physics")
    axes[3].set_ylabel("point error [m]")
    axes[3].set_xlabel("physics time [s]")
    for ax in axes:
        ax.grid(True, alpha=.3)
        ax.legend(loc="upper right")
        for transition in transitions:
            if transition["mode"] == "PROTECT":
                ax.axvline(transition["time"], color="red", alpha=.15)
    fig.tight_layout()
    fig.savefig(directory / "startup_diagnosis.png", dpi=150)
    plt.close(fig)
    print(json.dumps({key: result[key] for key in ("max_normal_interval_s", "base_height_range_m", "pitch_range_deg",
                                                   "horizontal_speed_rms_mps", "base_horizontal_excursion_m", "geometry")}, indent=2))


if __name__ == "__main__":
    main()
