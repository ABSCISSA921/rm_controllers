#!/usr/bin/env python3
"""Prepare, then explicitly run, one isolated original-launch integration attempt."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from xmlrpc.client import ServerProxy


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[4]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def unused_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def command_output(command, environment):
    return subprocess.check_output(command, env=environment, text=True).strip()


def config_view(destination, frozen):
    """A per-run package view, with one chosen YAML at each original filename."""
    source = WORKSPACE / "src/rm_config"
    destination.mkdir(parents=True)
    for item in source.iterdir():
        if item.name == "config":
            continue
        if item.name != ".git":
            (destination / item.name).symlink_to(item, target_is_directory=item.is_dir())
    config = destination / "config"
    config.mkdir()
    replacements = {"rm_controllers": frozen / "controller.yaml", "rm_manual": frozen / "manual.yaml"}
    for item in (source / "config").iterdir():
        target = config / item.name
        if item.name not in replacements:
            target.symlink_to(item, target_is_directory=item.is_dir())
            continue
        target.mkdir()
        for entry in item.iterdir():
            selected = replacements[item.name] if entry.name == "series_legged2.yaml" else entry
            (target / entry.name).symlink_to(selected, target_is_directory=entry.is_dir())


def prepare(output, frozen, scenario):
    output.mkdir(parents=True, exist_ok=False)
    (output / "truth").mkdir()
    (output / "logs").mkdir()
    prefix = output / "prefix"
    (prefix / "lib").mkdir(parents=True)
    library = frozen / "lib/librm_chassis_controllers.so"
    for path in (library, frozen / "controller.yaml", frozen / "manual.yaml"):
        if not path.is_file():
            raise FileNotFoundError(path)
    (prefix / "lib/librm_chassis_controllers.so").symlink_to(library)
    config_view(output / "packages/rm_config", frozen)
    environment = os.environ.copy()
    environment.pop("DISPLAY", None)  # Never connect to the user's desktop.
    environment.pop("WAYLAND_DISPLAY", None)
    ros_port, gazebo_port = unused_port(), unused_port()
    while gazebo_port == ros_port:
        gazebo_port = unused_port()
    environment.update({
        "ROBOT_TYPE": "series_legged2", "ROS_MASTER_URI": "http://127.0.0.1:%d" % ros_port,
        "ROS_HOSTNAME": "localhost", "ROS_IP": "127.0.0.1", "GAZEBO_IP": "127.0.0.1",
        "GAZEBO_MASTER_URI": "http://127.0.0.1:%d" % gazebo_port,
        "QT_QPA_PLATFORM": "offscreen", "ROS_LOG_DIR": str(output / "logs"),
        "ROS_HOME": str(output / "ros_home"), "XDG_CONFIG_HOME": str(output / "qt_config"),
        "XDG_CACHE_HOME": str(output / "qt_cache"), "BASE_LINK_GAZEBO_TRUTH_DIR": str(output / "truth"),
        "BASE_LINK_GAZEBO_MODEL": "series_legged2",
    })
    for variable, addition in [("CMAKE_PREFIX_PATH", str(prefix)), ("LD_LIBRARY_PATH", str(prefix / "lib")),
                               ("ROS_PACKAGE_PATH", str(output / "packages"))]:
        environment[variable] = addition + ":" + environment.get(variable, "")
    paths = {name: command_output(["rospack", "find", name], environment)
             for name in ("rm_description", "rm_config", "rm_gazebo", "rm_chassis_controllers", "rqt_virtual_dbus")}
    if Path(paths["rm_description"]).resolve() != (WORKSPACE / "src/rm_description").resolve():
        raise RuntimeError("rm_description does not resolve to this workspace")
    if Path(paths["rm_config"]).resolve() != (output / "packages/rm_config").resolve():
        raise RuntimeError("Frozen rm_config view was not selected")
    for name, relative in {"rm_gazebo": "src/rm_control/rm_gazebo",
                           "rm_chassis_controllers": "src/rm_controllers/rm_chassis_controllers",
                           "rqt_virtual_dbus": "src/rqt_virtual_dbus"}.items():
        if Path(paths[name]).resolve() != (WORKSPACE / relative).resolve():
            raise RuntimeError(name + " does not resolve to this workspace")
    urdf_source = Path(paths["rm_description"]) / "urdf/series_legged2/series_legged2.urdf.xacro"
    expanded = command_output(["/opt/ros/noetic/bin/xacro", str(urdf_source), "use_simulation:=true",
                               "load_gimbal:=true", "load_shooter:=true", "load_arm:=true", "roller_type:=simple"],
                              environment)
    (output / "expanded_series_legged2.urdf").write_text(expanded + "\n")
    robot = ET.fromstring(expanded)
    mesh_hashes = {}
    for mesh in robot.iter("mesh"):
        filename = mesh.attrib["filename"]
        if filename.startswith("package://rm_description/"):
            source = Path(paths["rm_description"]) / filename[len("package://rm_description/"):]
            mesh_hashes[filename] = digest(source) if source.is_file() else None
    joints = {}
    for joint in robot.findall("joint"):
        joints[joint.attrib["name"]] = {"type": joint.attrib["type"],
                                        "parent": joint.find("parent").attrib["link"],
                                        "child": joint.find("child").attrib["link"]}
    sampler = WORKSPACE / "base_link_control_plan/gazebo_tools/build/libbase_link_gazebo_observer.so"
    selected_environment = {name: environment[name] for name in environment if name in {
        "ROBOT_TYPE", "ROS_MASTER_URI", "ROS_HOSTNAME", "ROS_IP", "GAZEBO_IP", "GAZEBO_MASTER_URI",
        "QT_QPA_PLATFORM", "ROS_LOG_DIR", "ROS_HOME", "XDG_CONFIG_HOME", "XDG_CACHE_HOME", "CMAKE_PREFIX_PATH",
        "LD_LIBRARY_PATH", "ROS_PACKAGE_PATH", "BASE_LINK_GAZEBO_TRUTH_DIR", "BASE_LINK_GAZEBO_MODEL"}}
    manifest = {
        "status": "prepared_not_run", "frozen_artifact": str(frozen), "scenario": str(scenario),
        "paths": paths, "environment": selected_environment, "ros_port": ros_port,
        "controller_library": str(library), "controller_library_sha256": digest(library),
        "controller_yaml_sha256": digest(frozen / "controller.yaml"),
        "manual_yaml_sha256": digest(frozen / "manual.yaml"),
        "expanded_urdf_sha256": digest(output / "expanded_series_legged2.urdf"),
        "urdf_source_sha256": digest(urdf_source), "joints": joints,
        "description_head": command_output(["git", "-C", paths["rm_description"], "rev-parse", "HEAD"], environment),
        "description_status": command_output(["git", "-C", paths["rm_description"], "status", "--short"], environment),
        "mesh_sha256": mesh_hashes,
        "sampler": str(sampler), "sampler_sha256": digest(sampler) if sampler.exists() else None,
        "mechanism": "series_legged2 serial; C series_legged five-bar model is incompatible",
        "launches": [str(Path(paths["rm_gazebo"]) / "launch/empty_world.launch"),
                     str(Path(paths["rm_config"]) / "launch/manual.launch")],
        "instrumentation": "Original launch loaded by roslaunch; gzserver -s observer appended and gzclient removed in memory; physics/robot/algorithm unchanged",
        "truth_phase": "WorldUpdateEnd; contact source time retained separately; no asynchronous join",
        "effort_semantics": "Gazebo Joint::GetForce diagnostic; not certified actual actuator torque",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def descendants(pid):
    result = [pid]
    children = Path("/proc/%d/task/%d/children" % (pid, pid))
    if children.exists():
        for child in children.read_text().split():
            result.extend(descendants(int(child)))
    return result


def audit_library(parent, output, manifest):
    matches = []
    for process in list(parent.pm.procs):
        if not getattr(process, "pid", None):
            continue
        for pid in descendants(process.pid):
            maps = Path("/proc/%d/maps" % pid)
            if not maps.exists():
                continue
            text = maps.read_text()
            libraries = {line.split()[-1] for line in text.splitlines() if "librm_chassis_controllers.so" in line}
            for library in libraries:
                actual = Path(library).resolve()
                record = {"pid": pid, "path": str(actual), "sha256": digest(actual)}
                matches.append(record)
                if record["sha256"] != manifest["controller_library_sha256"]:
                    raise RuntimeError("Runtime loaded a different chassis controller library: " + str(actual))
                (output / "gazebo_process_maps.txt").write_text(text)
    return matches


def run(output, manifest):
    os.environ.pop("DISPLAY", None)
    os.environ.pop("WAYLAND_DISPLAY", None)
    os.environ.update(manifest["environment"])
    import roslaunch

    class ObservedLaunch(roslaunch.parent.ROSLaunchParent):
        def _load_config(self):
            super()._load_config()
            self.config.nodes = [node for node in self.config.nodes
                                 if not (node.package == "gazebo_ros" and node.type == "gzclient")]
            for node in self.config.nodes:
                if node.package == "gazebo_ros" and node.type == "gzserver":
                    node.args += " -s " + manifest["sampler"]

    class ManualLaunch(roslaunch.parent.ROSLaunchParent):
        def _load_config(self):
            super()._load_config()
            normalized = []
            # These optional lists have only commented-out entries in the
            # frozen YAML. ROS XMLRPC cannot serialize YAML null; an empty
            # list preserves the documented no-calibration configuration.
            for name in ("/rm_manual/controllers_list/calibration_controllers", "/rm_manual/gimbal_calibration"):
                parameter = self.config.params.get(name)
                if parameter is not None and parameter.value is None:
                    parameter.value = []
                    normalized.append(name)
            manifest["empty_list_parameter_normalization"] = normalized

    run_id = str(uuid.uuid4())
    roslaunch.configure_logging(run_id)
    world = ObservedLaunch(run_id, [manifest["launches"][0]], port=manifest["ros_port"], local_only=True)
    manual = ManualLaunch(run_id, [manifest["launches"][1]], local_only=True)
    gui = None
    gui_log = (output / "logs/rqt_driver.log").open("w")
    manifest["status"] = "running"
    try:
        world.start()
        master = ServerProxy(os.environ["ROS_MASTER_URI"])
        deadline = time.monotonic() + 120.
        while time.monotonic() < deadline:
            code, _, _ = master.lookupService("/base_link_integration", "/controller_manager/list_controllers")
            if code == 1:
                break
            world.spin_once()
            time.sleep(.1)
        else:
            raise RuntimeError("Gazebo controller_manager was not ready within 120 wall seconds")
        manual.start()
        gui = subprocess.Popen(["/usr/bin/python3", str(HERE / "rqt_driver.py"), "--script", manifest["scenario"],
                                "--output", str(output)], stdout=gui_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 600.
        matched = []
        while gui.poll() is None and time.monotonic() < deadline:
            world.spin_once()
            manual.spin_once()
            if not matched:
                matched = audit_library(world, output, manifest)
            time.sleep(.1)
        if gui.poll() is None:
            raise RuntimeError("GUI/physics attempt exceeded 600 wall seconds")
        manifest["gui_exit_code"] = gui.returncode
        manifest["runtime_controller_libraries"] = matched
        manifest["status"] = "attempt_finished" if gui.returncode == 0 and matched else "integration_failed"
    except Exception as error:
        manifest["status"] = "integration_failed"
        manifest["error"] = str(error)
        raise
    finally:
        if gui is not None and gui.poll() is None:
            os.killpg(gui.pid, signal.SIGINT)
            try:
                gui.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(gui.pid, signal.SIGTERM)
        manual.shutdown()
        world.shutdown()
        gui_log.close()
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario", type=Path, default=HERE / "stand_attempt.json")
    parser.add_argument("--run", action="store_true", help="Explicitly start the prepared isolated ROS/Gazebo attempt")
    args = parser.parse_args()
    output, frozen, scenario = args.output.resolve(), args.artifact.resolve(), args.scenario.resolve()
    manifest = prepare(output, frozen, scenario)
    print(json.dumps({"status": manifest["status"], "output": str(output), "mechanism": manifest["mechanism"]}, indent=2))
    if args.run:
        run(output, manifest)


if __name__ == "__main__":
    main()
