#!/usr/bin/env python3
"""Prepare ICE Robocon in rl_sar from its saved robot_lab run and URDF.

Requires Python 3.10+, PyYAML, torch and mujoco. Use MuJoCo 3.2.7 and CPU
PyTorch 2.3.0 to match this project's deployment libraries.
"""

import argparse
import hashlib
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch
from torch import nn
import yaml

from convert_urdf import convert_urdf


class ConfigLoader(yaml.SafeLoader):
    """Read Isaac Lab's tuple/slice YAML tags without constructing Python objects."""


ConfigLoader.add_constructor(
    "tag:yaml.org,2002:python/tuple", lambda loader, node: loader.construct_sequence(node)
)
ConfigLoader.add_constructor(
    "tag:yaml.org,2002:python/object/apply:builtins.slice",
    lambda loader, node: loader.construct_sequence(node),
)


def read_config(path):
    return yaml.load(path.read_text(), Loader=ConfigLoader)


def resolve(value, name):
    if not isinstance(value, dict):
        return value
    matches = [v for pattern, v in value.items() if re.fullmatch(pattern, name)]
    if len(matches) != 1:
        raise ValueError(f"Expected one configuration match for {name}, found {len(matches)}")
    return matches[0]


def export_policy(checkpoint, agent, output, input_dim, output_dim):
    actor = agent["actor"]
    if (actor["class_name"] != "MLPModel" or actor["activation"] != "elu"
            or actor.get("obs_normalization") or actor.get("network_cfg")):
        raise ValueError("This importer supports an unnormalized ELU MLPModel only")
    distribution = actor.get("distribution_cfg") or {}
    if distribution.get("class_name") != "GaussianDistribution":
        raise ValueError("Expected GaussianDistribution with deterministic mean output")
    if agent.get("obs_groups", {}).get("actor") != ["policy"] or agent.get("clip_actions") is not None:
        raise ValueError("Expected one actor observation group and no wrapper action clipping")
    saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
    state = saved["actor_state_dict"]
    unexpected = set(state) - {"distribution.std_param"} - {
        f"mlp.{i}.{part}" for i in range(0, 2 * (len(actor["hidden_dims"]) + 1), 2)
        for part in ("weight", "bias")
    }
    if unexpected:
        raise ValueError(f"Unsupported actor state: {sorted(unexpected)}")
    dimensions = [input_dim, *actor["hidden_dims"], output_dim]
    layers = []
    for i, (n_in, n_out) in enumerate(zip(dimensions, dimensions[1:])):
        layers.append(nn.Linear(n_in, n_out))
        if i < len(dimensions) - 2:
            layers.append(nn.ELU())
    policy = nn.Sequential(*layers).eval()
    policy.load_state_dict({k.removeprefix("mlp."): v for k, v in state.items() if k.startswith("mlp.")})
    scripted = torch.jit.script(policy)
    # Check the export against a direct evaluation of the checkpoint weights.
    generator = torch.Generator().manual_seed(42)
    samples = torch.randn(64, input_dim, generator=generator)
    with torch.inference_mode():
        expected = samples
        for i in range(len(dimensions) - 1):
            expected = nn.functional.linear(expected, state[f"mlp.{2*i}.weight"], state[f"mlp.{2*i}.bias"])
            if i < len(dimensions) - 2:
                expected = nn.functional.elu(expected)
        torch.testing.assert_close(scripted(samples), expected)
    output.parent.mkdir(parents=True, exist_ok=True)
    scripted.save(str(output))
    torch.testing.assert_close(torch.jit.load(str(output))(samples), expected)
    return int(saved["iter"])


def write_xml(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="unicode")
    with path.open("a") as stream:
        stream.write("\n")


def make_model(asset_root, urdf, destination, env, names, default_pos, efforts, force):
    destination.mkdir(parents=True, exist_ok=True)
    for directory in ("urdf", "meshes"):
        shutil.copytree(asset_root / directory, destination / directory, dirs_exist_ok=force)
    mjcf_dir = destination / "mjcf"
    mjcf_dir.mkdir(exist_ok=True)
    xml_path = mjcf_dir / "robocon.xml"
    if force:
        xml_path.unlink(missing_ok=True)
    convert_urdf(destination / urdf.relative_to(asset_root), xml_path, destination)
    root = ET.parse(xml_path).getroot()
    root.set("model", "ice_robocon")
    physics = env["sim"]["physics"]
    solver = physics["solver_cfg"]
    shape = physics["default_shape_cfg"]
    # Newton's nominal shape material is distinct from the terrain material.
    # The saved run uses mu=1, ke=160000, kd=1100, not MuJoCo's default
    # solref=(0.02, 1). Match the raw contact parameters observed in MJWarp.
    if shape["ke"] <= 0 or shape["kd"] <= 0:
        raise ValueError("Positive Newton contact stiffness and damping are required")
    contact_solref = f"{2.0 / shape['kd']} {shape['kd'] / (2.0 * np.sqrt(shape['ke']))}"
    ET.SubElement(root, "option", {
        "timestep": str(env["sim"]["dt"] / physics["num_substeps"]),
        "integrator": solver["integrator"], "cone": solver["cone"],
        "impratio": str(solver["impratio"]), "iterations": str(solver["iterations"]),
        "tolerance": str(solver["tolerance"]),
        "ls_iterations": str(solver["ls_iterations"]),
        "gravity": " ".join(map(str, env["sim"]["gravity"])),
    })
    base = root.find(f".//body[@name='{env['base_link_name']}']")
    if base is None:
        raise ValueError("The configured base body is absent from the converted URDF")
    base.set("pos", " ".join(map(str, env["scene"]["robot"]["init_state"]["pos"])))
    ET.SubElement(base, "freejoint", {"name": "floating_base"})
    ET.SubElement(base, "site", {"name": "imu", "pos": "0 0 0", "size": "0.005"})
    joints = {j.get("name"): j for j in root.findall(".//joint")}
    if set(joints) != set(names):
        raise ValueError("The URDF joints do not match the saved training joint names")
    # The training task disables robot self-collisions. Retain robot-ground contact.
    for geom in root.findall(".//geom"):
        if geom.get("contype", "1") != "0" or geom.get("conaffinity", "1") != "0":
            geom.set("contype", "2")
            geom.set("conaffinity", "1")
            geom.set("friction", f"{shape['mu']} 0.005 0.0001")
            geom.set("solref", contact_solref)
            geom.set("condim", "3")
            geom.set("group", "3")
    actuator = ET.SubElement(root, "actuator")
    sensor = ET.SubElement(root, "sensor")
    for name, effort in zip(names, efforts):
        ET.SubElement(actuator, "motor", {
            "name": name + "_motor", "joint": name, "gear": "1",
            "ctrllimited": "true", "ctrlrange": f"{-effort} {effort}",
        })
    for tag, suffix in (("jointpos", "pos"), ("jointvel", "vel"), ("jointactuatorfrc", "torque")):
        for name in names:
            ET.SubElement(sensor, tag, {"name": name + "_" + suffix, "joint": name})
    ET.SubElement(sensor, "framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu"})
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu"})
    write_xml(root, xml_path)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    qpos = model.qpos0.copy()
    for name, position in zip(names, default_pos):
        qpos[model.joint(name).qposadr[0]] = position
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", {"name": "home", "qpos": " ".join(map(str, qpos))})
    write_xml(root, xml_path)
    scene = ET.Element("mujoco", {"model": "ice_robocon_scene"})
    ET.SubElement(scene, "include", {"file": xml_path.name})
    ET.SubElement(scene, "statistic", {"center": "0 0 0.35", "extent": "1.5"})
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "global", {"azimuth": "135", "elevation": "-20"})
    asset = ET.SubElement(scene, "asset")
    ET.SubElement(asset, "texture", {"name": "grid", "type": "2d", "builtin": "checker",
                                     "rgb1": "0.2 0.25 0.3", "rgb2": "0.3 0.35 0.4", "width": "512", "height": "512"})
    ET.SubElement(asset, "material", {"name": "floor_material", "texture": "grid", "texrepeat": "10 10", "texuniform": "true"})
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", {"pos": "0 0 3", "dir": "0 0 -1", "directional": "true"})
    ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "size": "0 0 0.1",
                                  "contype": "1", "conaffinity": "2",
                                  "friction": f"{env['scene']['terrain']['physics_material']['dynamic_friction']} 0.005 0.0001",
                                  "solref": contact_solref,
                                  "material": "floor_material"})
    scene_path = mjcf_dir / "scene.xml"
    write_xml(scene, scene_path)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    assert model.nu == len(names) and model.nsensordata == 3 * len(names) + 7
    for i, name in enumerate(names):
        assert model.actuator_trnid[i, 0] == model.joint(name).id
        assert model.sensor_adr[i] == i
    return scene_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--asset-root", type=Path, required=True)
    parser.add_argument("--checkpoint", default="model_4999.pt")
    parser.add_argument("--model-only", action="store_true",
                        help="Generate MJCF using the saved environment and existing policy configuration; keep policy files")
    parser.add_argument("--force", action="store_true", help="Replace generated Robocon files")
    args = parser.parse_args()
    project = Path(__file__).resolve().parent.parent
    run = args.run_dir.resolve()
    asset_root = args.asset_root.resolve()
    env = read_config(run / "params/env.yaml")
    if args.model_only:
        base = read_config(project / "policy/ice/robocon/base.yaml")["ice/robocon"]
        config = read_config(project / "policy/ice/robocon/robot_lab/config.yaml")["ice/robocon/robot_lab"]
        if env["joint_names"] != base["joint_names"]:
            raise ValueError("Saved environment joint order differs from the deployed policy")
        if not np.isclose(env["sim"]["dt"], base["dt"]):
            raise ValueError("Saved environment timestep differs from the deployed policy")
        scene = make_model(
            asset_root, asset_root / "urdf/robocon.urdf",
            project / "src/rl_sar_zoo/ice/robocon_description", env,
            base["joint_names"], config["default_dof_pos"], config["torque_limits"], args.force,
        )
        print(f"Prepared model: {scene}")
        print("Existing policy and configuration files retained.")
        return
    agent = read_config(run / "params/agent.yaml")
    names = env["joint_names"]
    wheels = [names.index(name) for name in env["wheel_joint_names"]]
    if names != env["leg_joint_names"] + env["wheel_joint_names"]:
        raise ValueError("Expected leg actions followed by wheel actions")
    robot = env["scene"]["robot"]
    default_pos = [float(resolve(robot["init_state"]["joint_pos"], name)) for name in names]
    default_vel = [resolve(robot["init_state"]["joint_vel"], name) for name in names]
    if any(default_vel):
        raise ValueError("This importer expects zero default joint velocities")
    obs_names = {"base_ang_vel": "ang_vel", "projected_gravity": "gravity_vec", "velocity_commands": "commands",
                 "joint_pos": "dof_pos", "joint_vel": "dof_vel", "actions": "actions"}
    policy_obs = env["observations"]["policy"]
    terms = [key for key in obs_names if policy_obs.get(key) is not None]
    active = [key for key, value in policy_obs.items() if isinstance(value, dict) and "func" in value]
    if terms != active or len(terms) != 6 or policy_obs.get("history_length"):
        raise ValueError("Expected the six standard wheel-leg policy observations without history")
    for term in terms:
        if policy_obs[term].get("history_length") or policy_obs[term].get("delay_max_lag") or policy_obs[term].get("modifiers"):
            raise ValueError("Observation history, delay and modifiers need a custom adapter")
    observation_limits = []
    for term in terms:
        count = len(names) if term in ("joint_pos", "joint_vel", "actions") else 3
        limits = sorted(float(v) * policy_obs[term]["scale"] for v in policy_obs[term]["clip"])
        observation_limits.extend([limits] * count)
    for term in ("joint_pos", "joint_vel"):
        cfg = policy_obs[term]["params"]["asset_cfg"]
        if cfg["joint_names"] != names or not cfg["preserve_order"]:
            raise ValueError("Observation joint order must match policy action order")
    position = env["actions"]["joint_pos"]
    velocity = env["actions"]["joint_vel"]
    if position["joint_names"] + velocity["joint_names"] != names:
        raise ValueError("Action joint order does not match joint_names")
    gains, damping, efforts, scales, pos_limits, vel_limits = [], [], [], [], [], []
    for i, name in enumerate(names):
        actuators = [cfg for cfg in robot["actuators"].values()
                     if any(re.fullmatch(pattern, name) for pattern in cfg["joint_names_expr"])]
        if len(actuators) != 1:
            raise ValueError(f"Expected one actuator for {name}")
        actuator = actuators[0]
        gains.append(float(resolve(actuator["stiffness"], name)))
        damping.append(float(resolve(actuator["damping"], name)))
        efforts.append(float(resolve(actuator["joint_effort_limit"], name)))
        action = velocity if i in wheels else position
        scales.append(float(resolve(action["scale"], name)))
        limits = list(resolve(action["clip"], name))
        pos_limits.append([0.0, 0.0] if i in wheels else limits)
        vel_limits.append(limits if i in wheels else [0.0, 0.0])
    identity = list(range(len(names)))
    command_ranges = env["commands"]["base_velocity"]["ranges"]
    command_axes = ("lin_vel_x", "lin_vel_y", "ang_vel_z")
    base = {"dt": env["sim"]["dt"], "decimation": env["decimation"], "num_of_dofs": len(names),
            "initial_keyframe": "home", "camera_track_body": env["base_link_name"],
            "physics_sync_control": True,
            "command_limits_lower": [float(command_ranges[axis][0]) for axis in command_axes],
            "command_limits_upper": [float(command_ranges[axis][1]) for axis in command_axes],
            "fixed_kp": gains, "fixed_kd": damping,
            "torque_limits": efforts, "default_dof_pos": default_pos, "wheel_indices": wheels,
            "joint_names": names, "joint_controller_names": [name + "_controller" for name in names], "joint_mapping": identity}
    config = {"model_name": "policy.pt", "num_observations": 9 + 3 * len(names),
              "observations": [obs_names[name] for name in terms], "observations_history": [],
              "observations_history_priority": "term", "default_commands": [0.0, 0.0, 0.0],
              "clip_obs": 100.0, "clip_actions_lower": [], "clip_actions_upper": [],
              "clip_obs_lower": [v[0] for v in observation_limits],
              "clip_obs_upper": [v[1] for v in observation_limits],
              "rl_kp": gains, "rl_kd": damping, "num_of_dofs": len(names), "action_scale": scales,
              "wheel_indices": wheels, "lin_vel_scale": 1.0,
              "ang_vel_scale": policy_obs["base_ang_vel"]["scale"],
              "dof_pos_scale": policy_obs["joint_pos"]["scale"], "dof_vel_scale": policy_obs["joint_vel"]["scale"],
              "commands_scale": [policy_obs["velocity_commands"]["scale"]] * 3,
              "torque_limits": efforts, "default_dof_pos": default_pos, "joint_mapping": identity,
              "clip_dof_pos_lower": [v[0] for v in pos_limits], "clip_dof_pos_upper": [v[1] for v in pos_limits],
              "clip_dof_vel_lower": [v[0] for v in vel_limits], "clip_dof_vel_upper": [v[1] for v in vel_limits]}
    policy_dir = project / "policy/ice/robocon/robot_lab"
    if (policy_dir / "config.yaml").exists() and not args.force:
        parser.error("Robocon configuration already exists; use --force to regenerate")
    policy_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = run / args.checkpoint
    iteration = export_policy(checkpoint, agent, policy_dir / "policy.pt", config["num_observations"], len(names))
    urdf = asset_root / "urdf/robocon.urdf"
    scene = make_model(asset_root, urdf, project / "src/rl_sar_zoo/ice/robocon_description",
                       env, names, default_pos, efforts, args.force)
    header = "# Generated from the saved robot_lab run, not the current task defaults.\n"
    (policy_dir.parent / "base.yaml").write_text(header + yaml.safe_dump({"ice/robocon": base}, sort_keys=False, default_flow_style=None, width=100))
    (policy_dir / "config.yaml").write_text(header + "# Clip processed targets; retain raw actions for the next policy observation.\n"
                                          + yaml.safe_dump({"ice/robocon/robot_lab": config}, sort_keys=False, default_flow_style=None, width=100))
    metadata = {"task": "RobotLab-Isaac-Velocity-Flat-ICE-Robocon-v0", "checkpoint": str(checkpoint),
                "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(), "iteration": iteration,
                "source_env": str(run / "params/env.yaml"), "source_agent": str(run / "params/agent.yaml"),
                "source_urdf": str(urdf), "torch_version": str(torch.__version__), "mujoco_version": mujoco.__version__}
    (policy_dir / "source.yaml").write_text(yaml.safe_dump(metadata, sort_keys=False))
    print(f"Imported iteration {iteration}: {config['num_observations']} observations -> {len(names)} actions")
    print(f"Scene: {scene}")
    print("Run: ./cmake_build/bin/rl_sim_mujoco ice/robocon scene")


if __name__ == "__main__":
    main()
