#!/usr/bin/env python3
"""Replay the exported Robocon policy deterministically, with CSV/video output.

This Python diagnostic follows the RL SDK observation/target contract. The
native LibTorch/SDK integration is checked separately by test_ice_robocon.
"""

import argparse
import csv
import json
import os
from pathlib import Path

import numpy as np
import torch
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--command", type=float, nargs=3, metavar=("VX", "VY", "WZ"))
    parser.add_argument("--profile", choices=("four-states", "stop-start"), default="four-states")
    parser.add_argument("--policy-root", type=Path, help="Robocon directory containing base.yaml and robot_lab/")
    parser.add_argument("--ramp-commands", action="store_true", help="Use the deployed acceleration/deceleration limits")
    parser.add_argument("--drive", choices=("explicit", "implicit"), help="Override the configured MuJoCo drive")
    parser.add_argument(
        "--output", type=Path, default=Path("logs/robocon_diagnostics/latest")
    )
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    if args.command is not None and args.profile != "four-states":
        parser.error("Choose either a constant command or a named profile")
    if args.seconds is None:
        args.seconds = 34.0 if args.profile == "stop-start" else 22.0
    if args.seconds <= 2 or not np.isfinite(args.seconds):
        parser.error("seconds must be finite and greater than 2")
    if args.command and not np.isfinite(args.command).all():
        parser.error("commands must be finite")
    # Headless rendering does not need a desktop window.
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco as mj

    project = Path(__file__).resolve().parent.parent
    policy_root = args.policy_root.resolve() if args.policy_root else project / "policy/ice/robocon"
    cfg = yaml.safe_load(
        (policy_root / "robot_lab/config.yaml").read_text()
    )["ice/robocon/robot_lab"]
    base_cfg = yaml.safe_load((policy_root / "base.yaml").read_text())[
        "ice/robocon"
    ]
    args.drive = args.drive or base_cfg.get("mujoco_joint_drive", "explicit")
    if args.drive not in ("explicit", "implicit"):
        raise ValueError("mujoco_joint_drive must be explicit or implicit")
    source = yaml.safe_load(
        (policy_root / "robot_lab/source.yaml").read_text()
    )
    model = mj.MjModel.from_xml_path(
        str(project / "src/rl_sar_zoo/ice/robocon_description/mjcf/scene.xml")
    )
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(
        model, data, mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "home")
    )
    body = model.body("base_link").id
    count = base_cfg["num_of_dofs"]
    wheel = base_cfg["wheel_indices"]
    foot_bodies = [model.jnt_bodyid[model.joint(base_cfg["joint_names"][i]).id] for i in wheel]
    kp, kd = np.array(cfg["rl_kp"]), np.array(cfg["rl_kd"])
    default = np.array(cfg["default_dof_pos"])
    control_stride = round(base_cfg["dt"] / model.opt.timestep)
    if not np.isclose(control_stride * model.opt.timestep, base_cfg["dt"]):
        raise ValueError("Control dt must be a multiple of the physics timestep")
    policy_stride = control_stride * base_cfg["decimation"]
    if args.drive == "implicit":
        # A combined affine servo models the same PD gains with implicit
        # velocity feedback; effort limits constrain the sum of P and D.
        model.actuator_biastype[:] = mj.mjtBias.mjBIAS_AFFINE
        model.actuator_biasprm[:, 1] = -kp
        model.actuator_biasprm[:, 2] = -kd
        model.actuator_ctrllimited[:] = False
        model.actuator_forcelimited[:] = True
        model.actuator_forcerange[:] = np.array(cfg["torque_limits"])[:, None] * [-1, 1]
    mj.mj_forward(model, data)
    torch.set_num_threads(1)
    policy = torch.jit.load(
        str(policy_root / "robot_lab" / cfg["model_name"]), map_location="cpu"
    ).eval()
    profile = (
        [(0, 7, [0, 0, 0], "standing"), (7, 12, [0.5, 0, 0], "forward"),
         (12, 17, [0, 0.2, 0], "lateral"), (17, args.seconds, [0, 0, 0.6], "turn")]
        if args.profile == "four-states"
        else [(0, 7, [0, 0, 0], "standing"), (7, 12, [0.5, 0, 0], "forward"),
              (12, 16, [0, 0, 0], "stop_forward"), (16, 21, [0, 0.2, 0], "lateral"),
              (21, 25, [0, 0, 0], "stop_lateral"), (25, 30, [0, 0, 0.6], "turn"),
              (30, args.seconds, [0, 0, 0], "stop_turn")]
    )
    action = np.zeros(count)
    pos, vel = default.copy(), np.zeros(count)
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    diagnostics = {name: [] for name in ("time", "command", "joint_pos", "joint_vel", "position_target",
                                        "velocity_target", "actions", "foot_normal_force", "foot_position",
                                        "base_position", "base_rotation")}
    ramped_command = np.zeros(3)
    acceleration = np.array(base_cfg.get("command_acceleration_limits", [1.0, 0.3, 0.8]))
    deceleration = np.array(base_cfg.get("command_deceleration_limits", [1.5, 0.5, 1.2]))
    renderer = writer = None
    if args.video:
        import imageio.v2 as imageio
        from PIL import Image, ImageDraw

        model.vis.global_.offwidth, model.vis.global_.offheight = 960, 720
        renderer = mj.Renderer(model, height=720, width=960)
        camera = mj.MjvCamera()
        mj.mjv_defaultCamera(camera)
        camera.distance, camera.azimuth, camera.elevation = 2.0, 135, -22
        option = mj.MjvOption()
        option.geomgroup[3] = 0  # Show CAD visuals; retain collisions in physics.
        writer = imageio.get_writer(str(args.output / "rollout.mp4"), fps=30)
    next_frame = 0.0
    try:
        for step in range(round(args.seconds / model.opt.timestep)):
            t = step * model.opt.timestep
            command = np.array(args.command if args.command is not None else
                               next(value for begin, end, value, _ in profile if begin <= t < end))
            if args.ramp_commands:
                if step % control_stride == 0:
                    target = command if t >= 2.0 else np.zeros(3)
                    reversing = ramped_command * target < 0
                    speeding_up = (~reversing) & (np.abs(target) > np.abs(ramped_command))
                    limit = np.where(speeding_up, acceleration, deceleration) * base_cfg["dt"]
                    target = np.where(reversing, 0.0, target)
                    ramped_command += np.clip(target - ramped_command, -limit, limit)
                command = ramped_command.copy()
            if step % policy_stride == 0:
                joint_pos, joint_vel = (
                    data.sensordata[:count],
                    data.sensordata[count : 2 * count],
                )
                rotation = data.xmat[body].reshape(3, 3)
                relative = joint_pos - default
                relative[wheel] = 0
                observation = np.concatenate(
                    [
                        data.sensordata[3 * count + 4 : 3 * count + 7]
                        * cfg["ang_vel_scale"],
                        -rotation[2],
                        command * cfg["commands_scale"],
                        relative * cfg["dof_pos_scale"],
                        joint_vel * cfg["dof_vel_scale"],
                        action,
                    ]
                )
                observation = np.clip(
                    observation, cfg["clip_obs_lower"], cfg["clip_obs_upper"]
                )
                if t >= 2.0:
                    with torch.inference_mode():
                        action = policy(
                            torch.tensor(observation, dtype=torch.float32)[None]
                        ).numpy()[0]
                    if not np.isfinite(action).all():
                        raise ValueError(f"Non-finite action at {t}")
                    position_action = action * cfg["action_scale"]
                    velocity_action = np.zeros(count)
                    velocity_action[wheel] = position_action[wheel]
                    position_action[wheel] = 0
                    pos = np.clip(
                        default + position_action,
                        cfg["clip_dof_pos_lower"],
                        cfg["clip_dof_pos_upper"],
                    )
                    vel = np.clip(
                        velocity_action,
                        cfg["clip_dof_vel_lower"],
                        cfg["clip_dof_vel_upper"],
                    )
            if args.drive == "implicit":
                data.ctrl[:] = kp * pos + kd * vel
            elif step % control_stride == 0:
                data.ctrl[:] = kp * (pos - data.sensordata[:count]) + kd * (
                    vel - data.sensordata[count : 2 * count]
                )
            mj.mj_step(model, data)
            if step % policy_stride == 0:
                velocity = np.zeros(6)
                mj.mj_objectVelocity(
                    model, data, mj.mjtObj.mjOBJ_XBODY, body, velocity, 1
                )
                tilt = np.arccos(np.clip(data.xmat[body, 8], -1, 1))
                rows.append(
                    [
                        data.time,
                        *command,
                        velocity[3],
                        velocity[4],
                        velocity[2],
                        data.xpos[body, 2],
                        tilt,
                        *data.sensordata[count + np.array(wheel)],
                    ]
                )
                normal_force = np.zeros(len(wheel))
                for contact_index, contact in enumerate(data.contact):
                    bodies = model.geom_bodyid[contact.geom]
                    if 0 not in bodies:
                        continue
                    for foot, foot_body in enumerate(foot_bodies):
                        if foot_body in bodies:
                            force = np.zeros(6)
                            mj.mj_contactForce(model, data, contact_index, force)
                            normal_force[foot] += max(0.0, force[0])
                for name, value in (
                    ("time", data.time), ("command", command),
                    ("joint_pos", data.sensordata[:count]), ("joint_vel", data.sensordata[count:2*count]),
                    ("position_target", pos), ("velocity_target", vel), ("actions", action),
                    ("foot_normal_force", normal_force), ("foot_position", data.xpos[foot_bodies]),
                    ("base_position", data.xpos[body]), ("base_rotation", data.xmat[body].reshape(3, 3)),
                ):
                    diagnostics[name].append(np.array(value, copy=True))
            if renderer and data.time >= next_frame:
                camera.lookat[:] = data.xpos[body]
                renderer.update_scene(data, camera=camera, scene_option=option)
                frame = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0, 0, 960, 50), fill=(15, 22, 32))
                draw.text(
                    (15, 10),
                    f"Robocon | {Path(source['checkpoint']).stem} | t={data.time:5.2f}s | cmd vx/vy/wz {command[0]:+.2f} / {command[1]:+.2f} / {command[2]:+.2f}",
                    fill="white",
                    font_size=20,
                )
                writer.append_data(np.array(frame))
                next_frame += 1 / 30
    finally:
        if writer:
            writer.close()
        if renderer:
            renderer.close()
    arr = np.array(rows)
    np.savez_compressed(args.output / "diagnostics.npz", **{k: np.array(v) for k, v in diagnostics.items()})
    with (args.output / "telemetry.csv").open("w") as stream:
        csv_writer = csv.writer(stream)
        csv_writer.writerow(
            ["time", "cmd_vx", "cmd_vy", "cmd_wz", "vx", "vy", "wz", "height", "tilt"]
            + ["wheel_dq_" + base_cfg["joint_names"][i] for i in wheel]
        )
        csv_writer.writerows(rows)
    segments = []
    intervals = (
        [(max(begin, min(end, args.seconds) - 3, 2), min(end, args.seconds), label)
         for begin, end, _, label in profile if begin < min(end, args.seconds)]
        if args.command is None
        else [(max(2, args.seconds - 3), args.seconds, "constant")]
    )
    for begin, end, label in intervals:
        samples = arr[(arr[:, 0] >= begin) & (arr[:, 0] < end)]
        if not len(samples):
            continue
        segments.append(
            {
                "label": label,
                "interval": [begin, end],
                "command": samples[:, 1:4].mean(0).tolist(),
                "mean_velocity": samples[:, 4:7].mean(0).tolist(),
                "rmse": np.sqrt(
                    np.mean((samples[:, 4:7] - samples[:, 1:4]) ** 2, axis=0)
                ).tolist(),
                "wheel_speed_rms_rad_s": np.sqrt(np.mean(samples[:, 9:] ** 2, axis=0)).tolist(),
            }
        )
    report = {
        "checkpoint": source["checkpoint"],
        "checkpoint_sha256": source["checkpoint_sha256"],
        "mujoco": mj.__version__,
        "torch": torch.__version__,
        "drive": args.drive,
        "ramp_commands": args.ramp_commands,
        "profile": args.profile if args.command is None else "constant",
        "policy_root": str(policy_root),
        "seconds": args.seconds,
        "physics_dt": model.opt.timestep,
        "control_dt": base_cfg["dt"],
        "policy_dt": base_cfg["dt"] * base_cfg["decimation"],
        "min_height": float(arr[:, 7].min()),
        "max_tilt_rad": float(arr[:, 8].max()),
        "fell": bool(np.any(arr[:, 7] < 0.18) or np.any(arr[:, 8] > 1.0)),
        "segments": segments,
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return int(report["fell"])


if __name__ == "__main__":
    raise SystemExit(main())
