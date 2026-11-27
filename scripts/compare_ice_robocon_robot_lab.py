#!/usr/bin/env python3
"""Run the same nominal 22 s command profile in the original robot_lab runtime.

Run with robot_lab/.venv/bin/python, not rl_sar/.venv-deploy/bin/python.
Training task/physics code stays in robot_lab; no training files are changed.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import torch
import warp as wp

wp.config.enable_backward = False


def main():
    project = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-lab", type=Path, default=project.parent / "robot_lab")
    parser.add_argument(
        "--output", type=Path, default=project / "logs/robocon_diagnostics/robot_lab"
    )
    parser.add_argument("--low-commands", action="store_true")
    parser.add_argument("--ramp-commands", action="store_true")
    parser.add_argument("--forward-speed", type=float, help="Override the profile's forward speed")
    parser.add_argument(
        "--strafe-speed", type=float, help="Override the profile's lateral speed"
    )
    args = parser.parse_args()
    sys.path.insert(0, str(args.robot_lab / "scripts/reinforcement_learning/rsl_rl"))
    from isaaclab.app import add_launcher_args, launch_simulation
    from isaaclab.envs.mdp.commands import UniformVelocityCommand
    from isaaclab_rl.entrypoints.common import (
        add_common_play_args,
        apply_env_overrides,
        close_env,
        create_isaaclab_env,
    )
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli
    from keyboard_play import configure_keyboard
    from isaaclab_newton.physics import NewtonManager
    import robot_lab.tasks  # noqa: F401

    commands = (0.3, 0.1, 0.2) if args.low_commands else (0.5, 0.2, 0.6)
    if args.forward_speed is not None:
        if not np.isfinite(args.forward_speed):
            parser.error("forward-speed must be finite")
        commands = (args.forward_speed, commands[1], commands[2])
    if args.strafe_speed is not None:
        if not np.isfinite(args.strafe_speed):
            parser.error("strafe-speed must be finite")
        commands = (commands[0], args.strafe_speed, commands[2])

    class ScriptedCommands(UniformVelocityCommand):
        def _resample_command(self, env_ids):
            self.vel_command_b[env_ids] = 0

        def _update_command(self):
            t = self._env.common_step_counter * self._env.step_dt
            value = (
                [0, 0, 0]
                if t < 7
                else [commands[0], 0, 0]
                if t < 12
                else [0, commands[1], 0]
                if t < 17
                else [0, 0, commands[2]]
            )
            target = torch.tensor(value, device=self.device)
            if args.ramp_commands:
                reversing = self.vel_command_b * target < 0
                speeding_up = (~reversing) & (target.abs() > self.vel_command_b.abs())
                acceleration = torch.tensor(self.cfg.acceleration_limits, device=self.device)
                deceleration = torch.tensor(self.cfg.deceleration_limits, device=self.device)
                limit = torch.where(speeding_up, acceleration, deceleration) * self._env.step_dt
                target = torch.where(reversing, 0.0, target)
                self.vel_command_b += torch.clamp(target - self.vel_command_b, min=-limit, max=limit)
            else:
                self.vel_command_b[:] = target

    # Reuse the training launcher's argument defaults and task registry.
    launcher_parser = argparse.ArgumentParser()
    add_common_play_args(
        launcher_parser, agent_default="rsl_rl_cfg_entry_point", agent_help="agent"
    )
    add_launcher_args(launcher_parser)
    launch_args, overrides = setup_preset_cli(
        launcher_parser,
        [
            "--task",
            "RobotLab-Isaac-Velocity-Flat-ICE-Robocon-v0",
            "--num_envs",
            "1",
            "--viz",
            "none",
        ],
    )
    cfg, agent = resolve_task_config(
        launch_args.task, launch_args.agent, play_mode=True, overrides=overrides
    )
    apply_env_overrides(launch_args, cfg)
    launch_args.speed, launch_args.strafe_speed, launch_args.turn_speed = commands
    configure_keyboard(cfg, launch_args)
    cfg.seed = 42
    cfg.commands.base_velocity.class_type = ScriptedCommands
    cfg.commands.base_velocity.debug_vis = False
    # The scripted timeline owns transitions. Random resampling would otherwise
    # zero the ramp mid-segment even though the requested command is unchanged.
    cfg.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
    for name, term in list(vars(cfg.events).items()):
        if getattr(term, "mode", None) == "startup":
            setattr(cfg.events, name, None)
    cfg.events.randomize_reset_joints.params["position_range"] = (1.0, 1.0)
    cfg.events.randomize_reset_joints.params["velocity_range"] = (0.0, 0.0)
    args.output.mkdir(parents=True, exist_ok=True)
    cfg.sim.physics.solver_cfg.save_to_mjcf = str(args.output / "training_model.xml")
    torch.set_num_threads(1)
    with launch_simulation(cfg, launch_args):
        env = RslRlVecEnvWrapper(
            create_isaaclab_env(
                launch_args.task, cfg, launch_args, convert_marl_to_single_agent=False
            ),
            clip_actions=None,
        )
        try:
            base = env.unwrapped
            policy = torch.jit.load(
                str(project / "policy/ice/robocon/robot_lab/policy.pt"),
                map_location=base.device,
            ).eval()
            obs = env.get_observations()
            rows, resets = [], 0
            for step in range(round(22.0 / base.step_dt)):
                with torch.inference_mode():
                    action = (
                        policy(obs["policy"])
                        if step * base.step_dt >= 2
                        else torch.zeros((1, 16), device=base.device)
                    )
                    obs, _, done, _ = env.step(action)
                data = base.scene["robot"].data
                velocity = data.root_lin_vel_b.torch[0].cpu().numpy()
                angular_velocity = data.root_ang_vel_b.torch[0].cpu().numpy()
                height = data.root_pos_w.torch[0, 2].item()
                gravity = data.projected_gravity_b.torch[0].cpu().numpy()
                rows.append(
                    [
                        step * base.step_dt,
                        height,
                        np.arccos(np.clip(-gravity[2], -1, 1)),
                        *velocity[:2],
                        angular_velocity[2],
                        *base.command_manager.get_command("base_velocity")[0].cpu().numpy(),
                    ]
                )
                resets += int(done.sum())
            samples = np.array(rows)
            segments = []
            for begin, end, command in zip(
                (4, 9, 14, 19),
                (7, 12, 17, 22),
                (
                    [0, 0, 0],
                    [commands[0], 0, 0],
                    [0, commands[1], 0],
                    [0, 0, commands[2]],
                ),
            ):
                selected = samples[(samples[:, 0] >= begin) & (samples[:, 0] < end)]
                values = selected[:, 3:6]
                actual_command = selected[:, 6:9]
                segments.append(
                    {
                        "interval": [begin, end],
                        "target_command": command,
                        "command": actual_command.mean(0).tolist(),
                        "mean_velocity": values.mean(0).tolist(),
                        "rmse": np.sqrt(
                            np.mean((values - actual_command) ** 2, axis=0)
                        ).tolist(),
                    }
                )
            solver = NewtonManager._solver
            active = int(solver.mjw_data.nacon.numpy()[0])
            report = {
                "task": launch_args.task,
                "seed": 42,
                "startup_randomization": False,
                "observation_noise": False,
                "ramp_commands": args.ramp_commands,
                "seconds": 22,
                "resets": resets,
                "min_height": float(samples[:, 1].min()),
                "max_tilt_rad": float(samples[:, 2].max()),
                "segments": segments,
                "contact_friction": np.unique(
                    solver.mjw_data.contact.friction.numpy()[:active], axis=0
                ).tolist(),
                "contact_solref": np.unique(
                    solver.mjw_data.contact.solref.numpy()[:active], axis=0
                ).tolist(),
            }
            np.savez_compressed(args.output / "rollout.npz", samples=samples)
            (args.output / "report.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print("RESULT", json.dumps(report), flush=True)
        finally:
            close_env(env)


if __name__ == "__main__":
    main()
