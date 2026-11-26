/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

// Exercise the actual ICE states, LibTorch policy and MuJoCo plant without GUI
// timing. This uses the same pre-inference hook and control order as the viewer.
#include "fsm_ice_robocon.hpp"
#include <mujoco/mujoco.h>
#include <filesystem>
#include <fstream>

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

float difference(const std::vector<float>& a, const std::vector<float>& b)
{
    float result = 0.0f;
    require(a.size() == b.size(), "Vector size changed");
    for (size_t i = 0; i < a.size(); ++i) result = std::max(result, std::abs(a[i] - b[i]));
    return result;
}

class Controller : public RL
{
public:
    explicit Controller(const std::string& name = "manual")
    {
        robot_name = "ice/robocon";
        ang_vel_axis = "body";
        ReadYaml(robot_name, "base.yaml");
        InitJointNum(params.Get<int>("num_of_dofs"));
        InitOutputs();
        InitControl();
        control.velocity_from_axes = true;
        const auto root = std::filesystem::path(POLICY_DIR).parent_path();
        char error[1024] = {};
        m.reset(mj_loadXML((root / "src/rl_sar_zoo/ice/robocon_description/mjcf/scene.xml").c_str(),
                          nullptr, error, sizeof(error)));
        require(bool(m), error);
        d.reset(mj_makeData(m.get()));
        Reset();
        GetState(&robot_state);
        fsm = *FSMManager::GetInstance().CreateFSM(robot_name, this);
        const auto output = root / "logs/robocon_diagnostics/fsm";
        std::filesystem::create_directories(output);
        telemetry.open(output / (name + ".csv"));
        telemetry << "time,stage,cmd_vx,cmd_vy,cmd_wz,height,tilt,state,vx,vy,wz\n";
    }

    std::vector<float> Forward() override { return model->forward({ComputeObservation()}); }

    void GetState(RobotState<float>* state) override
    {
        for (int i = 0; i < 16; ++i)
        {
            state->motor_state.q[i] = d->sensordata[i];
            state->motor_state.dq[i] = d->sensordata[16+i];
        }
        for (int i = 0; i < 4; ++i) state->imu.quaternion[i] = d->sensordata[48+i];
        for (int i = 0; i < 3; ++i) state->imu.gyroscope[i] = d->sensordata[52+i];
    }

    void SetCommand(const RobotCommand<float>* command) override
    {
        for (int i = 0; i < 16; ++i)
            d->ctrl[i] = command->motor_command.tau[i]
                + command->motor_command.kp[i] * (command->motor_command.q[i] - d->sensordata[i])
                + command->motor_command.kd[i] * (command->motor_command.dq[i] - d->sensordata[16+i]);
    }

    void Reset()
    {
        mj_resetDataKeyframe(m.get(), d.get(), mj_name2id(m.get(), mjOBJ_KEY, "home"));
        mj_forward(m.get(), d.get());
        tick = 0;
    }

    std::string StateName() const { return fsm.current_state_->GetStateName(); }

    void Step(std::array<float, 3> input = {}, Input::Keyboard key = Input::Keyboard::None,
              bool running = true)
    {
        control.x = input[0]; control.y = input[1]; control.yaw = input[2];
        control.current_keyboard = key;
        GetState(&robot_state);
        ice_robocon_fsm::PrepareControl(*this, d->time, running);
        if (running && rl_init_done && tick % params.Get<int>("decimation") == 0)
        {
            ++episode_length_buf;
            obs.ang_vel = robot_state.imu.gyroscope;
            obs.base_quat = robot_state.imu.quaternion;
            obs.dof_pos = robot_state.motor_state.q;
            obs.dof_vel = robot_state.motor_state.dq;
            obs.commands = {control.x, control.y, control.yaw};
            obs.actions = Forward();
            ComputeOutput(obs.actions, output_dof_pos, output_dof_vel, output_dof_tau);
            output_dof_pos_queue.push(output_dof_pos);
            output_dof_vel_queue.push(output_dof_vel);
        }
        const auto before = robot_command.motor_command;
        const std::string previous_state = StateName();
        StateController(&robot_state, &robot_command);
        const bool previously_driven = std::any_of(before.kd.begin(), before.kd.end(),
            [](float value) { return value != 0.0f; });
        if (previously_driven && previous_state != StateName() && StateName() == "RLFSMStateRLLocomotion")
        {
            for (size_t i = 0; i < before.q.size(); ++i)
                if (before.kp[i] > 0 && robot_command.motor_command.kp[i] > 0)
                    require(std::abs(before.q[i] - robot_command.motor_command.q[i]) < 1e-4f,
                            "Position-controlled joint jumped when policy state entered");
            require(difference(before.dq, robot_command.motor_command.dq) < 1e-4f,
                    "Wheel target jumped when policy state entered");
        }
        require(ice_robocon_fsm::ValidCommand(robot_command.motor_command, 16), "Invalid motor command");
        control.current_keyboard = control.last_keyboard = Input::Keyboard::None;
        control.current_gamepad = Input::Gamepad::None;
        SetCommand(&robot_command);
        if (running)
        {
            for (int step = 0; step < 2; ++step) mj_step(m.get(), d.get());
            ++tick;
        }
        const int body = mj_name2id(m.get(), mjOBJ_BODY, "base_link");
        const double height = d->xpos[3*body+2];
        const double tilt = std::acos(std::clamp(d->xmat[9*body+8], -1.0, 1.0));
        mjtNum measured[6];
        mj_objectVelocity(m.get(), d.get(), mjOBJ_XBODY, body, measured, 1);
        actual_velocity = {measured[3], measured[4], measured[2]};
        if (StateName() == "RLFSMStateGetUp" || StateName() == "RLFSMStateRLLocomotion")
        {
            min_height = std::min(min_height, height);
            max_tilt = std::max(max_tilt, tilt);
            require(std::isfinite(height) && height > 0.18 && tilt < 1.0,
                    "Robot fell in " + stage + " at t=" + std::to_string(d->time));
        }
        telemetry << d->time << ',' << stage << ',' << control.x << ',' << control.y << ','
                  << control.yaw << ',' << height << ',' << tilt << ',' << StateName() << ','
                  << actual_velocity[0] << ',' << actual_velocity[1] << ',' << actual_velocity[2] << '\n';
    }

    void Run(float seconds, std::array<float, 3> input = {})
    {
        const std::array<float, 3> bound{1.5f, 0.5f, 1.2f};
        const int ticks = std::lround(seconds / params.Get<float>("dt"));
        double forward_sum = 0.0;
        int samples = 0;
        for (int i = 0; i < ticks; ++i)
        {
            const auto previous = std::array<float, 3>{control.x, control.y, control.yaw};
            Step(input);
            if (i >= ticks - 200) { forward_sum += actual_velocity[0]; ++samples; }
            if (StateName() == "RLFSMStateRLLocomotion")
            {
                const auto current = std::array<float, 3>{control.x, control.y, control.yaw};
                for (int axis = 0; axis < 3; ++axis)
                    require(std::abs(current[axis] - previous[axis]) <= bound[axis] * 0.005f + 1e-5f,
                            "Velocity command discontinuity in " + stage);
            }
        }
        if (stage == "forward" || stage == "reverse" || stage == "full_forward")
            require(std::abs(forward_sum / samples - input[0]) < std::max(0.15f, std::abs(input[0]) * 0.25f),
                    "Forward velocity tracking failed in " + stage);
    }

    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d{nullptr, mj_deleteData};
    int tick = 0;
    std::string stage = "setup";
    double min_height = 10.0, max_tilt = 0.0;
    std::array<double, 3> actual_velocity{};
    std::ofstream telemetry;
};

int main()
{
    try
    {
        Controller rl;
        rl.Step({}, Input::Keyboard::Num0, false);
        rl.Step({}, Input::Keyboard::None, false);
        require(rl.StateName() == "RLFSMStateGetUp", "Manual get-up did not start");
        const auto paused_pose = rl.robot_command.motor_command;
        for (int i = 0; i < 100; ++i) rl.Step({}, Input::Keyboard::None, false);
        require(difference(paused_pose.q, rl.robot_command.motor_command.q) < 1e-6f,
                "Get-up advanced while paused");
        rl.Step({}, Input::Keyboard::Num1, false);
        rl.Step({}, Input::Keyboard::None, false);
        require(rl.StateName() == "RLFSMStateGetUp", "Policy enabled before get-up finished");
        rl.stage = "getup";
        rl.Run(2.2f);
        rl.Step({}, Input::Keyboard::Num1);
        rl.Step();
        require(rl.StateName() == "RLFSMStateRLLocomotion", "Policy did not start");
        rl.stage = "handoff";
        rl.Run(1.0f);

        rl.stage = "forward"; rl.Run(5.0f, {0.5f, 0, 0});
        rl.stage = "stop"; rl.Run(3.0f);
        require(std::abs(rl.control.x) < 1e-4f, "Release did not stop the command");
        rl.stage = "reverse"; rl.Run(4.0f, {-0.5f, 0, 0});
        rl.stage = "combined"; rl.Run(5.0f, {0.5f, 0.2f, 0.4f});
        rl.stage = "turn"; rl.Run(4.0f, {0, 0, 0.6f});
        rl.stage = "full_forward"; rl.Run(6.0f, {2.0f, 0, 0});
        rl.stage = "stop_again"; rl.Run(3.0f);

        rl.stage = "pause";
        rl.Run(0.2f, {1.0f, 0, 0});
        rl.Step({1.0f, 0, 0}, Input::Keyboard::None, false);
        const auto paused_command = rl.robot_command.motor_command;
        const float paused_velocity = rl.control.x;
        for (int i = 0; i < 100; ++i) rl.Step({-1, 0, 0}, Input::Keyboard::None, false);
        require(difference(paused_command.q, rl.robot_command.motor_command.q) < 1e-6f &&
                std::abs(paused_velocity - rl.control.x) < 1e-6f, "Pause advanced a ramp");
        rl.Run(1.0f);

        rl.stage = "reset";
        rl.Reset();
        rl.Step();
        require(std::abs(rl.control.x) < 1e-6f, "Reset retained a velocity command");
        rl.Run(1.0f);

        rl.stage = "normal_exit";
        rl.Run(3.0f, {0.5f, 0, 0});
        rl.Step({0.5f, 0, 0}, Input::Keyboard::Num9);
        rl.Step({0.5f, 0, 0}, Input::Keyboard::Num1);
        rl.Run(1.0f, {0.5f, 0, 0});
        require(rl.StateName() == "RLFSMStateRLLocomotion", "Cancel did not retain policy control");
        rl.Step({0.5f, 0, 0}, Input::Keyboard::Num0);
        require(rl.StateName() == "RLFSMStateRLLocomotion", "Normal exit skipped braking");
        for (int i = 0; i < 1600 && rl.StateName() == "RLFSMStateRLLocomotion"; ++i)
            rl.Step({0.5f, 0, 0});
        require(rl.StateName() == "RLFSMStateGetUp", "Normal exit never reached stand");
        rl.Run(2.2f);
        rl.Step({}, Input::Keyboard::Num1);
        rl.Step();
        rl.Run(1.0f);
        require(rl.StateName() == "RLFSMStateRLLocomotion", "Policy re-entry failed");

        rl.stage = "getdown";
        rl.Run(2.0f, {0.5f, 0, 0});
        rl.Step({0.5f, 0, 0}, Input::Keyboard::Num9);
        for (int i = 0; i < 1800 && rl.StateName() != "RLFSMStatePassive"; ++i) rl.Step();
        require(rl.StateName() == "RLFSMStatePassive", "Get-down never completed");

        std::cout << "PASS: get-up, policy handoff, acceleration/deceleration, reverse, combined/full commands, "
                     "pause, reset, stop-before-exit, policy re-entry and get-down\n"
                  << "Minimum active height: " << rl.min_height << " m; maximum tilt: " << rl.max_tilt << " rad\n";

        Controller automatic("auto");
        automatic.fsm.RequestStateChange("RLFSMStateRLLocomotion");
        automatic.stage = "auto_policy";
        automatic.Run(2.0f);
        require(automatic.StateName() == "RLFSMStateRLLocomotion", "Auto-policy entry failed");
        automatic.Run(1.0f, {0.5f, 0, 0});
        automatic.Step({0.5f, 0, 0}, Input::Keyboard::P);
        automatic.Step();
        require(automatic.StateName() == "RLFSMStatePassive", "Immediate damping waited for normal stop");
        std::cout << "PASS: --auto-policy entry and immediate P damping\n";

        Controller pose("pose");
        for (const auto& leg : std::vector<std::pair<std::string, float>>{
                 {"fr", -1}, {"fl", 1}, {"rr", -1}, {"rl", 1}})
        {
            pose.d->qpos[pose.m->jnt_qposadr[mj_name2id(pose.m.get(), mjOBJ_JOINT,
                (leg.first + "_thigh_joint").c_str())]] = leg.second * 0.05;
            pose.d->qpos[pose.m->jnt_qposadr[mj_name2id(pose.m.get(), mjOBJ_JOINT,
                (leg.first + "_calf_joint").c_str())]] = -leg.second * 0.1;
        }
        mj_forward(pose.m.get(), pose.d.get());
        const int wheel = mj_name2id(pose.m.get(), mjOBJ_GEOM, "fr_foot_link_collision_0");
        pose.d->qpos[2] -= pose.d->geom_xpos[3*wheel+2] - pose.m->geom_size[3*wheel] - 0.001;
        mj_forward(pose.m.get(), pose.d.get());
        pose.stage = "different_initial_pose";
        pose.Step({}, Input::Keyboard::Num0);
        pose.Step();
        for (int i = 0; i < 440; ++i)
        {
            const auto before = pose.robot_command.motor_command.q;
            pose.Step();
            require(difference(before, pose.robot_command.motor_command.q) < 0.005f,
                    "Get-up position target changed too abruptly");
        }
        const auto stand = pose.params.Get<std::vector<float>>("default_dof_pos");
        require(difference(stand, pose.robot_command.motor_command.q) < 1e-5f,
                "Get-up failed to reach the configured stand target");
        std::cout << "PASS: smooth get-up from a different grounded joint pose\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << std::endl;
        return 1;
    }
}
