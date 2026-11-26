/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ICE_ROBOCON_FSM_HPP
#define ICE_ROBOCON_FSM_HPP

#include "fsm_go2w.hpp"
#include <array>
#include <cmath>
#include <limits>

namespace ice_robocon_fsm
{

using MotorCommand = RobotCommand<float>::MotorCommand;

inline float SmoothStep(float progress)
{
    const float s = std::clamp(progress, 0.0f, 1.0f);
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

inline bool Key(const RL& rl, Input::Keyboard keyboard, Input::Gamepad gamepad)
{
    return rl.control.current_keyboard == keyboard || rl.control.current_gamepad == gamepad;
}

inline void ClearOutputs(RL& rl)
{
    std::vector<float> discarded;
    while (rl.output_dof_pos_queue.try_pop(discarded)) {}
    while (rl.output_dof_vel_queue.try_pop(discarded)) {}
    while (rl.output_dof_tau_queue.try_pop(discarded)) {}
}

inline bool ValidCommand(const MotorCommand& command, size_t count)
{
    for (const auto* values : {&command.q, &command.dq, &command.kp, &command.kd, &command.tau})
    {
        if (values->size() != count) return false;
        for (float value : *values) if (!std::isfinite(value)) return false;
    }
    return true;
}

inline MotorCommand Blend(const MotorCommand& start, const MotorCommand& target, float weight)
{
    MotorCommand result = target;
    auto mix = [weight](float a, float b) { return a + weight * (b - a); };
    for (size_t i = 0; i < target.q.size(); ++i)
    {
        result.q[i] = mix(start.q[i], target.q[i]);
        result.dq[i] = mix(start.dq[i], target.dq[i]);
        result.kp[i] = mix(start.kp[i], target.kp[i]);
        result.kd[i] = mix(start.kd[i], target.kd[i]);
        result.tau[i] = mix(start.tau[i], target.tau[i]);
    }
    return result;
}

// Shared by this robot's four states. Simulation time freezes ramps on pause.
struct MotionContext
{
    explicit MotionContext(RL* robot) : rl(*robot) { Reload(); }

    void Reload()
    {
        acceleration = Limits("command_acceleration_limits", {1.0f, 0.3f, 0.8f});
        deceleration = Limits("command_deceleration_limits", {1.5f, 0.5f, 1.2f});
        getup_duration = Duration("getup_duration", 2.0f);
        getdown_duration = Duration("getdown_duration", 2.0f);
        transition_duration = Duration("policy_transition_duration", 0.5f);
        settle_duration = Duration("stop_settle_duration", 0.3f);
    }

    bool Tick(double time, bool running)
    {
        requested = {rl.control.x, rl.control.y, rl.control.yaw};
        const double nominal = rl.params.Get<float>("dt");
        const bool reset = std::isfinite(last_time) &&
            (time < last_time - 1e-6 || time - last_time > 4.0 * nominal);
        dt = running && std::isfinite(last_time) && !reset
            ? static_cast<float>(std::clamp(time - last_time, 0.0, nominal)) : 0.0f;
        last_time = time;
        return reset;
    }

    void ResetVelocity() { velocity.fill(0.0f); }

    void SetVelocity(bool stopping)
    {
        for (size_t i = 0; i < velocity.size(); ++i)
        {
            float target = stopping || !std::isfinite(requested[i]) ? 0.0f : requested[i];
            // Brake through zero before reversing. These rates refer to speed
            // magnitude, and therefore work for negative commands as well.
            const bool reversing = velocity[i] * target < 0.0f;
            const bool speeding_up = !reversing && std::abs(target) > std::abs(velocity[i]);
            const float step = (speeding_up ? acceleration[i] : deceleration[i]) * dt;
            if (reversing) target = 0.0f;
            velocity[i] += std::clamp(target - velocity[i], -step, step);
        }
        PublishVelocity();
    }

    void PublishVelocity()
    {
        rl.control.x = velocity[0];
        rl.control.y = velocity[1];
        rl.control.yaw = velocity[2];
    }

    bool Stopped() const
    {
        for (float value : velocity) if (std::abs(value) > 1e-4f) return false;
        for (int wheel : rl.params.Get<std::vector<int>>("wheel_indices"))
            if (!std::isfinite(rl.robot_state.motor_state.dq[wheel]) ||
                std::abs(rl.robot_state.motor_state.dq[wheel]) > 1.0f) return false;
        for (float value : rl.robot_state.imu.gyroscope)
            if (!std::isfinite(value) || std::abs(value) > 0.3f) return false;
        return true;
    }

    RL& rl;
    std::array<float, 3> velocity{}, requested{}, acceleration{}, deceleration{};
    float dt = 0.0f;
    float getup_duration = 2.0f, getdown_duration = 2.0f;
    float transition_duration = 0.5f, settle_duration = 0.3f;
    double last_time = std::numeric_limits<double>::quiet_NaN();

private:
    std::array<float, 3> Limits(const std::string& name, const std::array<float, 3>& defaults)
    {
        const auto values = rl.params.Get<std::vector<float>>(name,
            std::vector<float>(defaults.begin(), defaults.end()));
        if (values.size() != 3) throw std::runtime_error(name + " must contain vx, vy and wz");
        std::array<float, 3> result{};
        for (size_t i = 0; i < result.size(); ++i)
        {
            if (!std::isfinite(values[i]) || values[i] <= 0)
                throw std::runtime_error(name + " must be finite and positive");
            result[i] = values[i];
        }
        return result;
    }

    float Duration(const std::string& name, float fallback)
    {
        const float value = rl.params.Get<float>(name, fallback);
        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error(name + " must be finite and positive");
        return value;
    }
};

class State : public RLFSMState
{
public:
    State(RL* robot, const std::string& name, std::shared_ptr<MotionContext> motion)
        : RLFSMState(*robot, name), motion(std::move(motion)) {}

    virtual void PrepareInput()
    {
        motion->ResetVelocity();
        motion->PublishVelocity();
    }

    MotorCommand CurrentCommand() const
    {
        auto command = rl.robot_command.motor_command;
        const auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
        const auto kd = rl.params.Get<std::vector<float>>("fixed_kd");
        for (size_t i = 0; i < command.q.size(); ++i)
        {
            // A passive q target is not meaningful. Also handle --auto-policy
            // before the first passive control tick has initialized gains.
            if (command.kp[i] == 0.0f) command.q[i] = rl.robot_state.motor_state.q[i];
            if (command.kp[i] == 0.0f && command.kd[i] == 0.0f)
            {
                command.kp[i] = kp[i];
                command.kd[i] = kd[i];
            }
        }
        return command;
    }

    std::shared_ptr<MotionContext> motion;
};

class RLFSMStatePassive : public State
{
public:
    RLFSMStatePassive(RL* robot, std::shared_ptr<MotionContext> motion)
        : State(robot, "RLFSMStatePassive", std::move(motion)) {}

    void Enter() override
    {
        rl.rl_init_done = false;
        ClearOutputs(rl);
        motion->ResetVelocity();
        std::cout << LOGGER::NOTE << "ICE passive: press 0 to stand; P is immediate damping." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.kp[i] = 0.0f;
            fsm_command->motor_command.kd[i] = 8.0f;
            fsm_command->motor_command.tau[i] = 0.0f;
        }
    }

    void Exit() override {}
    std::string CheckChange() override
    {
        return Key(rl, Input::Keyboard::Num0, Input::Gamepad::A) ? "RLFSMStateGetUp" : state_name_;
    }
};

// Go2W's four-state structure, with Robocon CAD poses and quintic transitions.
class PoseState : public State
{
public:
    PoseState(RL* robot, const std::string& name, std::shared_ptr<MotionContext> motion, bool getting_up)
        : State(robot, name, std::move(motion)), getting_up(getting_up) {}

    void Enter() override
    {
        rl.rl_init_done = false;
        ClearOutputs(rl);
        motion->Reload();
        motion->ResetVelocity();
        elapsed = 0.0f;
        rl.now_state = *fsm_state;
        if (getting_up && (!rl.fsm.previous_state_ ||
            rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive"))
            rl.start_state = rl.now_state;
        start = CurrentCommand();
        target = start;
        target.q = getting_up ? rl.params.Get<std::vector<float>>("default_dof_pos")
                              : rl.start_state.motor_state.q;
        target.kp = rl.params.Get<std::vector<float>>("fixed_kp");
        target.kd = rl.params.Get<std::vector<float>>("fixed_kd");
        target.dq.assign(target.q.size(), 0.0f);
        target.tau.assign(target.q.size(), 0.0f);
        // Like Go2W, enable pose support gains immediately; position starts at
        // the previous target (or measured pose when entering from passive).
        start.kp = target.kp;
        start.kd = target.kd;
        start.tau = target.tau;
        if (!ValidCommand(start, target.q.size()) ||
            !ValidCommand(target, rl.params.Get<int>("num_of_dofs")))
            throw std::runtime_error("Invalid ICE pose transition");
        fsm_command->motor_command = start;
    }

    void Run() override
    {
        const float duration = getting_up ? motion->getup_duration : motion->getdown_duration;
        elapsed = std::min(duration, elapsed + motion->dt);
        fsm_command->motor_command = Blend(start, target, SmoothStep(elapsed / duration));
    }

    void Exit() override {}
    std::string CheckChange() override
    {
        if (Key(rl, Input::Keyboard::P, Input::Gamepad::LB_X)) return "RLFSMStatePassive";
        const float duration = getting_up ? motion->getup_duration : motion->getdown_duration;
        if (!getting_up)
        {
            if (Key(rl, Input::Keyboard::Num0, Input::Gamepad::A)) return "RLFSMStateGetUp";
            if (elapsed >= duration) return "RLFSMStatePassive";
        }
        else if (elapsed >= duration)
        {
            if (Key(rl, Input::Keyboard::Num1, Input::Gamepad::RB_DPadUp)) return "RLFSMStateRLLocomotion";
            if (Key(rl, Input::Keyboard::Num9, Input::Gamepad::B)) return "RLFSMStateGetDown";
        }
        return state_name_;
    }

private:
    bool getting_up;
    float elapsed = 0.0f;
    MotorCommand start, target;
};

class RLFSMStateGetUp : public PoseState
{
public:
    RLFSMStateGetUp(RL* robot, std::shared_ptr<MotionContext> motion)
        : PoseState(robot, "RLFSMStateGetUp", std::move(motion), true) {}
};

class RLFSMStateGetDown : public PoseState
{
public:
    RLFSMStateGetDown(RL* robot, std::shared_ptr<MotionContext> motion)
        : PoseState(robot, "RLFSMStateGetDown", std::move(motion), false) {}
};

class RLFSMStateRLLocomotion : public State
{
public:
    RLFSMStateRLLocomotion(RL* robot, std::shared_ptr<MotionContext> motion)
        : State(robot, "RLFSMStateRLLocomotion", std::move(motion)) {}

    void Enter() override
    {
        ready = false;
        have_policy = false;
        elapsed = settled = 0.0f;
        pending_exit.clear();
        last_episode = 0;
        start = CurrentCommand();
        if (!rl.fsm.previous_state_ || rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
            rl.start_state = *fsm_state;
        rl.rl_init_done = false;
        ClearOutputs(rl);
        try
        {
            rl.config_name = "robot_lab";
            rl.InitRL(rl.robot_name + "/" + rl.config_name);
            motion->Reload();
            motion->ResetVelocity();
            rl.episode_length_buf = 0;
            fsm_command->motor_command = start;
            target = start;
            ready = true;
            rl.rl_init_done = true;
            std::cout << LOGGER::NOTE << "ICE policy: smooth handoff, velocity ramps enabled; "
                      << "0/9 brake before changing pose, P requests damping." << std::endl;
        }
        catch (const std::exception& error)
        {
            std::cout << LOGGER::ERROR << "ICE policy initialization: " << error.what() << std::endl;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void PrepareInput() override
    {
        const auto previous_exit = pending_exit;
        if (Key(rl, Input::Keyboard::Num9, Input::Gamepad::B)) pending_exit = "RLFSMStateGetDown";
        else if (Key(rl, Input::Keyboard::Num0, Input::Gamepad::A)) pending_exit = "RLFSMStateGetUp";
        else if (Key(rl, Input::Keyboard::Num1, Input::Gamepad::RB_DPadUp)) pending_exit.clear();
        if (!pending_exit.empty() && pending_exit != previous_exit)
            std::cout << LOGGER::NOTE << "ICE slowing to rest before " << pending_exit << std::endl;
        motion->SetVelocity(!pending_exit.empty() || Key(rl, Input::Keyboard::P, Input::Gamepad::LB_X));
    }

    void Run() override
    {
        if (!ready) return;
        const bool waiting_for_first_policy = !have_policy;
        if (rl.episode_length_buf != last_episode)
        {
            // Keep an unblended target between policy updates. Only the
            // transition envelope is evaluated at every control tick.
            RLControl();
            target = fsm_command->motor_command;
            last_episode = rl.episode_length_buf;
            if (!ValidCommand(target, rl.params.Get<int>("num_of_dofs")))
            {
                ready = false;
                rl.rl_init_done = false;
                rl.fsm.RequestStateChange("RLFSMStatePassive");
                return;
            }
            have_policy = true;
        }
        if (have_policy)
        {
            if (!waiting_for_first_policy)
                elapsed = std::min(motion->transition_duration, elapsed + motion->dt);
            fsm_command->motor_command = Blend(start, target, SmoothStep(elapsed / motion->transition_duration));
        }
        else fsm_command->motor_command = start;

        settled = !pending_exit.empty() && motion->Stopped() ? settled + motion->dt : 0.0f;
    }

    void Exit() override
    {
        rl.rl_init_done = false;
        ClearOutputs(rl);
        motion->ResetVelocity();
    }

    std::string CheckChange() override
    {
        if (Key(rl, Input::Keyboard::P, Input::Gamepad::LB_X)) return "RLFSMStatePassive";
        if (!ready) return "RLFSMStatePassive";
        if (!pending_exit.empty() && settled >= motion->settle_duration) return pending_exit;
        return state_name_;
    }

private:
    bool ready = false, have_policy = false;
    float elapsed = 0.0f, settled = 0.0f;
    unsigned long long last_episode = 0;
    std::string pending_exit;
    MotorCommand start, target;
};

// Called after reading state/input and BEFORE inference. Robot-specific
// behavior stays here; the adapter does not change other robots' FSMs.
inline void PrepareControl(RL& rl, double simulation_time, bool running)
{
    auto* state = dynamic_cast<State*>(rl.fsm.current_state_.get());
    if (!state) return;
    state->fsm_state = &rl.robot_state;
    state->fsm_command = &rl.robot_command;
    if (state->motion->Tick(simulation_time, running))
    {
        ClearOutputs(rl);
        rl.start_state = rl.robot_state;
        auto& command = rl.robot_command.motor_command;
        command.q = rl.robot_state.motor_state.q;
        command.dq.assign(command.q.size(), 0.0f);
        command.kp = rl.params.Get<std::vector<float>>("fixed_kp");
        command.kd = rl.params.Get<std::vector<float>>("fixed_kd");
        command.tau.assign(command.q.size(), 0.0f);
        state->Enter();
    }
    state->PrepareInput();
}

} // namespace ice_robocon_fsm

class IceRoboconFSMFactory : public Go2WFSMFactory
{
public:
    using Go2WFSMFactory::Go2WFSMFactory;
    std::string GetType() const override { return "ice/robocon"; }

    std::shared_ptr<FSMState> CreateState(void* context, const std::string& name) override
    {
        using namespace ice_robocon_fsm;
        RL* robot = static_cast<RL*>(context);
        auto motion = contexts[robot].lock();
        if (!motion)
        {
            motion = std::make_shared<MotionContext>(robot);
            contexts[robot] = motion;
        }
        if (name == "RLFSMStatePassive") return std::make_shared<RLFSMStatePassive>(robot, motion);
        if (name == "RLFSMStateGetUp") return std::make_shared<RLFSMStateGetUp>(robot, motion);
        if (name == "RLFSMStateGetDown") return std::make_shared<RLFSMStateGetDown>(robot, motion);
        if (name == "RLFSMStateRLLocomotion") return std::make_shared<RLFSMStateRLLocomotion>(robot, motion);
        return nullptr;
    }

private:
    std::unordered_map<RL*, std::weak_ptr<ice_robocon_fsm::MotionContext>> contexts;
};

REGISTER_FSM_FACTORY(IceRoboconFSMFactory, "RLFSMStatePassive")

#endif // ICE_ROBOCON_FSM_HPP
