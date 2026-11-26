/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"
#include "velocity_input.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <memory>
#include <deque>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Button
{
public:
    Button() {}

    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class RL_Sim : public RL
{
public:
    RL_Sim(int argc, char **argv);
    ~RL_Sim();

    std::unique_ptr<mj::Simulate> sim;
    static RL_Sim* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    void ResetSimulation();
    void PhysicsControl();
    bool WindowKey(int key, int action);
    void UpdateVelocityCommand();
    VelocityInput velocity_input;
    std::deque<Input::Keyboard> window_keys;
    bool physics_sync_control = false;
    bool policy_step_due = false;
    double next_control_time = 0.0;
    long long control_tick = 0;
    std::ofstream telemetry;

    // loop
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // mujoco
    mjData *mj_data;
    mjModel *mj_model;
    std::thread physics_thread;
    std::string scene_name;

    // joystick
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    float axis_deadzone = 0.05f;
    void SetupSysJoystick(const std::string& device);
    void GetSysJoystick();

    // others
    std::string gazebo_model_name;
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;
    void StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names);
};

#endif // RL_SIM_HPP
