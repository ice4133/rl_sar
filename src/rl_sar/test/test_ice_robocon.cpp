/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

// Exercise the deployed LibTorch policy and RL SDK against MuJoCo without a GUI.
#include "rl_sdk.hpp"
#include <mujoco/mujoco.h>
#include <cmath>
#include <filesystem>
#include <stdexcept>

class TestController : public RL
{
public:
    std::vector<float> Forward() override { return model->forward({ComputeObservation()}); }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

void near(float actual, float expected, const std::string& message)
{
    require(std::abs(actual - expected) < 1e-4f, message);
}

int main()
{
    try
    {
        TestController rl;
        rl.ang_vel_axis = "body";
        rl.ReadYaml("ice/robocon", "base.yaml");
        rl.InitRL("ice/robocon/robot_lab");
        // Saturate processed targets while preserving the original action.
        rl.obs.actions.assign(16, 100.0f);
        std::vector<float> pos, vel, tau;
        rl.ComputeOutput(rl.obs.actions, pos, vel, tau);
        near(pos[0], 0.675f, "Hip target clipping failed");
        near(pos[1], 0.875f, "Right thigh offset/target clipping failed");
        near(pos[4], 1.375f, "Left thigh offset/target clipping failed");
        near(vel[12], -30.0f, "Right wheel sign/velocity clipping failed");
        near(vel[13], 30.0f, "Left wheel velocity clipping failed");
        near(rl.obs.actions[0], 100.0f, "Clipping changed the previous raw action");
        rl.obs.ang_vel = {400.0f, -400.0f, 0.0f};
        rl.obs.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
        auto observation = rl.ComputeObservation();
        require(observation.size() == 57, "Wrong observation dimension");
        near(observation[0], 25.0f, "Angular velocity clipping/scaling failed");
        near(observation[1], -25.0f, "Angular velocity clipping/scaling failed");
        near(observation[41], 100.0f, "Previous raw action observation changed");

        // Existing policies without the new target limits keep their outputs.
        TestController legacy;
        legacy.ReadYaml("go2w", "base.yaml");
        legacy.InitRL("go2w/robot_lab");
        legacy.ComputeOutput(std::vector<float>(16, 1.0f), pos, vel, tau);
        near(pos[0], 0.125f, "Legacy position output changed");
        near(pos[1], 0.75f, "Legacy default position changed");
        near(vel[12], 5.0f, "Legacy wheel output changed");

        const auto project = std::filesystem::path(POLICY_DIR).parent_path();
        const auto scene = project / "src/rl_sar_zoo/ice/robocon_description/mjcf/scene.xml";
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
            mj_loadXML(scene.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
        require(m != nullptr, error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
        require(m->nu == 16 && m->nsensordata == 55, "Wrong actuator or sensor count");
        const int key = mj_name2id(m.get(), mjOBJ_KEY, "home");
        require(key >= 0, "Missing home keyframe");
        mj_resetDataKeyframe(m.get(), d.get(), key);
        mj_forward(m.get(), d.get());
        const int base = mj_name2id(m.get(), mjOBJ_BODY, "base_link");
        auto kp = rl.params.Get<std::vector<float>>("rl_kp");
        auto kd = rl.params.Get<std::vector<float>>("rl_kd");
        auto names = rl.params.Get<std::vector<std::string>>("joint_names");
        for (int i = 0; i < 16; ++i)
        {
            require(m->actuator_trnid[2*i] == mj_name2id(m.get(), mjOBJ_JOINT, names[i].c_str()), "Actuator order mismatch");
            require(m->sensor_adr[i] == i && m->sensor_adr[16+i] == 16+i, "Sensor order mismatch");
        }
        rl.InitObservations();
        pos = rl.params.Get<std::vector<float>>("default_dof_pos");
        vel.assign(16, 0.0f);
        const int control_stride = std::lround(rl.params.Get<float>("dt") / m->opt.timestep);
        const int policy_stride = control_stride * rl.params.Get<int>("decimation");
        double min_height = 10.0, max_tilt = 0.0;
        const int steps = std::lround(22.0 / m->opt.timestep);
        for (int step = 0; step < steps; ++step)
        {
            if (step % policy_stride == 0)
            {
                for (int i = 0; i < 16; ++i)
                {
                    rl.obs.dof_pos[i] = d->sensordata[i];
                    rl.obs.dof_vel[i] = d->sensordata[16+i];
                }
                for (int i = 0; i < 4; ++i) rl.obs.base_quat[i] = d->sensordata[48+i];
                for (int i = 0; i < 3; ++i) rl.obs.ang_vel[i] = d->sensordata[52+i];
                const double t = d->time;
                rl.obs.commands = t < 7 ? std::vector<float>{0, 0, 0}
                    : t < 12 ? std::vector<float>{0.3f, 0, 0}
                    : t < 17 ? std::vector<float>{0, 0.1f, 0}
                    : std::vector<float>{0, 0, 0.2f};
                if (t >= 2.0)
                {
                    rl.obs.actions = rl.Forward();
                    require(rl.obs.actions.size() == 16, "Wrong policy output size");
                    for (float action : rl.obs.actions) require(std::isfinite(action), "Non-finite policy output");
                    rl.ComputeOutput(rl.obs.actions, pos, vel, tau);
                }
            }
            if (step % control_stride == 0)
            {
                for (int i = 0; i < 16; ++i)
                {
                    d->ctrl[i] = kp[i] * (pos[i] - d->sensordata[i])
                               + kd[i] * (vel[i] - d->sensordata[16+i]);
                }
            }
            mj_step(m.get(), d.get());
            const double height = d->xpos[3*base+2];
            const double tilt = std::acos(std::clamp(d->xmat[9*base+8], -1.0, 1.0));
            min_height = std::min(min_height, height);
            max_tilt = std::max(max_tilt, tilt);
            require(std::isfinite(height) && height > 0.18, "Robot fell during policy rollout at t=" + std::to_string(d->time));
            require(tilt < 1.0, "Robot tilted beyond the training termination threshold");
        }
        std::cout << "PASS: clipping, raw actions, legacy Go2W, LibTorch model and 22s MuJoCo rollout\n"
                  << "Minimum base height: " << min_height << " m; maximum tilt: " << max_tilt << " rad\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAIL: " << e.what() << std::endl;
        return 1;
    }
}
