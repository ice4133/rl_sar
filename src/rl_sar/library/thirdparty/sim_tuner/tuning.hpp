// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "parameters.hpp"
#include <mujoco/mujoco.h>
#include <deque>
#include <functional>
#include <memory>
#include <random>
#include <vector>

namespace sim_tuner {
struct Command {
    std::vector<double> q, dq, kp, kd, tau, limits;
};

// All methods must run under the simulator mutex. This class has no network I/O.
class Tuning {
public:
    Tuning(mjModel* model, mjData* data, std::vector<int> mapping,
           std::vector<std::string> names, std::string robot, std::string drive,
           const std::string& base_body);
    const Values& values() const { return values_; }
    void Submit(Command command);
    void Tick();
    void Reset();
    void Perturb(std::vector<float>& q, std::vector<float>& dq, std::vector<float>& gyro);
    std::string Handle(const std::string& request, bool running, const std::function<void()>& reset);
    std::string Snapshot(long long id, bool running) const;
    size_t queued() const { return queue_.size(); }
private:
    void Apply(const Values& next);
    double Noise(double sigma);
    struct TimedCommand { double sent, due; Command command; };
    mjModel* m_;
    mjData* d_;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> scratch_;
    std::vector<int> mapping_;
    std::vector<std::string> names_;
    std::string robot_, drive_;
    int base_ = 1;
    std::vector<bool> robot_bodies_;
    std::vector<double> friction_, pair_friction_, solref_, pair_solref_, damping_, frictionloss_,
        mass_, inertia_, ipos_, armature_, gravity_;
    Values values_ = Defaults();
    std::deque<TimedCommand> queue_;
    Command active_;
    bool active_valid_ = false;
    double last_tick_ = -1, applied_at_ = 0;
    unsigned long long version_ = 0;
    std::mt19937 random_{42};
    std::normal_distribution<double> gaussian_{0, 1};
};
}  // namespace sim_tuner
