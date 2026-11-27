// SPDX-License-Identifier: Apache-2.0
#include "tuning.hpp"
#include "mujoco_joint_drive.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>

namespace sim_tuner {
namespace {
std::string Quote(const std::string& text) {
    std::ostringstream out; out << '"';
    for (unsigned char c : text) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << c;
    }
    return out.str() + '"';
}
double Finite(double v) { return std::isfinite(v) ? v : 0; }
template <class T> void Array(std::ostream& out, const std::vector<T>& data) {
    out << '[';
    for (size_t i = 0; i < data.size(); ++i) { if (i) out << ','; out << Finite(data[i]); }
    out << ']';
}
std::vector<double> Copy(const mjtNum* ptr, int size) { return {ptr, ptr + size}; }
}
Tuning::Tuning(mjModel* model, mjData* data, std::vector<int> mapping,
               std::vector<std::string> names, std::string robot, std::string drive,
               const std::string& base_body)
    : m_(model), d_(data), scratch_(mj_makeData(model), mj_deleteData), mapping_(std::move(mapping)),
      names_(std::move(names)), robot_(std::move(robot)), drive_(std::move(drive)),
      robot_bodies_(model->nbody, false),
      friction_(Copy(model->geom_friction, 3 * model->ngeom)),
      pair_friction_(Copy(model->pair_friction, 5 * model->npair)),
      solref_(Copy(model->geom_solref, mjNREF * model->ngeom)),
      pair_solref_(Copy(model->pair_solref, mjNREF * model->npair)),
      damping_(Copy(model->dof_damping, model->nv)), frictionloss_(Copy(model->dof_frictionloss, model->nv)),
      mass_(Copy(model->body_mass, model->nbody)), inertia_(Copy(model->body_inertia, 3 * model->nbody)),
      ipos_(Copy(model->body_ipos, 3 * model->nbody)), armature_(Copy(model->dof_armature, model->nv)),
      gravity_(Copy(model->opt.gravity, 3)) {
    if (!scratch_) throw std::runtime_error("无法分配调参模型数据");
    const int named = mj_name2id(m_, mjOBJ_BODY, base_body.c_str());
    if (named > 0) base_ = m_->body_rootid[named];
    if (base_ <= 0 || base_ >= m_->nbody) throw std::runtime_error("无法找到机器人基座");
    for (int b = 1; b < m_->nbody; ++b) robot_bodies_[b] = m_->body_rootid[b] == base_;
    for (int actuator : mapping_) {
        if (actuator < 0 || actuator >= m_->nu || actuator >= static_cast<int>(mapping_.size()))
            throw std::runtime_error("调参器的 joint_mapping 与执行器/传感器不匹配");
    }
    if (m_->nsensordata < 3 * static_cast<int>(mapping_.size()))
        throw std::runtime_error("调参器需要位置、速度和力矩传感器");
    if (names_.size() != mapping_.size()) {
        names_.clear();
        for (size_t i = 0; i < mapping_.size(); ++i) names_.push_back("joint_" + std::to_string(i));
    }
}
void Tuning::Apply(const Values& next) {
    bool constants_changed = false;
    for (const auto& p : kParameters)
        if (p.reset && next.at(p.key) != values_.at(p.key)) constants_changed = true;
    for (int g = 0; g < m_->ngeom; ++g) {
        const bool collision = m_->geom_contype[g] || m_->geom_conaffinity[g];
        m_->geom_friction[3*g] = next.at("friction_override") && collision
            ? next.at("sliding_friction") : friction_[3*g];
        if (solref_[mjNREF*g] > 0)
            m_->geom_solref[mjNREF*g] = solref_[mjNREF*g] * next.at("contact_time_scale");
    }
    for (int p = 0; p < m_->npair; ++p) {
        for (int axis = 0; axis < 2; ++axis)
            m_->pair_friction[5*p+axis] = next.at("friction_override")
                ? next.at("sliding_friction") : pair_friction_[5*p+axis];
        if (pair_solref_[mjNREF*p] > 0)
            m_->pair_solref[mjNREF*p] = pair_solref_[mjNREF*p] * next.at("contact_time_scale");
    }
    for (int j = 0; j < m_->njnt; ++j) {
        if (!robot_bodies_[m_->jnt_bodyid[j]] || m_->jnt_type[j] != mjJNT_HINGE) continue;
        int dof = m_->jnt_dofadr[j];
        m_->dof_damping[dof] = damping_[dof] + next.at("joint_damping_add");
        m_->dof_frictionloss[dof] = frictionloss_[dof] + next.at("joint_friction_add");
        m_->dof_armature[dof] = armature_[dof] + next.at("armature_add");
    }
    for (int b = 1; b < m_->nbody; ++b) if (robot_bodies_[b]) {
        m_->body_mass[b] = mass_[b] * next.at("mass_scale");
        for (int axis = 0; axis < 3; ++axis)
            m_->body_inertia[3*b+axis] = inertia_[3*b+axis] * next.at("mass_scale") * next.at("inertia_scale");
    }
    for (int axis = 0; axis < 3; ++axis) {
        m_->body_ipos[3*base_+axis] = ipos_[3*base_+axis] + next.at(axis == 0 ? "com_x" : axis == 1 ? "com_y" : "com_z");
        m_->opt.gravity[axis] = gravity_[axis] * next.at("gravity_scale");
    }
    if (next.at("delay_ms") != values_.at("delay_ms")) queue_.clear();
    if (next.at("noise_seed") != values_.at("noise_seed")) {
        random_.seed(static_cast<unsigned>(next.at("noise_seed"))); gaussian_.reset();
    }
    values_ = next;
    // Use separate mjData: mj_setConst evaluates qpos0 and may overwrite scratch state.
    if (constants_changed) mj_setConst(m_, scratch_.get());
    ++version_; applied_at_ = d_->time;
}
void Tuning::Submit(Command command) {
    const size_t n = mapping_.size();
    if (command.q.size() != n || command.dq.size() != n || command.kp.size() != n ||
        command.kd.size() != n || command.tau.size() != n || command.limits.size() != n)
        throw std::runtime_error("调参指令维度错误");
    // Paused control loops must not accumulate commands at an unchanged simulation time.
    if (!queue_.empty() && std::abs(queue_.back().sent - d_->time) < 1e-10) queue_.pop_back();
    queue_.push_back({d_->time, d_->time + values_.at("delay_ms") * .001, std::move(command)});
    if (queue_.size() > 8192) throw std::runtime_error("调参延迟队列超出容量");
}
void Tuning::Tick() {
    if (last_tick_ >= 0 && d_->time < last_tick_ - 1e-10) Reset();
    last_tick_ = d_->time;
    while (!queue_.empty() && queue_.front().due <= d_->time + 1e-10) {
        active_ = std::move(queue_.front().command); queue_.pop_front(); active_valid_ = true;
    }
    if (!active_valid_) {
        // Before the first delayed packet arrives, no motor command has been received.
        for (int actuator : mapping_) {
            d_->ctrl[actuator] = 0;
            m_->actuator_biasprm[mjNBIAS*actuator+1] = 0;
            m_->actuator_biasprm[mjNBIAS*actuator+2] = 0;
        }
        return;
    }
    const double strength = values_.at("motor_strength");
    const bool implicit = UseImplicitMujocoDrive(drive_);
    for (size_t i = 0; i < mapping_.size(); ++i) {
        int a = mapping_[i];
        double limit = active_.limits[i] * values_.at("torque_limit_scale");
        SetMujocoJointDrive(m_, d_, a, d_->sensordata[a], d_->sensordata[a+mapping_.size()],
            active_.q[i], active_.dq[i], active_.kp[i]*values_.at("kp_scale")*strength,
            active_.kd[i]*values_.at("kd_scale")*strength, active_.tau[i]*strength, limit, implicit);
        if (!implicit) d_->ctrl[a] = std::clamp(double(d_->ctrl[a]), -limit, limit);
    }
}
void Tuning::Reset() {
    queue_.clear(); active_ = Command{}; active_valid_ = false; last_tick_ = -1;
    random_.seed(static_cast<unsigned>(values_.at("noise_seed"))); gaussian_.reset();
}
double Tuning::Noise(double sigma) { return sigma == 0 ? 0 : gaussian_(random_) * sigma; }
void Tuning::Perturb(std::vector<float>& q, std::vector<float>& dq, std::vector<float>& gyro) {
    for (auto& value : q) value += Noise(values_.at("joint_pos_noise"));
    for (auto& value : dq) value += Noise(values_.at("joint_vel_noise"));
    for (size_t i = 0; i < std::min<size_t>(3, gyro.size()); ++i)
        gyro[i] += Noise(values_.at("gyro_noise")) + values_.at(i == 0 ? "gyro_bias_x" : i == 1 ? "gyro_bias_y" : "gyro_bias_z");
}
std::string Tuning::Handle(const std::string& request, bool running, const std::function<void()>& reset) {
    long long id = -1;
    try {
        const auto node = YAML::Load(request);
        if (!node.IsMap()) throw std::runtime_error("请求必须为 JSON 对象");
        id = node["id"].as<long long>();
        if (node["protocol"].as<int>() != kProtocol) throw std::runtime_error("调参协议版本不一致");
        const std::string op = node["op"].as<std::string>();
        if (op == "status") return Snapshot(id, running);
        if (op != "set" && op != "defaults") throw std::runtime_error("未知操作");
        Values next = op == "defaults" ? Defaults() : values_;
        if (op == "set") {
            auto patch = node["params"];
            if (!patch.IsMap() || patch.size() > kParameters.size()) throw std::runtime_error("params 必须是参数对象");
            for (auto it : patch) {
                auto key = it.first.as<std::string>();
                double value = it.second.as<double>(); Validate(key, value); next[key] = value;
            }
        }
        bool needs_reset = false;
        for (const auto& p : kParameters) if (p.reset && next.at(p.key) != values_.at(p.key)) needs_reset = true;
        const bool do_reset = node["reset"] && node["reset"].as<bool>();
        if (needs_reset && !do_reset) throw std::runtime_error("模型参数修改需要使用“应用并复位”");
        Apply(next);
        if (do_reset) reset();
        applied_at_ = d_->time;
        return Snapshot(id, running);
    } catch (const std::exception& error) {
        return "{\"protocol\":1,\"id\":" + std::to_string(id) + ",\"ok\":false,\"error\":" + Quote(error.what()) + "}";
    }
}
std::string Tuning::Snapshot(long long id, bool running) const {
    std::ostringstream out; out << std::setprecision(12);
    out << "{\"protocol\":1,\"id\":" << id << ",\"ok\":true,\"robot\":" << Quote(robot_)
        << ",\"drive\":" << Quote(drive_) << ",\"version\":" << version_
        << ",\"applied_at\":" << Finite(applied_at_) << ",\"sim_time\":" << Finite(d_->time)
        << ",\"running\":" << (running ? "true" : "false") << ",\"timestep_ms\":" << m_->opt.timestep*1000
        << ",\"queued\":" << queue_.size() << ",\"params\":{";
    bool comma = false;
    for (const auto& entry : values_) {
        if (comma) out << ','; comma = true;
        out << Quote(entry.first) << ':' << entry.second;
    }
    double mass = 0, friction_min = std::numeric_limits<double>::infinity(), friction_max = 0;
    for (int b = 1; b < m_->nbody; ++b) if (robot_bodies_[b]) mass += m_->body_mass[b];
    for (int g = 0; g < m_->ngeom; ++g) if (m_->geom_contype[g] || m_->geom_conaffinity[g]) {
        friction_min = std::min(friction_min, double(m_->geom_friction[3*g]));
        friction_max = std::max(friction_max, double(m_->geom_friction[3*g]));
    }
    out << "},\"mass\":" << mass << ",\"friction_min\":" << Finite(friction_min)
        << ",\"friction_max\":" << friction_max << ",\"height\":" << Finite(d_->xpos[3*base_+2])
        << ",\"joint_names\":[";
    for (size_t i = 0; i < names_.size(); ++i) { if (i) out << ','; out << Quote(names_[i]); }
    std::vector<double> q, dq, tau, target_q, target_dq;
    for (size_t i = 0; i < mapping_.size(); ++i) {
        int a = mapping_[i]; q.push_back(d_->sensordata[a]); dq.push_back(d_->sensordata[a+mapping_.size()]);
        tau.push_back(d_->sensordata[a+2*mapping_.size()]);
        target_q.push_back(active_valid_ ? active_.q[i] : q.back());
        target_dq.push_back(active_valid_ ? active_.dq[i] : 0);
    }
    out << "],\"q\":"; Array(out, q);
    out << ",\"dq\":"; Array(out, dq);
    out << ",\"tau\":"; Array(out, tau);
    out << ",\"target_q\":"; Array(out, target_q);
    out << ",\"target_dq\":"; Array(out, target_dq);
    out << '}'; return out.str();
}
}  // namespace sim_tuner
