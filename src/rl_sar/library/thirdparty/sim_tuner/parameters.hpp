// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

namespace sim_tuner {
constexpr int kProtocol = 1;
constexpr int kDefaultPort = 9870;

struct Parameter {
    const char* key;
    const char* label;
    const char* group;
    const char* unit;
    double initial, minimum, maximum, step;
    bool reset;
    const char* help;
};

// Shared by the panel and simulator; multipliers always refer to the loaded model.
inline constexpr std::array<Parameter, 24> kParameters{{
    {"friction_override", "覆盖滑动摩擦", "接触", "0/1", 0, 0, 1, 1, false,
     "开启后统一设置碰撞 geom 和显式接触 pair 的滑动摩擦；关闭后恢复原模型。"},
    {"sliding_friction", "滑动摩擦系数", "接触", "", .6, 0, 3, .01, false,
     "需开启覆盖滑动摩擦。轮子和地面同时修改，避免两者取较大值的影响。"},
    {"contact_time_scale", "接触时间常数倍率", "接触", "×", 1, .2, 5, .05, false,
     "仅缩放正值 solref 的时间常数。直接刚度格式保持原值；MuJoCo 仍可能按步长限制时间常数。"},
    {"delay_ms", "等效指令延迟", "驱动", "ms", 0, 0, 500, 1, false,
     "完整电机目标在仿真时间中延迟执行。精度受物理步长限制；修改延迟会清空在途指令，保留已执行目标。"},
    {"motor_strength", "电机强度", "驱动", "×", 1, 0, 2, .01, false,
     "同时缩放 PD 反馈和前馈，之后再限幅；不改变最大力矩设置。"},
    {"torque_limit_scale", "力矩上限倍率", "驱动", "×", 1, .1, 2, .01, false,
     "相对 YAML 力矩上限缩放，仍受模型关节 actuatorfrcrange 的硬限制。"},
    {"kp_scale", "位置增益 Kp", "驱动", "×", 1, 0, 3, .01, false,
     "缩放当前 FSM 发出的 Kp，包括站立和策略状态。"},
    {"kd_scale", "速度增益 Kd", "驱动", "×", 1, 0, 3, .01, false,
     "缩放当前 FSM 发出的 Kd，包括站立和策略状态。"},
    {"joint_damping_add", "关节附加阻尼", "驱动", "N·m·s/rad", 0, 0, 5, .01, false,
     "在机器人转动关节的原始被动阻尼上增加，不影响浮动基座。"},
    {"joint_friction_add", "关节附加摩擦", "驱动", "N·m", 0, 0, 3, .01, false,
     "在机器人转动关节的原始 frictionloss 上增加。"},
    {"joint_pos_noise", "关节位置噪声 σ", "观测", "rad", 0, 0, .1, .001, false,
     "独立零均值高斯噪声，仅作用于控制器读到的状态，不修改物理状态。"},
    {"joint_vel_noise", "关节速度噪声 σ", "观测", "rad/s", 0, 0, 2, .01, false,
     "独立零均值高斯噪声；噪声采样频率为控制状态读取频率。"},
    {"gyro_noise", "陀螺仪噪声 σ", "观测", "rad/s", 0, 0, 1, .001, false,
     "三个轴独立的零均值高斯噪声。"},
    {"gyro_bias_x", "陀螺仪 X 偏置", "观测", "rad/s", 0, -.5, .5, .001, false, "固定 X 轴偏置。"},
    {"gyro_bias_y", "陀螺仪 Y 偏置", "观测", "rad/s", 0, -.5, .5, .001, false, "固定 Y 轴偏置。"},
    {"gyro_bias_z", "陀螺仪 Z 偏置", "观测", "rad/s", 0, -.5, .5, .001, false, "固定 Z 轴偏置。"},
    {"noise_seed", "噪声随机种子", "观测", "", 42, 0, 1000000, 1, false,
     "整数。修改或复位时重置随机序列；相同控制采样序列可复现。"},
    {"mass_scale", "机器人质量倍率", "模型", "×", 1, .2, 3, .01, true,
     "缩放机器人子树的质量，并按同一倍率缩放惯量，需应用并复位。"},
    {"inertia_scale", "额外惯量倍率", "模型", "×", 1, .2, 3, .01, true,
     "在质量倍率之外额外缩放三项主惯量，需应用并复位。"},
    {"com_x", "基座重心 X 偏移", "模型", "m", 0, -.15, .15, .001, true, "基座局部坐标系偏移，需应用并复位。"},
    {"com_y", "基座重心 Y 偏移", "模型", "m", 0, -.15, .15, .001, true, "基座局部坐标系偏移，需应用并复位。"},
    {"com_z", "基座重心 Z 偏移", "模型", "m", 0, -.15, .15, .001, true, "基座局部坐标系偏移，需应用并复位。"},
    {"armature_add", "关节附加转子惯量", "模型", "kg·m²", 0, 0, .1, .001, true,
     "在机器人转动关节的原始 armature 上增加，需应用并复位。"},
    {"gravity_scale", "重力倍率", "模型", "×", 1, 0, 2, .01, false, "缩放原模型的重力向量。"}
}};

using Values = std::map<std::string, double>;
inline Values Defaults() {
    Values result;
    for (const auto& p : kParameters) result[p.key] = p.initial;
    return result;
}
inline const Parameter& Spec(const std::string& key) {
    for (const auto& p : kParameters) if (key == p.key) return p;
    throw std::runtime_error("未知参数: " + key);
}
inline void Validate(const std::string& key, double value) {
    const auto& p = Spec(key);
    if (!std::isfinite(value) || value < p.minimum || value > p.maximum)
        throw std::runtime_error("参数超出范围: " + key);
    if ((key == "noise_seed" || key == "friction_override") && value != std::floor(value))
        throw std::runtime_error("参数必须为整数: " + key);
}
}  // namespace sim_tuner
