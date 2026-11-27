/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RL_SAR_MUJOCO_JOINT_DRIVE_HPP
#define RL_SAR_MUJOCO_JOINT_DRIVE_HPP

#include <mujoco/mujoco.h>
#include <stdexcept>
#include <string>

inline bool UseImplicitMujocoDrive(const std::string& mode)
{
    if (mode == "implicit") return true;
    if (mode == "explicit") return false;
    throw std::runtime_error("mujoco_joint_drive must be explicit or implicit");
}

// For the generated gear=1 motor actuators. The affine bias lets MuJoCo's
// implicit integrator evaluate feedback at each physics step, as in training.
inline void SetMujocoJointDrive(mjModel* model, mjData* data, int actuator,
                               double position, double velocity,
                               double target_position, double target_velocity,
                               double kp, double kd, double feedforward,
                               double effort_limit, bool implicit)
{
    if (!implicit)
    {
        data->ctrl[actuator] = feedforward + kp * (target_position - position)
                                         + kd * (target_velocity - velocity);
        return;
    }
    if (model->opt.integrator != mjINT_IMPLICIT && model->opt.integrator != mjINT_IMPLICITFAST)
        throw std::runtime_error("Implicit joint drive requires the implicit or implicitfast integrator");
    if (effort_limit <= 0)
        throw std::runtime_error("Implicit joint drive requires a positive effort limit");
    model->actuator_biastype[actuator] = mjBIAS_AFFINE;
    model->actuator_biasprm[mjNBIAS * actuator] = 0;
    model->actuator_biasprm[mjNBIAS * actuator + 1] = -kp;
    model->actuator_biasprm[mjNBIAS * actuator + 2] = -kd;
    // Limit the combined feedback + feedforward force, not the setpoint before
    // subtracting feedback. This also handles changes between FSM states.
    model->actuator_ctrllimited[actuator] = false;
    model->actuator_forcelimited[actuator] = true;
    model->actuator_forcerange[2 * actuator] = -effort_limit;
    model->actuator_forcerange[2 * actuator + 1] = effort_limit;
    data->ctrl[actuator] = feedforward + kp * target_position + kd * target_velocity;
}

#endif
