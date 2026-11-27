/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mujoco_joint_drive.hpp"
#include <cmath>
#include <iostream>
#include <memory>

void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

int main()
{
    try
    {
        const char* xml = R"(<mujoco>
          <option timestep="0.0025" integrator="implicitfast" gravity="0 0 0"/>
          <worldbody><body><joint name="wheel" axis="0 0 1"/>
            <inertial pos="0 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
          </body></worldbody>
          <actuator><motor joint="wheel" gear="1" ctrllimited="true" ctrlrange="-2 2"/></actuator>
        </mujoco>)";
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        mj_addBufferVFS(&vfs, "drive.xml", xml, std::char_traits<char>::length(xml));
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
            mj_loadXML("drive.xml", &vfs, error, sizeof(error)), mj_deleteModel);
        mj_deleteVFS(&vfs);
        require(bool(m), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);

        // Legacy explicit mode retains a gear=1 motor and the original formula.
        d->qpos[0] = 0.9;
        SetMujocoJointDrive(m.get(), d.get(), 0, 0.9, 0, 1, 0, 10, 0, 0, 2, false);
        mj_forward(m.get(), d.get());
        require(std::abs(d->actuator_force[0] - 1) < 1e-9, "Explicit drive changed");

        // The setpoint is 10, but the combined feedback force is only 1.
        SetMujocoJointDrive(m.get(), d.get(), 0, 0.9, 0, 1, 0, 10, 0, 0, 2, true);
        mj_forward(m.get(), d.get());
        require(std::abs(d->actuator_force[0] - 1) < 1e-9, "Setpoint was clipped before feedback");
        SetMujocoJointDrive(m.get(), d.get(), 0, 0.9, 0, 2, 0, 10, 0, 0, 2, true);
        mj_forward(m.get(), d.get());
        require(std::abs(d->actuator_force[0] - 2) < 1e-9, "Combined effort limit failed");

        // A light unloaded wheel must damp without alternating velocity signs.
        mj_resetData(m.get(), d.get());
        d->qvel[0] = 2;
        for (int step = 0; step < 400; ++step)
        {
            const double before = d->qvel[0];
            SetMujocoJointDrive(m.get(), d.get(), 0, d->qpos[0], before, 0, 0, 0, 0.5, 0, 2, true);
            mj_step(m.get(), d.get());
            require(d->qvel[0] >= -1e-10 && d->qvel[0] <= before + 1e-10, "Implicit wheel damping oscillated");
        }
        require(std::abs(d->qvel[0]) < 1e-8, "Wheel did not settle");
        std::cout << "PASS: explicit compatibility, implicit feedback/effort clipping and unloaded wheel damping\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
