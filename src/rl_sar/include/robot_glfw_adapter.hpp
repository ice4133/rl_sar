/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RL_SAR_ROBOT_GLFW_ADAPTER_HPP
#define RL_SAR_ROBOT_GLFW_ADAPTER_HPP

#include <functional>
#include "glfw_adapter.h"

// Extend the existing UI without replacing its GLFW callbacks/user pointer.
// GLFW APIs stay on the render/main thread. Robot input handlers acquire sim.mtx.
class RobotGlfwAdapter : public mujoco::GlfwAdapter
{
public:
    std::function<bool(int, int)> robot_key;
    std::function<void(bool)> robot_focus;

    void PollEvents() override
    {
        mujoco::GlfwAdapter::PollEvents();
        if (robot_focus)
        {
            GLFWwindow* window = glfwGetCurrentContext();
            robot_focus(window && glfwGetWindowAttrib(window, GLFW_FOCUSED) == GLFW_TRUE);
        }
    }

protected:
    void OnKey(int key, int scancode, int action) override
    {
        if (robot_key && robot_key(key, action)) return;
        mujoco::GlfwAdapter::OnKey(key, scancode, action);
    }
};

#endif
