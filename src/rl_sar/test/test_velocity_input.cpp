/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#include "velocity_input.hpp"
#include <iostream>
#include <limits>

void check(const std::array<float, 3>& actual, const std::array<float, 3>& expected)
{
    for (int i = 0; i < 3; ++i)
        if (std::abs(actual[i] - expected[i]) > 1e-5f)
            throw std::runtime_error("Unexpected command on axis " + std::to_string(i));
}

int main()
{
    try
    {
        VelocityInput input;
        input.SetLimits({-2, -0.3f, -0.8f}, {2, 0.3f, 0.8f});
        input.SetKey(VelocityInput::Forward, true);
        check(input.Command(), {2, 0, 0});
        input.SetKey(VelocityInput::Forward, true); // OS repeat must not accumulate.
        check(input.Command(), {2, 0, 0});
        input.SetKey(VelocityInput::TurnLeft, true);
        check(input.Command(), {2, 0, 0.8f});
        input.SetKey(VelocityInput::Forward, false);
        check(input.Command(), {0, 0, 0.8f});
        input.SetKey(VelocityInput::TurnLeft, false);
        check(input.Command(), {0, 0, 0});
        input.SetKey(VelocityInput::Reverse, true);
        input.SetKey(VelocityInput::Right, true);
        input.SetKey(VelocityInput::TurnRight, true);
        check(input.Command(), {-2, -0.3f, -0.8f});
        input.SetKey(VelocityInput::Forward, true);
        check(input.Command(), {0, -0.3f, -0.8f});
        input.SetFocused(false); // Losing focus clears held keys.
        input.SetFocused(true);
        check(input.Command(), {0, 0, 0});

        input.SetJoystick({0.5f, -0.5f, 1});
        check(input.Command(), {1, -0.15f, 0.8f});
        input.SetJoystick({1, 1, 1});
        check(input.Command(), {2, 0.3f, 0.8f});
        input.SetKey(VelocityInput::Forward, true);
        input.SetJoystick({0, 0, 0}); // Idle polling must not erase a held key.
        check(input.Command(), {2, 0, 0});
        input.SetKey(VelocityInput::Stop, true);
        check(input.Command(), {0, 0, 0});
        input.SetFocused(false);
        input.SetJoystick({0, 0, 0});
        check(input.Command(), {0, 0, 0});

        if (VelocityInput::NormalizeJoystickAxis(-32768) != -1 ||
            VelocityInput::NormalizeJoystickAxis(32767) != 1 ||
            VelocityInput::NormalizeJoystickAxis(1000) != 0)
            throw std::runtime_error("Joystick endpoints/deadzone differ from keyboard axes");

        input.SetLimits({-0.4f, -0.2f, -0.5f}, {0.7f, 0.3f, 0.6f});
        input.SetJoystick({-1, -1, -1});
        check(input.Command(), {-0.4f, -0.2f, -0.5f});
        input.SetJoystick({1, 1, 1});
        check(input.Command(), {0.7f, 0.3f, 0.6f});
        input.SetJoystick({2, -2, std::numeric_limits<float>::quiet_NaN()});
        check(input.Command(), {0.7f, -0.2f, 0});
        input.SetLimits({-0.4f, 0, -0.5f}, {0.7f, 0, 0.6f});
        input.SetJoystick({1, 1, 1});
        check(input.Command(), {0.7f, 0, 0.6f});

        bool rejected = false;
        try { input.SetLimits({0.1f, -1, -1}, {1, 1, 1}); }
        catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Invalid limits were accepted");
        std::cout << "PASS: press/release, repeat, combined/opposite keys, focus loss, joystick scaling/deadzone and input priority\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
