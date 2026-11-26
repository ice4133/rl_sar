/*
 * Copyright (c) 2024-2026 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef RL_SAR_VELOCITY_INPUT_HPP
#define RL_SAR_VELOCITY_INPUT_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

// Callers synchronize access. Keyboard and joystick share normalized axes and
// physical command limits; no integration or key-repeat timing is involved.
class VelocityInput
{
public:
    enum Key { Forward, Reverse, Left, Right, TurnLeft, TurnRight, Stop, KeyCount };

    void SetLimits(const std::array<float, 3>& lower, const std::array<float, 3>& upper)
    {
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] > 0 || upper[i] < 0)
                throw std::runtime_error("Velocity limits must be finite with lower <= 0 <= upper");
        lower_ = lower;
        upper_ = upper;
    }

    void SetKey(Key key, bool pressed) { held_[key] = pressed; }
    void SetFocused(bool focused)
    {
        if (!focused) held_.fill(false);
    }
    void SetJoystick(const std::array<float, 3>& axes) { joystick_ = axes; }

    static float NormalizeJoystickAxis(int raw, float deadzone = 0.05f)
    {
        const float value = std::clamp(raw / (raw < 0 ? 32768.0f : 32767.0f), -1.0f, 1.0f);
        return std::abs(value) <= deadzone ? 0.0f : value;
    }

    std::array<float, 3> Command() const
    {
        auto axes = joystick_;
        // Held keyboard directions take priority; opposite keys cancel, even
        // when a joystick is also deflected. An idle joystick cannot erase keys.
        if (std::any_of(held_.begin(), held_.end(), [](bool value) { return value; }))
            axes = {float(held_[Forward]) - float(held_[Reverse]),
                    float(held_[Left]) - float(held_[Right]),
                    float(held_[TurnLeft]) - float(held_[TurnRight])};
        if (held_[Stop]) axes.fill(0.0f);
        for (int i = 0; i < 3; ++i)
        {
            const float value = std::isfinite(axes[i]) ? std::clamp(axes[i], -1.0f, 1.0f) : 0.0f;
            axes[i] = value * (value >= 0 ? upper_[i] : -lower_[i]);
        }
        return axes;
    }

private:
    std::array<bool, KeyCount> held_{};
    std::array<float, 3> joystick_{};
    std::array<float, 3> lower_{-1, -1, -1};
    std::array<float, 3> upper_{1, 1, 1};
};

#endif
