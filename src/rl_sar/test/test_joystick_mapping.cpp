// SPDX-License-Identifier: Apache-2.0
#include "joystick.hh"
#include "joystick_mapping.hpp"
#include "velocity_input.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv)
{
    try
    {
        // Recorded maps from this machine's Xbox Bluetooth controller.
        std::vector<int> bluetooth_buttons;
        for (int code = 0x130; code <= 0x13e; ++code) bluetooth_buttons.push_back(code);
        JoystickMapping bluetooth({0, 1, 2, 5, 9, 10, 16, 17}, bluetooth_buttons);
        require(bluetooth.Button(0) == 0 && bluetooth.Button(1) == 1, "A/B mapping failed");
        require(bluetooth.Button(3) == 2 && bluetooth.Button(4) == 3, "Bluetooth X/Y mapping failed");
        require(bluetooth.Button(6) == 4 && bluetooth.Button(7) == 5, "Bluetooth LB/RB mapping failed");
        require(bluetooth.Button(13) == 9 && bluetooth.Button(14) == 10, "Stick button mapping failed");
        require(bluetooth.Axis(2) == 3 && bluetooth.Axis(3) == -1, "Yaw uses wrong right-stick axis");
        require(bluetooth.Axis(4) == -1 && bluetooth.Axis(5) == -1, "Triggers leaked into velocity");
        require(bluetooth.Axis(6) == 6 && bluetooth.Axis(7) == 7, "D-pad mapping failed");

        // Conventional xpad USB map, including slots absent in the BT report.
        JoystickMapping usb({0, 1, 2, 3, 4, 5, 16, 17},
            {0x130,0x131,0x133,0x134,0x136,0x137,0x13a,0x13b,0x13c,0x13d,0x13e});
        require(usb.Button(4) == 4 && usb.Button(5) == 5 && usb.Axis(3) == 3, "USB layout regressed");
        require(usb.Axis(2) == -1 && usb.Axis(5) == -1, "USB triggers leaked into yaw");
        require(usb.Button(200) == -1 && usb.Axis(200) == -1, "Unknown input not ignored");
        require(JoystickMapping({},{}).Axis(3) == 3, "Unavailable ioctl fallback changed");

        // Raw BT axis events must produce the same physical velocity as keyboard input.
        int axes[10]{};
        axes[bluetooth.Axis(1)] = -32768;
        axes[bluetooth.Axis(0)] = 32767;
        axes[bluetooth.Axis(2)] = -32768;
        VelocityInput input;
        input.SetLimits({-2,-.3f,-.8f},{2,.3f,.8f});
        input.SetJoystick({-VelocityInput::NormalizeJoystickAxis(axes[1]),
            -VelocityInput::NormalizeJoystickAxis(axes[0]), -VelocityInput::NormalizeJoystickAxis(axes[3])});
        const auto command = input.Command();
        require(command[0] == 2 && std::abs(command[1]+.3f) < 1e-6 && std::abs(command[2]-.8f) < 1e-6,
            "Raw Bluetooth events did not reach velocity commands");

        if (argc == 3 && std::string(argv[1]) == "--device")
        {
            Joystick device(argv[2]); require(device.isFound(), "Device could not be opened");
            const auto axis_codes = device.axisCodes(), button_codes = device.buttonCodes();
            require(!axis_codes.empty() && !button_codes.empty(), "Device ioctl maps could not be read");
            JoystickMapping actual(axis_codes,button_codes);
            std::cout << device.name() << ": LX=" << actual.axes[0] << " LY=" << actual.axes[1]
                      << " RX=" << actual.axes[3] << " LB=" << actual.buttons[4] << " RB=" << actual.buttons[5] << '\n';
            require(actual.axes[0] >= 0 && actual.axes[1] >= 0 && actual.axes[3] >= 0 &&
                    actual.buttons[4] >= 0 && actual.buttons[5] >= 0, "Device controls not mapped");
        }
        std::cout << "PASS: Xbox Bluetooth/USB buttons and sticks, trigger exclusion, raw axes to velocity, unknown inputs and ioctl fallback\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
