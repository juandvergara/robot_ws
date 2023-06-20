// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_demo_example_1/visibility_control.h"

#include "ros2_control_demo_example_1/pico_scorbot.hpp"

namespace ros2_control_demo_example_1
{
    class RRBotSystemPositionOnlyHardware : public hardware_interface::SystemInterface
    {

        struct Config
        {
            std::string lower_device = "";
            std::string upper_device = "";
            int baud_rate = 0;
            int timeout_ms = 0;
            float k_gamma = 0;
            float mm_by_steps = 0;
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RRBotSystemPositionOnlyHardware);

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ROS2_CONTROL_DEMO_EXAMPLE_1_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        PicoScorbot scorbot_;
        Config cfg_;
        std::vector<double> hw_commands_;
        std::vector<double> hw_states_;
    };

} // namespace ros2_control_demo_example_1

#endif // ROS2_CONTROL_DEMO_EXAMPLE_1__RRBOT_HPP_
