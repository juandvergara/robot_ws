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

#include "ros2_control_demo_example_1/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_1
{
  hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.upper_device = info_.hardware_parameters["upper_device"];
    cfg_.lower_device = info_.hardware_parameters["lower_device"];
    cfg_.baud_rate = stod(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = stod(info_.hardware_parameters["timeout_ms"]);
    cfg_.k_gamma = stof(info_.hardware_parameters["k_gamma"]);
    cfg_.mm_by_steps = stof(info_.hardware_parameters["mm_by_steps"]);

    // END: This part here is for exemplary purposes - Please do not copy to your production code
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // RRBotSystemPositionOnly has exactly one state and command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
            "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Configuring ...please wait...");

    // reset values always when configuring hardware
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  RRBotSystemPositionOnlyHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  RRBotSystemPositionOnlyHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Activating ...please wait...");

    scorbot_.connect(cfg_.upper_device, cfg_.lower_device, cfg_.baud_rate, cfg_.timeout_ms);
    scorbot_.ConfigExtruder(cfg_.mm_by_steps);
    scorbot_.getDataFromDevices(hw_states_);

    rclcpp::sleep_for(std::chrono::nanoseconds(100000));
    
    // command and state should be equal when starting
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
    }

    rclcpp::sleep_for(std::chrono::nanoseconds(100000));
    scorbot_.configKGammaGain(cfg_.k_gamma);
    rclcpp::sleep_for(std::chrono::nanoseconds(100000));

    RCLCPP_WARN(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Encoder left %s, encoder right %s.", std::to_string(hw_commands_[4]).c_str(), std::to_string(hw_commands_[5]).c_str());
    // hw_commands_[4] = (-hw_states_[4] + hw_states_[5] - hw_states_[3] - hw_states_[2]) * 180.0 / PI;
    // hw_commands_[5] = (hw_states_[4] + hw_states_[5] + hw_states_[3] + hw_states_[2]) * 180.0 / PI;

    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Deactivating ...please wait...");

    scorbot_.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RRBotSystemPositionOnlyHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Reading...");

    scorbot_.getDataFromDevices(hw_states_);

    // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully read!");

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RRBotSystemPositionOnlyHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    // RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Writing...");

    scorbot_.sendDataToDevices(hw_commands_);

    // RCLCPP_INFO(
    // rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_demo_example_1::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)
