// Copyright 2023 ros2_control Development Team
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

#include "ros2_control_braccio/braccio_hardware.hpp"
#include <string>
#include <vector>

namespace ros2_control_braccio
{
CallbackReturn RobotSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  // setup communication with robot hardware
  ctrl_position_motor={0,0,0,0,0,0};
  zenoh::Config config = zenoh::Config::create_default();
  // zenoh::Session session = zenoh::Session::open(std::move(config));
  session = std::make_unique<zenoh::Session>(zenoh::Session::open(std::move(config)));

  // zenoh::Publisher publisher = session.declare_publisher(zenoh::KeyExpr("braccio_control/positions"));
  publisher = std::make_unique<zenoh::Publisher>(session->declare_publisher(zenoh::KeyExpr("braccio_control/positions")));
  // publisher.put(zenoh::ext::serialize(ctrl_position_motor));
  const std::vector<float> input = {0,0,0,0,0,0};
  publisher->put(zenoh::ext::serialize(input));
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  for (const auto & [name, descr] : sensor_state_interfaces_)
  {
    set_state(name, 0.0);
  }

  return CallbackReturn::SUCCESS;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    set_state(name_vel, get_command(name_vel));
    set_state(name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds());
  }
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

}  // namespace ros2_control_braccio

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_braccio::RobotSystem, hardware_interface::SystemInterface)
