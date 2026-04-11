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

#include "braccio_control/braccio_hardware.hpp"
#include <string>
#include <vector>

namespace braccio_control
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
  session = std::make_unique<zenoh::Session>(zenoh::Session::open(std::move(config)));
  publisher = std::make_unique<zenoh::Publisher>(session->declare_publisher(zenoh::KeyExpr("braccio_control/positions")));

  auto serializer = zenoh::ext::Serializer();
  serializer.serialize(ctrl_position_motor.rot1);
  serializer.serialize(ctrl_position_motor.arm1_up);
  serializer.serialize(ctrl_position_motor.arm2_up);
  serializer.serialize(ctrl_position_motor.arm3_up);
  serializer.serialize(ctrl_position_motor.rot_grasp);
  serializer.serialize(ctrl_position_motor.grasp);
  zenoh::Bytes bytes = std::move(serializer).finish();
  publisher->put(std::move(bytes));

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

  return CallbackReturn::SUCCESS;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++){
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    set_state(name_pos, get_command(name_pos));
    // std::cout<<name_pos<<get_command(name_pos)<<std::endl;
  }
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  // for (const auto & [name, descr] : joint_command_interfaces_){
  for (std::size_t i = 0; i < info_.joints.size(); i++){
    const auto name = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    // std::cout<<name<<get_command(name)<<std::endl;
    if (name == "arm_rot"){
      ctrl_position_motor.rot1=static_cast<float>(get_command(name));
    }else if(name == "arm_1_up_down"){
      ctrl_position_motor.arm1_up=static_cast<float> (get_command(name));
    }else if(name == "arm_2_up_down"){
      ctrl_position_motor.arm2_up=static_cast<float> (get_command(name));
    }else if(name == "arm_3_up_down"){
      ctrl_position_motor.arm3_up=static_cast<float> (get_command(name));
    }else if(name == "rot_grasp"){
      ctrl_position_motor.rot_grasp=static_cast<float> (get_command(name));
    }else if(name == "grasp"){
      ctrl_position_motor.grasp=static_cast<float> (get_command(name));
    }
  }
  auto serializer = zenoh::ext::Serializer();
  serializer.serialize(ctrl_position_motor.rot1);
  serializer.serialize(ctrl_position_motor.arm1_up);
  serializer.serialize(ctrl_position_motor.arm2_up);
  serializer.serialize(ctrl_position_motor.arm3_up);
  serializer.serialize(ctrl_position_motor.rot_grasp);
  serializer.serialize(ctrl_position_motor.grasp);
  zenoh::Bytes bytes = std::move(serializer).finish();
  publisher->put(std::move(bytes));
  return return_type::OK;
}

}  // namespace braccio_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  braccio_control::RobotSystem, hardware_interface::SystemInterface)
