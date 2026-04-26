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

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <chrono>
#include <thread>

#include <sensor_msgs/msg/joy.hpp>

struct vel_commands_struct{
  double x,y,z;
  double wx,wy,wz;
};
struct pos_commands_struct{
  double rz,up1,up2,up3,gr,gc;
};

std::vector<std::string> joint_names_ = {"arm_rot", "arm_1_up_down", "arm_2_up_down", "arm_3_up_down", "rot_grasp", "grasp_ctrl"};

pos_commands_struct pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

class braccio_control_joy : public rclcpp::Node
{
public:
  braccio_control_joy()
  : Node("braccio_control_joy")
  {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/braccio_controller/joint_trajectory", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
      std::bind(&braccio_control_joy::convertInputJoy, this, std::placeholders::_1)
      );
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&braccio_control_joy::joint_callback, this, std::placeholders::_1));
    
    auto robot_param = rclcpp::Parameter();
    this->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
    this->get_parameter("robot_description", robot_param);
    auto robot_description = robot_param.as_string();
    kdl_parser::treeFromString(robot_description, robot_tree);
    // res = robot_tree.getChain("base_servo_ud", "tool01", chain);
    res = robot_tree.getChain("base_servo_ud", "tool01", chain);
    if (res==false){
      throw;
    }
    joint_positions = KDL::JntArray(chain.getNrOfJoints());
    joint_velocities = KDL::JntArray(chain.getNrOfJoints());
    twist = KDL::Twist();
    // create KDL solvers
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain);
    if (ik_vel_solver_==nullptr){
      throw;
    }
    // trajectory_msgs::msg::JointTrajectory trajectory_msg;
    // for (unsigned int i = 0; i < chain.getNrOfSegments(); i++){
    //   auto joint = chain.getSegment(i).getJoint();
    //   if (joint.getType() != KDL::Joint::Fixed){
    //     trajectory_msg.joint_names.push_back(joint.getName());
    //   }
    // }
    // trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
    // trajectory_point_msg.positions.resize(chain.getNrOfJoints());
    // trajectory_point_msg.velocities.resize(chain.getNrOfJoints());
  }


  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
      // Simple mapping (assumes message order matches chain order for brevity)
      for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
          joint_positions(i) = msg->position[i];
          
      }
  }

  bool cnv_msg_joy(const sensor_msgs::msg::Joy::SharedPtr msg){
    vel_commands = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool update = false;
    if (msg->axes[1]>0.1||msg->axes[1]<-0.1){
          RCLCPP_WARN(this->get_logger(), "----- %f ------",msg->axes[1]);
      vel_commands.x=msg->axes[1];
      update = true;
    }
    if (msg->axes[0]>0.1||msg->axes[0]<-0.1){
      vel_commands.wx=msg->axes[0];
      update = true;
    }
    if (msg->axes[4]>0.1||msg->axes[4]<-0.1){
      vel_commands.wx=msg->axes[4];
      update = true;
    }
    vel_commands.wy=msg->axes[3];
    if (msg->axes[3]>0.1||msg->axes[3]<-0.1){
      vel_commands.wy=msg->axes[3];
      update = true;
    }
    return update;

  }

  int convertInputJoy(const sensor_msgs::msg::Joy::SharedPtr msg){
    
    cnv_msg_joy(msg);
    if (!cnv_msg_joy(msg)){return 0;}
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
    
      for (const std::string &s : joint_names_) {
        double val=0.0;
        double valv=0.0;

        if (s == "arm_rot"){
          val=pos.rz;
          valv=0.0;
        }else if (s=="arm_1_up_down"){
          // val=joint_positions(0);
          // valv=joint_velocities(0);
        }else if (s=="arm_2_up_down"){
          // val=joint_positions(1);
          // valv=joint_velocities(1);
        }else if (s=="arm_3_up_down"){
          // val=joint_positions(2);
          // valv=joint_velocities(2);
        }else if (s=="rot_grasp"){
          val=pos.gr;
          valv=0.0;
        }else if (s=="grasp_ctrl"){
          val=pos.gc;
          valv=0.0;
        }
        trajectory_point_msg.positions.push_back(val);
          RCLCPP_WARN(this->get_logger(), "%s", s.c_str());
          RCLCPP_WARN(this->get_logger(), "%f",val);
        trajectory_point_msg.velocities.push_back(valv);
          RCLCPP_WARN(this->get_logger(), "%f", valv);
      }
      double time_point = 0.25;
      double time_point_sec = std::floor(time_point);
      trajectory_point_msg.time_from_start.sec = static_cast<int>(time_point_sec);
      trajectory_point_msg.time_from_start.nanosec =
        static_cast<uint32_t>((time_point - time_point_sec) * 1E9);
      trajectory_msg.points.push_back(trajectory_point_msg);

    int trajectory_len = 1;
    // double total_time = 0.5;
    // double dt = total_time / static_cast<double>(trajectory_len - 1);

    double max_v = 0.01;
    double tmp = 0.0;
    tmp=pos.rz+max_v*vel_commands.x;
    if (tmp>3.14||tmp<-3.14){
      pos.rz = 3.14;
    }else{
      pos.rz=tmp;
    }
    pos.up1 = 0.0;
    pos.up2 = 0.0;
    pos.up3 = 0.0;

    tmp=pos.gr+max_v*vel_commands.wx;
    if (tmp>3.14||tmp<-3.14){
      pos.gr = 3.14;
    }else{
      pos.gr=tmp;
    }

    tmp=pos.gc+max_v*vel_commands.wy;
    if (tmp>3.14||tmp<-3.14){
      pos.gc = 3.14;
    }else{
      pos.gc=tmp;
    }

    for (int i = 0; i < trajectory_len; i++){
      double fraction = i / (static_cast<double>(trajectory_len - 1));
      
      // twist.vel.x(vel_commands.x*fraction);
      // twist.vel.y(vel_commands.y*fraction);
      // twist.vel.z(vel_commands.z*fraction);
      // twist.rot.x(vel_commands.wx*fraction);
      // twist.rot.y(vel_commands.wy*fraction);
      // twist.rot.z(vel_commands.wz*fraction);
      twist.vel.x(1.0);
      twist.vel.y(0.0*fraction);
      twist.vel.z(0.0);
      twist.rot.x(0.0);
      twist.rot.y(0.0);
      twist.rot.z(0.0);
      
    
      int ik_status = ik_vel_solver_->CartToJnt(joint_positions, twist, joint_velocities);
      if (ik_status < 0) {
            RCLCPP_WARN(this->get_logger(), "IK Solver could not find a solution!");
            return -1;
        }
      // copy to trajectory_point_msg
      // std::memcpy(
      //   trajectory_point_msg.positions.data(), joint_positions.data.data(),
      //   trajectory_point_msg.positions.size() * sizeof(double));
      // std::memcpy(
      //   trajectory_point_msg.velocities.data(), joint_velocities.data.data(),
      //   trajectory_point_msg.velocities.size() * sizeof(double));
      
        
    trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg2;
      for (const std::string &s : joint_names_) {
        double val=0.0;
        double valv=0.0;

        if (s == "arm_rot"){
          val=pos.rz;
          valv=0.0;
        }else if (s=="arm_1_up_down"){

          val=0.0;
          valv=0.0;
          // val=joint_positions(0);
          // valv=joint_velocities(0);
        }else if (s=="arm_2_up_down"){
          val=0.0;
          valv=0.0;
          // val=joint_positions(1);
          // valv=joint_velocities(1);
        }else if (s=="arm_3_up_down"){
          val=0.0;
          valv=0.0;
          // val=joint_positions(2);
          // valv=joint_velocities(2);
        }else if (s=="rot_grasp"){
          val=pos.gr;
          valv=0.0;
        }else if (s=="grasp_ctrl"){
          val=pos.gc;
          valv=0.0;
        }
        trajectory_point_msg2.positions.push_back(val);
          RCLCPP_WARN(this->get_logger(), "%s", s.c_str());
          RCLCPP_WARN(this->get_logger(), "%f",val);
        trajectory_point_msg2.velocities.push_back(valv);
          RCLCPP_WARN(this->get_logger(), "%f", valv);
      }


      // integrate joint velocities
      // joint_positions.data += joint_velocities.data * dt;

      // set timing information
      time_point = 0.5;
      time_point_sec = std::floor(time_point);
      trajectory_point_msg2.time_from_start.sec = static_cast<int>(time_point_sec);
      trajectory_point_msg2.time_from_start.nanosec =
        static_cast<uint32_t>((time_point - time_point_sec) * 1E9);
      trajectory_msg.points.push_back(trajectory_point_msg2);
      // send zero velocities in the end
    }
    // auto & last_point_msg = trajectory_msg.points.back();
    // std::fill(last_point_msg.velocities.begin(), last_point_msg.velocities.end(), 0.0);
    this->publisher_->publish(trajectory_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return 0;
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  KDL::JntArray joint_positions;
  KDL::JntArray joint_velocities;
  KDL::Twist twist;
  KDL::Tree robot_tree;
  KDL::Chain chain;
  vel_commands_struct vel_commands {0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 };
  bool res;
  // trajectory_msgs::msg::JointTrajectory trajectory_msg;
  // trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
    
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<braccio_control_joy>());
  rclcpp::shutdown();
  return 0;
}