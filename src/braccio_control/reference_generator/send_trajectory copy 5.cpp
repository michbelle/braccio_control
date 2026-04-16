// Copyright 2023 ros2_control Development Team
// Licensed under the Apache License, Version 2.0

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>

struct vel_commands_struct {
  double x, y, z;
  double wx, wy, wz;
};

class BraccioControlJoy : public rclcpp::Node
{
public:
  BraccioControlJoy() : Node("braccio_control_joy")
  {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/braccio_controller/joint_trajectory", 10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&BraccioControlJoy::convertInputJoy, this, std::placeholders::_1));
    
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&BraccioControlJoy::joint_callback, this, std::placeholders::_1));
    
    // Get Robot Description from /robot_state_publisher
    this->declare_parameter("robot_description", "");
    std::string robot_description = this->get_parameter("robot_description").as_string();
    
    if (!kdl_parser::treeFromString(robot_description, robot_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl tree");
      return;
    }

    if (!robot_tree.getChain("base_link", "tool0", chain)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain");
      return;
    }

    // Initialize KDL structures
    joint_positions_current.resize(chain.getNrOfJoints());
    joint_velocities.resize(chain.getNrOfJoints());
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain);

    // Initialize Trajectory Message Headers
    trajectory_msg.header.frame_id = "base_link";
    for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
      auto joint = chain.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::Fixed) {
        trajectory_msg.joint_names.push_back(joint.getName());
      }
    }
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
  {
    // Important: Update our internal state from the actual robot state
    // This assumes the order in JointState matches our KDL chain
    for (unsigned int i = 0; i < msg->position.size() && i < joint_positions_current.rows(); ++i) {
        joint_positions_current(i) = msg->position[i];
    }
  }

  void convertInputJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // 1. Reset command struct
    vel_commands = {0,0,0,0,0,0};
    bool active = false;

    // Mapping Joystick to Cartesian Velocities
    vel_commands.x = msg->axes[1] * 0.1; // Scale factor 0.1 m/s
    vel_commands.y = msg->axes[0] * 0.1;
    
    // Z axis control (using buttons/triggers)
    if (msg->buttons[4]) vel_commands.z = 0.05;      // LB
    else if (msg->axes[2] < 0.9) vel_commands.z = -0.05; // LT axis

    // Orientation
    vel_commands.wx = msg->axes[4] * 0.2;
    vel_commands.wy = msg->axes[3] * 0.2;

    // Check if there is any input to avoid publishing static points
    if (std::abs(vel_commands.x) > 0.01 || std::abs(vel_commands.y) > 0.01 || 
        std::abs(vel_commands.z) > 0.01 || std::abs(vel_commands.wx) > 0.01) {
        active = true;
    }

    if (!active) return;

    // 2. Prepare Trajectory
    trajectory_msg.points.clear(); 
    trajectory_msg.header.stamp = this->now();

    KDL::Twist twist;
    twist.vel = KDL::Vector(vel_commands.x, vel_commands.y, vel_commands.z);
    twist.rot = KDL::Vector(vel_commands.wx, vel_commands.wy, vel_commands.wz);

    // 3. IK Calculation
    // We solve for joint velocities based on current positions
    KDL::JntArray tmp_joint_positions_current=joint_positions_current;

    int ret = ik_vel_solver_->CartToJnt(tmp_joint_positions_current, twist, joint_velocities);
    
    if (ret >= 0) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      double dt = 0.1; // Look-ahead time

      point.positions.resize(chain.getNrOfJoints());
      point.velocities.resize(chain.getNrOfJoints());

      for (unsigned int i = 0; i < chain.getNrOfJoints(); i++) {
        // Position Control logic: Current Position + (Velocity * dt)
        point.positions[i] = tmp_joint_positions_current(i) + (joint_velocities(i) * dt);
        point.velocities[i] = joint_velocities(i);
      }

      point.time_from_start = rclcpp::Duration::from_seconds(dt);
      trajectory_msg.points.push_back(point);
      
      publisher_->publish(trajectory_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "IK Solver failed!");
    }
  }

  // Members
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  KDL::JntArray joint_positions_current;
  KDL::JntArray joint_velocities;
  KDL::Tree robot_tree;
  KDL::Chain chain;
  vel_commands_struct vel_commands;
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BraccioControlJoy>());
  rclcpp::shutdown();
  return 0;
}