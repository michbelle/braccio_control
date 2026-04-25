#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp> // Levenberg-Marquardt IK
#include <kdl/frames.hpp>

class JoyToController : public rclcpp::Node {
public:
    JoyToController() : Node("joy_to_controller_node") {
        this->declare_parameter("robot_description", "");
        std::string urdf_xml = this->get_parameter("robot_description").as_string();

        KDL::Tree tree;
        kdl_parser::treeFromString(urdf_xml, tree);
        tree.getChain("base_servo_ud", "tool01", chain_);

        // Initialize Solvers
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        // LMA solver is robust for most 6/7-DOF chains
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, 1000, 1e-4);

        unsigned int nj = chain_.getNrOfJoints();
        current_joints_.resize(nj);
        target_joints_.resize(nj);
        // joint_names_ = {"arm_rot", "arm_1_up_down", "arm_2_up_down", "arm_3_up_down", "rot_grasp", "grasp_ctrl"}; // Match your URDF
        joint_names_ = {"arm_1_up_down", "arm_2_up_down", "arm_3_up_down"}; // Match your URDF

        // Publisher to ros2_control controller
        target_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/braccio_controller/joint_trajectory", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToController::joy_callback, this, std::placeholders::_1));
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JoyToController::joint_callback, this, std::placeholders::_1));
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Simple mapping (assumes message order matches chain order for brevity)
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
            current_joints_(i) = msg->position[i];
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        KDL::Frame current_frame;
        fk_solver_->JntToCart(current_joints_, current_frame);

        // 1. Calculate Target Cartesian Frame
        KDL::Frame target_frame = current_frame;
        target_frame.p.x(current_frame.p.x() + msg->axes[1] * 0.01);
        target_frame.p.y(current_frame.p.y() + msg->axes[0] * 0.01);
        target_frame.p.z(current_frame.p.z() + msg->axes[4] * 0.01);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Moving to -> X: %.3f, Y: %.3f, Z: %.3f", 
                target_frame.p.x(), target_frame.p.y(), target_frame.p.z());
        // 2. Solve Inverse Kinematics
        // Use current_joints as the 'seed' for the solver to find the nearest solution
        int ik_status = ik_solver_->CartToJnt(current_joints_, target_frame, target_joints_);

        if (ik_status >= 0) {
            publish_trajectory();
        } else {
            RCLCPP_WARN(this->get_logger(), "IK Solver could not find a solution!");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    }

    void publish_trajectory() {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        // for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        //     point.positions.push_back(target_joints_(i));
        // }
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            point.positions.push_back(target_joints_(i));
        }


        double time_point = 0.2;
        double time_point_sec = std::floor(time_point);
        point.time_from_start.sec = static_cast<int>(time_point_sec);
        point.time_from_start.nanosec =
          static_cast<uint32_t>((time_point - time_point_sec) * 1E9);
        
        // Small duration for smooth real-time response
        // point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        traj_msg.points.push_back(point);
        // traj_msg.header.stamp = this->get_clock()->now();

        target_pub_->publish(traj_msg);
    }

    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::JntArray current_joints_;
    KDL::JntArray target_joints_;
    std::vector<std::string> joint_names_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToController>());
    rclcpp::shutdown();
    return 0;
}