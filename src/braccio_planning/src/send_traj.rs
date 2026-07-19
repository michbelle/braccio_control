


// use rclrs::*
// std_msgs::msg::String as StringMsg;
// use ros_env::std_msgs::msg::String;

// use ros_env::sensor_msgs::msg::Joy as Joy;
// use ros_env::sensor_msgs::msg::JointState as JointState;
// use ros_env::trajectory_msgs::msg::JointTrajectory as JointTrajectory;
// use ros_env::trajectory_msgs::msg::JointTrajectoryPoint as JointTrajectoryPoint;
// use ros_env::std_msgs::msg::Bool as Bool;

struct state_joint{
    rz : f64,
    up1 : f64,
    up2 : f64,
    up3 : f64,
    gr : f64,
    gc : f64,
}

use anyhow::{Error, Result};
use rclrs::*;
use ros_env::*

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();

    let node = executor.create_node("minimal_publisher")?;

    let publisher = node.create_publisher::< ros_env::std_msgs::msg::String>("pub_msg")?;

    let mut message =  ros_env::std_msgs::msg::String::default();

    let mut publish_count: u32 = 1;

    let joint_names  = ["arm_rot", "arm_1_up_down", "arm_2_up_down", "arm_3_up_down", "rot_grasp", "grasp_ctrl"];

    while context.ok() {
        message.data = format!("Hello, world! {}", publish_count);
        println!("Publishing: [{}]", message.data);
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}