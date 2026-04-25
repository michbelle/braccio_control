#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

// ─── helpers ─────────────────────────────────────────────────────────────────

static double deadzone(double v, double dz)
{
  if (std::abs(v) < dz) return 0.0;
  return std::copysign((std::abs(v) - dz) / (1.0 - dz), v);
}

static double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

// ─── node ────────────────────────────────────────────────────────────────────

class JoyArmController : public rclcpp::Node
{
public:
  explicit JoyArmController(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("joy_arm_controller", opts)
  {
    declare_and_load_params();

    if (!init_kdl())
      throw std::runtime_error("KDL initialization failed – check robot_description, "
                               "base_link, and ee_link parameters.");

    const int n = n_arm_joints();
    q_current_.resize(n);
    q_seed_.resize(n);
    KDL::SetToZero(q_current_);
    KDL::SetToZero(q_seed_);

    // ── ROS interfaces ──────────────────────────────────────────────────────
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){ joint_state_cb(m); });

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SystemDefaultsQoS(),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr m){ joy_cb(m); });

    arm_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      p_.arm_topic, rclcpp::SystemDefaultsQoS());

    grip_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      p_.grip_topic, rclcpp::SystemDefaultsQoS());

    // ── control timer ───────────────────────────────────────────────────────
    dt_ = 1.0 / p_.rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      [this](){ control_loop(); });

    RCLCPP_INFO(get_logger(),
      "Ready: %d arm joints [%s→%s], gripper joint '%s'",
      n, p_.base_link.c_str(), p_.ee_link.c_str(),
      p_.gripper_joint.c_str());
  }

private:
  // ─── parameter pod ─────────────────────────────────────────────────────────
  struct Params {
    std::string base_link, ee_link, gripper_joint;
    std::string arm_topic, grip_topic;
    std::vector<std::string> arm_joints;
    double rate_hz{50.0};
    double max_lin_vel{0.10};     // m/s
    double max_ang_vel{0.50};     // rad/s
    double max_grip_vel{0.05};    // rad/s (or m/s depending on joint type)
    double traj_horizon{0.20};    // seconds ahead for trajectory waypoint
    double deadzone{0.05};
    double max_dq_per_cycle{0.10}; // rad – per-joint velocity safety cap
    double ik_eps{1e-5};
    int64_t    ik_max_iter{500};
    // Axis indices (standard PS4/Xbox layout via joy_node)
    int64_t ax_tx{1}, ax_ty{0}, ax_tz{4};   // left-Y, left-X, right-Y
    int64_t ax_rx{7}, ax_ry{6}, ax_rz{3};   // dpad-Y, dpad-X, right-X
    int64_t ax_grip_open{5}, ax_grip_close{2}; // R2, L2 (range 1→-1 when pressed)
    double gripper_min{0.0}, gripper_max{0.8};
  } p_;

  void declare_and_load_params()
  {
    auto d = [this](const std::string & n, auto v){
      declare_parameter(n, v);
    };

    d("robot_description",     std::string{});
    d("base_link",             std::string{"base_link"});
    d("ee_link",               std::string{"tool0"});
    d("gripper_joint",         std::string{"gripper_joint"});
    d("arm_joint_names",       std::vector<std::string>{});
    d("arm_controller_topic",  std::string{"/joint_trajectory_controller/joint_trajectory"});
    d("gripper_controller_topic", std::string{"/gripper_controller/joint_trajectory"});
    d("control_rate_hz",       50.0);
    d("max_lin_vel",           0.10);
    d("max_ang_vel",           0.50);
    d("max_grip_vel",          0.05);
    d("trajectory_horizon_s",  0.20);
    d("joystick_deadzone",     0.05);
    d("max_dq_per_cycle",      0.10);
    d("ik_eps",                1e-5);
    d("ik_max_iterations",     500);
    d("joy_axis_tx",           1);
    d("joy_axis_ty",           0);
    d("joy_axis_tz",           4);
    d("joy_axis_rx",           7);
    d("joy_axis_ry",           6);
    d("joy_axis_rz",           3);
    d("joy_axis_grip_open",    5);
    d("joy_axis_grip_close",   2);
    d("gripper_min_pos",       0.0);
    d("gripper_max_pos",       0.8);

    p_.base_link      = get_parameter("base_link").as_string();
    p_.ee_link        = get_parameter("ee_link").as_string();
    p_.gripper_joint  = get_parameter("gripper_joint").as_string();
    p_.arm_joints     = get_parameter("arm_joint_names").as_string_array();
    p_.arm_topic      = get_parameter("arm_controller_topic").as_string();
    p_.grip_topic     = get_parameter("gripper_controller_topic").as_string();
    p_.rate_hz        = get_parameter("control_rate_hz").as_double();
    p_.max_lin_vel    = get_parameter("max_lin_vel").as_double();
    p_.max_ang_vel    = get_parameter("max_ang_vel").as_double();
    p_.max_grip_vel   = get_parameter("max_grip_vel").as_double();
    p_.traj_horizon   = get_parameter("trajectory_horizon_s").as_double();
    p_.deadzone       = get_parameter("joystick_deadzone").as_double();
    p_.max_dq_per_cycle = get_parameter("max_dq_per_cycle").as_double();
    p_.ik_eps         = get_parameter("ik_eps").as_double();
    p_.ik_max_iter    = get_parameter("ik_max_iterations").as_int();
    p_.ax_tx          = get_parameter("joy_axis_tx").as_int();
    p_.ax_ty          = get_parameter("joy_axis_ty").as_int();
    p_.ax_tz          = get_parameter("joy_axis_tz").as_int();
    p_.ax_rx          = get_parameter("joy_axis_rx").as_int();
    p_.ax_ry          = get_parameter("joy_axis_ry").as_int();
    p_.ax_rz          = get_parameter("joy_axis_rz").as_int();
    p_.ax_grip_open   = get_parameter("joy_axis_grip_open").as_int();
    p_.ax_grip_close  = get_parameter("joy_axis_grip_close").as_int();
    p_.gripper_min    = get_parameter("gripper_min_pos").as_double();
    p_.gripper_max    = get_parameter("gripper_max_pos").as_double();
  }

  // ─── KDL setup ─────────────────────────────────────────────────────────────
  bool init_kdl()
  {
    const std::string urdf = get_parameter("robot_description").as_string();
    if (urdf.empty()) {
      RCLCPP_ERROR(get_logger(), "robot_description is empty");
      return false;
    }

    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
      return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
      RCLCPP_ERROR(get_logger(), "Failed to build KDL tree");
      return false;
    }

    if (!tree.getChain(p_.base_link, p_.ee_link, chain_)) {
      RCLCPP_ERROR(get_logger(), "No chain from '%s' to '%s'",
                   p_.base_link.c_str(), p_.ee_link.c_str());
      return false;
    }

    // Auto-populate arm_joint_names from KDL chain if not provided
    if (p_.arm_joints.empty()) {
      for (unsigned i = 0; i < chain_.getNrOfSegments(); ++i) {
        const auto & j = chain_.getSegment(i).getJoint();
        if (j.getType() != KDL::Joint::None)
          p_.arm_joints.push_back(j.getName());
      }
      RCLCPP_INFO(get_logger(), "Auto-detected %zu arm joints",
                  p_.arm_joints.size());
    }

    // Joint limits from URDF
    q_lo_.resize(n_arm_joints());
    q_hi_.resize(n_arm_joints());
    for (int i = 0; i < n_arm_joints(); ++i) {
      auto jnt = urdf_model.getJoint(p_.arm_joints[i]);
      if (jnt && jnt->limits) {
        q_lo_(i) = jnt->limits->lower;
        q_hi_(i) = jnt->limits->upper;
      } else {
        q_lo_(i) = -M_PI;
        q_hi_(i) =  M_PI;
        RCLCPP_WARN(get_logger(), "No limits for '%s', using ±π",
                    p_.arm_joints[i].c_str());
      }
    }

    fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

    // LMA IK: weight position errors 100× more than orientation – good default
    // for most manipulation tasks where position accuracy dominates.
    Eigen::Matrix<double, 6, 1> L;
    L << 1.0, 1.0, 1.0, 0.01, 0.01, 0.01;
    ik_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(
      chain_, L, p_.ik_eps, p_.ik_max_iter);

    return true;
  }

  int n_arm_joints() const
  {
    return static_cast<int>(p_.arm_joints.size());
  }

  // ─── Callbacks ─────────────────────────────────────────────────────────────
  void joint_state_cb(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);

    for (size_t i = 0; i < msg->name.size(); ++i)
      state_map_[msg->name[i]] = msg->position[i];

    for (int j = 0; j < n_arm_joints(); ++j) {
      auto it = state_map_.find(p_.arm_joints[j]);
      if (it != state_map_.end())
        q_current_(j) = it->second;
    }

    auto it = state_map_.find(p_.gripper_joint);
    if (it != state_map_.end()) {
      gripper_pos_current_ = it->second;
      if (!gripper_initialized_) {
        // Seed the target from measured position on first contact.
        gripper_pos_target_ = gripper_pos_current_;
        gripper_initialized_ = true;
      }
    }

    js_received_ = true;
  }

  void joy_cb(sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(joy_mtx_);
    latest_joy_ = msg;
  }

  // ─── Control loop ──────────────────────────────────────────────────────────
  void control_loop()
  {
    if (!js_received_) return;

    sensor_msgs::msg::Joy::ConstSharedPtr joy;
    {
      std::lock_guard<std::mutex> lk(joy_mtx_);
      joy = latest_joy_;
    }
    if (!joy) return;

    // Stale joystick guard (> 500 ms without update → treat as zero input)
    const auto joy_age = (now() - rclcpp::Time(joy->header.stamp)).seconds();
    if (joy_age > 0.5) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Joystick data stale (%.2f s), holding position", joy_age);
      return;
    }

    // ── snapshot current arm state ──────────────────────────────────────────
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      q_seed_ = q_current_;
    }

    // ── forward kinematics ─────────────────────────────────────────────────
    KDL::Frame f_current;
    if (fk_->JntToCart(q_seed_, f_current) < 0) {
      RCLCPP_WARN(get_logger(), "FK failed");
      return;
    }

    // ── parse joystick → Cartesian delta ───────────────────────────────────
    const auto [dt, dg] = parse_joy(joy);

    // ── if no motion commanded, skip IK computation ─────────────────────────
    const bool arm_cmd   = (std::abs(dt.vel) > 1e-9 || std::abs(dt.rot) > 1e-9);
    const bool grip_cmd  = std::abs(dg) > 1e-9;

    if (arm_cmd) {
      KDL::Frame f_target = apply_delta(f_current, dt);

      KDL::JntArray q_result(n_arm_joints());
      const int ret = ik_->CartToJnt(q_seed_, f_target, q_result);

      if (ret < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                             "IK failed (code %d) – likely near singularity or "
                             "target out of workspace", ret);
      } else {
        enforce_limits(q_result);
        safety_clamp(q_seed_, q_result);
        publish_arm_traj(q_result);
        q_seed_ = q_result;   // advance seed for next cycle
      }
    }

    if (grip_cmd || gripper_initialized_) {
      publish_grip_traj(dg);
    }
  }

  // ─── Joystick parsing ──────────────────────────────────────────────────────
  struct CartDelta {
    double tx{0}, ty{0}, tz{0};
    double rx{0}, ry{0}, rz{0};
    double vel{0}, rot{0};  // norms, used to gate IK
  };

  // Returns {arm_delta, gripper_delta}
  std::pair<CartDelta, double> parse_joy(
    sensor_msgs::msg::Joy::ConstSharedPtr joy) const
  {
    const double lv = p_.max_lin_vel * dt_;
    const double av = p_.max_ang_vel * dt_;
    const double gv = p_.max_grip_vel * dt_;
    const double dz = p_.deadzone;

    auto ax = [&](int64_t idx) -> double {
      if (idx < 0 || idx >= static_cast<int>(joy->axes.size())) return 0.0;
      return deadzone(static_cast<double>(joy->axes[idx]), dz);
    };

    // Trigger axes: value ∈ [1.0 (released), -1.0 (fully pressed)]
    // Map to [0, 1]: pressed_ratio = (1 - axis) / 2
    auto trigger = [&](int64_t idx) -> double {
      if (idx < 0 || idx >= static_cast<int>(joy->axes.size())) return 0.0;
      return (1.0 - static_cast<double>(joy->axes[idx])) * 0.5;
    };

    CartDelta d;
    d.tx = ax(p_.ax_tx) * lv;
    d.ty = ax(p_.ax_ty) * lv;
    d.tz = ax(p_.ax_tz) * lv;
    d.rx = ax(p_.ax_rx) * av;
    d.ry = ax(p_.ax_ry) * av;
    d.rz = ax(p_.ax_rz) * av;
    d.vel = std::sqrt(d.tx*d.tx + d.ty*d.ty + d.tz*d.tz);
    d.rot = std::sqrt(d.rx*d.rx + d.ry*d.ry + d.rz*d.rz);

    // Gripper: open wins over close if both pressed simultaneously
    const double g_open  = trigger(p_.ax_grip_open)  * gv;
    const double g_close = trigger(p_.ax_grip_close) * gv;
    const double dg = g_open - g_close;

    return {d, dg};
  }

  // Apply translational and rotational delta to current EE frame.
  // Translation is in world frame; rotation increments are expressed in
  // the EE frame (tool-frame rotation – more intuitive for manipulation).
  KDL::Frame apply_delta(const KDL::Frame & f, const CartDelta & d) const
  {
    const KDL::Vector p_new = f.p + KDL::Vector(d.tx, d.ty, d.tz);
    // Tool-frame rotation: R_new = R_current * dR_tool
    const KDL::Rotation dR  = KDL::Rotation::RPY(d.rx, d.ry, d.rz);
    const KDL::Rotation R_new = f.M * dR;
    return KDL::Frame(R_new, p_new);
  }

  // ─── Safety ────────────────────────────────────────────────────────────────
  void enforce_limits(KDL::JntArray & q) const
  {
    for (int i = 0; i < n_arm_joints(); ++i)
      q(i) = clamp(q(i), q_lo_(i), q_hi_(i));
  }

  // Per-joint velocity cap: prevents IK from producing runaway jumps near
  // singularities or after workspace boundary violations.
  void safety_clamp(const KDL::JntArray & q0, KDL::JntArray & q1) const
  {
    for (int i = 0; i < n_arm_joints(); ++i) {
      const double dq = q1(i) - q0(i);
      if (std::abs(dq) > p_.max_dq_per_cycle)
        q1(i) = q0(i) + std::copysign(p_.max_dq_per_cycle, dq);
    }
  }

  // ─── Publishers ────────────────────────────────────────────────────────────
  void publish_arm_traj(const KDL::JntArray & q)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names  = p_.arm_joints;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(n_arm_joints());
    pt.velocities.assign(n_arm_joints(), 0.0);
    for (int i = 0; i < n_arm_joints(); ++i)
      pt.positions[i] = q(i);
    pt.time_from_start = rclcpp::Duration::from_seconds(p_.traj_horizon);

    traj.points.push_back(pt);
    arm_pub_->publish(traj);
  }

  void publish_grip_traj(double delta)
  {
    gripper_pos_target_ = clamp(
      gripper_pos_target_ + delta, p_.gripper_min, p_.gripper_max);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names  = {p_.gripper_joint};

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions      = {gripper_pos_target_};
    pt.velocities     = {0.0};
    pt.time_from_start = rclcpp::Duration::from_seconds(p_.traj_horizon);

    traj.points.push_back(pt);
    grip_pub_->publish(traj);
  }

  // ─── Members ───────────────────────────────────────────────────────────────
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA>       ik_;

  KDL::JntArray q_current_, q_seed_, q_lo_, q_hi_;
  std::map<std::string, double> state_map_;
  bool js_received_{false};

  double gripper_pos_current_{0.0};
  double gripper_pos_target_{0.0};
  bool   gripper_initialized_{false};
  double dt_{0.02};

  std::mutex state_mtx_, joy_mtx_;
  sensor_msgs::msg::Joy::ConstSharedPtr latest_joy_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr        joy_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr grip_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JoyArmController>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "%s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}