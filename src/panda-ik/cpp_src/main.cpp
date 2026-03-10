#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geosacs_msgs/msg/weighted_pose.hpp"

#include <array>
#include <cmath>
#include <string>

#include "PandaIKRust.h"

using std::placeholders::_1;

std::array<double, 6> joint_angles = {-1.6211348, -1.1968796, 1.2744727, 0.0253202, 1.4815394, -0.0567378};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("panda_ik");

  RCLCPP_INFO(node->get_logger(), "panda_ik starting");

  std::string urdf;
  node->declare_parameter("URDF", "");
  node->get_parameter("URDF", urdf);
  RCLCPP_INFO(node->get_logger(), "URDF param length: %zu", urdf.size());

  bool weighted_pose = false;
  node->declare_parameter("weighted_pose", false);
  node->get_parameter("weighted_pose", weighted_pose);

  if (!init(urdf.c_str())) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize PandaIKRust with URDF.");
    rclcpp::shutdown();
    return 1;
  }

  auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("output", 10);

  geometry_msgs::msg::PoseStamped commandedPose;
  geometry_msgs::msg::PoseStamped nextCommandedPose;
  geometry_msgs::msg::Twist commandedVel;  // default zero

  // Seed a valid quaternion before any command arrives (identity)
  nextCommandedPose.pose.orientation.w = 1.0;

  const std::string tip_name = "lio_tcp_joint";
  const int freq = 100;
  double q_weight = 1.5;

  bool have_command = false;

  // /weighted_pose (used only if weighted_pose==true)
  auto sub_weighted = node->create_subscription<geosacs_msgs::msg::WeightedPose>(
      "/weighted_pose", 10,
      [&](const geosacs_msgs::msg::WeightedPose::SharedPtr msg) {
        if (!weighted_pose) return;
        nextCommandedPose.pose = msg->pose;
        q_weight = msg->weight;
        have_command = true;
        RCLCPP_INFO_ONCE(node->get_logger(), "Received first weighted pose");
      });

  // /commanded_pose (used when weighted_pose==false)
  auto sub_pose = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/commanded_pose", 10,
      [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (weighted_pose) return;
        nextCommandedPose = *msg;
        have_command = true;
        RCLCPP_INFO_ONCE(node->get_logger(), "Received first commanded pose");
      });

  rclcpp::Rate loop_rate(freq);

  while (rclcpp::ok()) {
    if (!have_command) {
      // Publish the hard-coded initial joints until a command arrives
      std_msgs::msg::Float64MultiArray m;
      m.data.assign(joint_angles.begin(), joint_angles.end());
      pub->publish(m);

      rclcpp::spin_some(node);
      loop_rate.sleep();
      continue;
    }

    commandedPose = nextCommandedPose;

    // Ensure a valid quaternion before calling IK
    const double qnorm_sq =
        commandedPose.pose.orientation.x * commandedPose.pose.orientation.x +
        commandedPose.pose.orientation.y * commandedPose.pose.orientation.y +
        commandedPose.pose.orientation.z * commandedPose.pose.orientation.z +
        commandedPose.pose.orientation.w * commandedPose.pose.orientation.w;
    if (qnorm_sq < 1e-12) {
      commandedPose.pose.orientation.x = 0.0;
      commandedPose.pose.orientation.y = 0.0;
      commandedPose.pose.orientation.z = 0.0;
      commandedPose.pose.orientation.w = 1.0;
    }

    bool valid_output = true;
    std::array<bool, 4> errors = {false, false, false, false};
    std::array<double, 6> robot_state = joint_angles;

    std::array<double, 3> position = {
        commandedPose.pose.position.x,
        commandedPose.pose.position.y,
        commandedPose.pose.position.z};

    std::array<double, 4> orientation = {
        commandedPose.pose.orientation.x,
        commandedPose.pose.orientation.y,
        commandedPose.pose.orientation.z,
        commandedPose.pose.orientation.w};

    std::array<double, 3> velocity = {
        commandedVel.linear.x,
        commandedVel.linear.y,
        commandedVel.linear.z};

    solve(robot_state.data(), tip_name.c_str(),
          position.data(), orientation.data(), velocity.data(),
          errors.data(), q_weight);

    if (errors[0]) {
      robot_state = joint_angles;  // fallback if IK reported an error
      valid_output = false;
    }

    if (!valid_output) {
      robot_state = joint_angles;
    }

    // Publish result
    std_msgs::msg::Float64MultiArray joint_msg;
    joint_msg.data.assign(robot_state.begin(), robot_state.end());
    pub->publish(joint_msg);

    joint_angles = robot_state;

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
