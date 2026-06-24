#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <kdl/kdl.hpp>
#include <chrono>
#include <iostream>
#include <cmath>
#include <fstream>
#include <array>

#include "PandaIKRust.h"

using namespace std;

std::array<double, 6> joint_angles = {-1.6211348, -1.1968796, 1.2744727, 0.0253202, 1.4815394, -0.0567378};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("panda_ik_teleop");


    RCLCPP_INFO(node->get_logger(), "This is MAIN_TELEOP ##############################################");


    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);

    std::string urdf;
    node->declare_parameter("URDF", "");
    if (!node->get_parameter("URDF", urdf)) {
        RCLCPP_ERROR(node->get_logger(), "Could not find required parameter: URDF");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Loaded URDF param: '%s'", urdf.c_str());

    if (!init(urdf.c_str())) return 0;

    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("output", 1);
    auto panda_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("panda_commanded_pose", 1);
    auto event_pub = node->create_publisher<std_msgs::msg::String>("event", 1);

    geometry_msgs::msg::PoseStamped commandedPose;
    commandedPose.header.frame_id = "LIO_robot_base_link";
    commandedPose.pose.position.x = 0.001852;
    commandedPose.pose.position.y = -0.170257;
    commandedPose.pose.position.z = 0.782513;
    commandedPose.pose.orientation.x = -0.697770;
    commandedPose.pose.orientation.y = 0.715739;
    commandedPose.pose.orientation.z = 0.015818;
    commandedPose.pose.orientation.w = -0.024141;

    geometry_msgs::msg::Twist commandedVel;

    // IK tip is ALWAYS the TCP. Never take it from incoming message headers:
    // velocity_controller stamps its twists with "lio_gripper_joint" (the finger
    // pivot, 0.1245 m short of the TCP along the approach axis), and letting that
    // override the tip mid-grasp made the arm overshoot every grasp by exactly
    // that distance.
    const std::string frame_id = "lio_tcp_joint";
    bool initialized = false;
    bool start = true;
    bool run = true;
    int freq = 100;

    // Persistent subscription handles
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr direction_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr traj_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr commanded_pose_sub;

    // Assign the subscriptions and store them
    twist_sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        "input", 1,
        [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            RCLCPP_INFO(node->get_logger(), "Got twist input!");
            commandedVel = msg->twist;
            start = true;
        });

    command_sub = node->create_subscription<std_msgs::msg::String>(
        "commands", 1,
        [&](std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "stop_ik") run = false;
            if (msg->data == "start_ik") run = true;
        });

    direction_sub = node->create_subscription<std_msgs::msg::String>(
        "directions", 1,
        [&](std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "Left") commandedPose.pose.position.x += 0.05;
            else if (msg->data == "Right") commandedPose.pose.position.x -= 0.05;
            else if (msg->data == "Front") commandedPose.pose.position.y += 0.05;
            else if (msg->data == "Back") commandedPose.pose.position.y -= 0.05;
            else if (msg->data == "Up") commandedPose.pose.position.z += 0.05;
            else if (msg->data == "Down") commandedPose.pose.position.z -= 0.05;
            start = true;
        });

    traj_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/simulator/end_effector_pose", 1,
        [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            RCLCPP_INFO(node->get_logger(), "Received pose update");
            commandedPose.pose = msg->pose;
            start = true;
        });

    commanded_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/commanded_pose", 1,
        [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            RCLCPP_INFO(node->get_logger(), "Received commanded_pose for grasp execution");
            // The executor URDF (lio_arm.urdf) is rooted at base_footprint, which
            // coincides with LIO_robot_base_link (lio_joint1 at z=0.212 in both).
            // solve() therefore expects poses in LIO_robot_base_link, NOT
            // LIO_base_link (they differ by z=+0.2655m and Rz(+90°)).
            try {
                geometry_msgs::msg::PoseStamped transformed =
                    tfBuffer.transform(*msg, "LIO_robot_base_link",
                                       tf2::durationFromSec(0.2));
                commandedPose.pose = transformed.pose;
                RCLCPP_INFO(node->get_logger(),
                    "IK target in LIO_robot_base_link: x=%.3f y=%.3f z=%.3f",
                    transformed.pose.position.x,
                    transformed.pose.position.y,
                    transformed.pose.position.z);
            } catch (tf2::TransformException &ex) {
                if (msg->header.frame_id == "LIO_robot_base_link") {
                    commandedPose.pose = msg->pose;
                } else {
                    RCLCPP_ERROR(node->get_logger(),
                        "TF transform %s->LIO_robot_base_link failed: %s — "
                        "ignoring commanded_pose (executing it in the wrong frame "
                        "would send the arm to a rotated/offset target)",
                        msg->header.frame_id.c_str(), ex.what());
                    return;
                }
            }
            commandedVel = geometry_msgs::msg::Twist();  // zero velocity
            start = true;
        });

    rclcpp::Rate loop_rate(freq);
    auto last_msg_time = std::chrono::system_clock::now();
    std::chrono::milliseconds minimum_msg_delay = std::chrono::milliseconds(0);

    while (rclcpp::ok()) {
        if (!start || !run) {
            rclcpp::spin_some(node);
            loop_rate.sleep();
            continue;
        }

        std::array<bool, 4> errors = {false, false, false, false};
        std::array<double, 6> robot_state = joint_angles;
        std::array<double, 3> position = {
            commandedPose.pose.position.x,
            commandedPose.pose.position.y,
            commandedPose.pose.position.z
        };

        commandedPose.pose.position.x += commandedVel.linear.x / freq;
        commandedPose.pose.position.y += commandedVel.linear.y / freq;
        commandedPose.pose.position.z += commandedVel.linear.z / freq;

        KDL::Rotation robot_rot = KDL::Rotation::Quaternion(
            commandedPose.pose.orientation.x,
            commandedPose.pose.orientation.y,
            commandedPose.pose.orientation.z,
            commandedPose.pose.orientation.w);

        KDL::Rotation motion = KDL::Rotation::RPY(
            commandedVel.angular.x / freq,
            commandedVel.angular.y / freq,
            commandedVel.angular.z / freq);

        (motion * robot_rot).GetQuaternion(
            commandedPose.pose.orientation.x,
            commandedPose.pose.orientation.y,
            commandedPose.pose.orientation.z,
            commandedPose.pose.orientation.w);

        double norm = std::sqrt(
            commandedPose.pose.orientation.x * commandedPose.pose.orientation.x +
            commandedPose.pose.orientation.y * commandedPose.pose.orientation.y +
            commandedPose.pose.orientation.z * commandedPose.pose.orientation.z +
            commandedPose.pose.orientation.w * commandedPose.pose.orientation.w);

        if (norm != 1) {
            commandedPose.pose.orientation.x /= norm;
            commandedPose.pose.orientation.y /= norm;
            commandedPose.pose.orientation.z /= norm;
            commandedPose.pose.orientation.w /= norm;
        }

        std::array<double, 4> orientation = {
            commandedPose.pose.orientation.x,
            commandedPose.pose.orientation.y,
            commandedPose.pose.orientation.z,
            commandedPose.pose.orientation.w
        };

        std::array<double, 3> velocity = {
            commandedVel.linear.x,
            commandedVel.linear.y,
            commandedVel.linear.z
        };

        solve(robot_state.data(), frame_id.c_str(), position.data(), orientation.data(), velocity.data(), errors.data(), 1.00);

        if (errors[0]) {
            robot_state = joint_angles;
        }

        commandedPose.header.stamp = node->get_clock()->now();
        panda_pub->publish(commandedPose);

        // RCLCPP_INFO(node->get_logger(), "Published!");

        bool valid_output = true;
        if (initialized) {
            for (int ii = 0; ii < 6; ii++) {
                if (std::fabs(robot_state[ii] - joint_angles[ii]) > 0.1) {
                    valid_output = false;
                }
            }
            if (!valid_output) {
                if (std::fabs(commandedPose.pose.position.x - position[0]) < 0.05 &&
                    std::fabs(commandedPose.pose.position.y - position[1]) < 0.05 &&
                    std::fabs(commandedPose.pose.position.z - position[2]) < 0.05) {
                    valid_output = true;
                }
            }
        }

        if (!valid_output) {
            robot_state = joint_angles;
        }

        initialized = true;

        std_msgs::msg::Float64MultiArray joint_msg;
        joint_msg.data.insert(joint_msg.data.end(), robot_state.begin(), robot_state.end());
        pub->publish(joint_msg);
        joint_angles = robot_state;

        auto now = std::chrono::system_clock::now();
        while ((now - last_msg_time) < minimum_msg_delay) {
            now = std::chrono::system_clock::now();
        }
        last_msg_time = now;

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
