#!/usr/bin/env python3
import math
import array

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

import signal, sys

class IKInterface(Node):
    def __init__(self):
        super().__init__('ik_interface_ros2')

        self.declare_parameter('physical_robot', False)
        self.physical_robot = self.get_parameter('physical_robot').get_parameter_value().bool_value

        self.ik_js = None
        self.robot_js = None
        self.gripper_angle = 0.0

        self.joint_states_lio_pub = self.create_publisher(JointState, 'ik_interface/joint_states_lio', 10)
        self.joint_states_sim_pub = self.create_publisher(JointState, 'ik_interface/joint_states_sim', 10)

        self.create_subscription(String, '/gripper_state', self.gripper_state_cb, 10)
        self.create_subscription(Float64MultiArray, '/panda_ik/output', self.ik_cb, 10)
        self.create_subscription(JointState, '/lio_joint_states', self.lio_joint_states_cb, 10)

        self.timer = self.create_timer(1.0 / 12.0, self.run_loop)

        self.get_logger().info('ik_interface has been started')

        # publisher to robot stop topic
        self.stop_pub = self.create_publisher(String, '/myp_commanded_state', 10)

        # handle Ctrl+C from terminal  (We might need to add another topic to control from other nodes, but just need so call the same cb)
        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)
        

    def _on_sigint(self, signum, frame):
        """Called when user presses Ctrl+C or kills the process."""
        msg = String()
        msg.data = "stop"
        try:
            self.stop_pub.publish(msg)
            self.get_logger().warn("Ctrl+C detected — published 'stop' to /myp_commanded_state")
            # Giving a brief moment to send the message
            rclpy.spin_once(self, timeout_sec=0.2)
        except Exception as e:
            self.get_logger().error(f"Failed to publish stop message: {e}")

      
        sys.exit(0)


    def gripper_state_cb(self, msg: String) -> None:   #
        if msg.data == 'toggle_gripper':
            self.gripper_angle = 0.523594 if self.gripper_angle == 0.0 else 0.0  # ~30 degrees in rad

            print(f'Gripper angle set to {math.degrees(self.gripper_angle):.1f} degrees')

    def lio_joint_states_cb(self, msg: JointState) -> None:
        if self.physical_robot:
            # self.get_logger().info('**** Physical robot active ****')
            self.robot_js = msg.position

    def ik_cb(self, msg: Float64MultiArray) -> None:
        self.ik_js = msg.data

    def run_loop(self) -> None:
        if self.ik_js is None:
            return

        ik_joint_states = self.ik_js

        if self.physical_robot:
            if self.robot_js is None:
                return
            self.publish_simulation(self.robot_js)
            self.publish_robot(ik_joint_states)
        else:
            self.publish_simulation(ik_joint_states)
            self.publish_robot(ik_joint_states)

    def publish_simulation(self, joint_states) -> None:
        arm = list(joint_states)
        msg_sim = JointState()
        msg_sim.name = [
            'wheel_actuated_left_joint', 'wheel_actuated_right_joint',
            'caster_left_wheel_joint', 'caster_left_base_joint',
            'caster_right_wheel_joint', 'caster_right_base_joint',
            'lio_joint1', 'lio_joint2', 'lio_joint3',
            'lio_joint4', 'lio_joint5', 'lio_joint6',
            'lio_gripper_joint', 'lio_passive_gripper_joint'
        ]

        msg_sim.position = array.array('d', [0.0] * 14)
        if len(arm) == 6:
            msg_sim.position[6:12] = array.array('d', arm)
            msg_sim.position[12] = self.gripper_angle
            msg_sim.position[13] = -self.gripper_angle
        else:
            msg_sim.position[6:12] = array.array('d', arm[:6])
            msg_sim.position[12] = arm[6]
            msg_sim.position[13] = -arm[6]

        msg_sim.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_sim_pub.publish(msg_sim)

    def publish_robot(self, joint_states) -> None:
        self.get_logger().debug('Publishing to robot')

        arm = joint_states[:6]
        msg_lio = JointState()
        msg_lio.name = [
            'lio_joint1', 'lio_joint2', 'lio_joint3',
            'lio_joint4', 'lio_joint5', 'lio_joint6', 'lio_gripper_joint'
        ]
        
        msg_lio.position = [math.degrees(rad) for rad in arm]
        msg_lio.position.append(math.degrees(self.gripper_angle))

        msg_lio.header.stamp = self.get_clock().now().to_msg()
        self.joint_states_lio_pub.publish(msg_lio)


def main(args=None):
    rclpy.init(args=args)
    node = IKInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
