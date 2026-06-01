#!/usr/bin/env python3

import numpy as np
import sys
from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as Rot


class JoyDriver(Node):
    def __init__(self):
        super().__init__('joy_driver')

        self._twist_pub = self.create_publisher(Twist, "/commanded_vel", 10)
        self.gripper_state_pub = self.create_publisher(String, "/gripper_state", 1)

        self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self._a_pressed = False
        self._b_pressed = False
        self._x_pressed = False
        self._y_pressed = False
        self._grasped = False
        self._ax2_init = False
        self._ax5_init = False

        # Rotation from SpaceMouse sensor frame to robot base frame.
        # Set sm_to_base_rpy (roll, pitch, yaw in radians) to align axes.
        # Default is identity (no change). Tune by observing /commanded_vel
        # while moving each SpaceMouse axis individually.
        self.declare_parameter('sm_to_base_rpy', [0.0, 0.0, 0.0])
        rpy = self.get_parameter('sm_to_base_rpy').get_parameter_value().double_array_value
        self._R = Rot.from_euler('xyz', list(rpy)).as_matrix()

    def mapping(self, x, low=0.005, high=0.25):
        a = (np.log(high) - np.log(low)) / 0.9
        b = np.exp(np.log(low) - 0.1 * a)
        return np.sign(x) * b * (np.exp(a * np.abs(x)) - 1)

    def on_joy(self, msg):

        # self.get_logger().info(f'Received joy msg: {msg}')
        
        ax = np.array(msg.axes, dtype=float)

        # SpaceMouse (spacenav_node) publishes:
        # - axes length = 6: [vx, vy, vz, wx, wy, wz]
        # - buttons length = 2
        is_spacenav = (len(msg.axes) >= 6) and (len(msg.buttons) <= 2)

        t = Twist()

        if is_spacenav:
            lin = self._R @ np.array([
                self.mapping(ax[0]),
                self.mapping(ax[1]),
                self.mapping(ax[2]),
            ])
            ang = self._R @ np.array([
                self.mapping(ax[3], low=0.01, high=1),
                self.mapping(ax[4], low=0.01, high=1),
                self.mapping(ax[5], low=0.01, high=1),
            ])
            t.linear.x, t.linear.y, t.linear.z    = float(lin[0]), float(lin[1]), float(lin[2])
            t.angular.x, t.angular.y, t.angular.z = float(ang[0]), float(ang[1]), float(ang[2])
        else:
            # Xbox/gamepad mapping (legacy).
            # Some controllers use triggers which can have non-0 "rest" values.
            if len(ax) < 6:
                return

            if ax[2] != 0:
                self._ax2_init = True
            if not self._ax2_init:
                ax[2] = 1
            if ax[5] != 0:
                self._ax5_init = True
            if not self._ax5_init:
                ax[5] = 1

            t.linear.x = self.mapping(ax[1])
            t.linear.y = self.mapping(ax[0])
            t.linear.z = self.mapping((ax[2] - ax[5]) / 2)
            t.angular.x = self.mapping(-ax[3], low=0.01, high=1)
            t.angular.y = self.mapping(ax[4], low=0.01, high=1)

            # Guard against shorter button arrays.
            if len(msg.buttons) > 4 and msg.buttons[4]:
                t.angular.z = -np.pi / 4
            if len(msg.buttons) > 5 and msg.buttons[5]:
                t.angular.z = np.pi / 4

        self._twist_pub.publish(t)

        # Left button: toggle gripper
        if len(msg.buttons) > 0 and msg.buttons[0] and not self._a_pressed:
            self._a_pressed = True
            self.gripper_state_pub.publish(String(data="toggle_gripper"))
        elif len(msg.buttons) > 0 and not msg.buttons[0]:
            self._a_pressed = False




def main(args=None):
    rclpy.init(args=args)
    node = JoyDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()