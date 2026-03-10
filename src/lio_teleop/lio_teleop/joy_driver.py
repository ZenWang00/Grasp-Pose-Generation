#!/usr/bin/env python3

import numpy as np
import sys
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String



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
        self._right_pressed = False
        self._grasped = False
        self._ax2_init = False
        self._ax5_init = False

    def mapping(self, x, low=0.005, high=0.25):
        a = (np.log(high) - np.log(low)) / 0.9
        b = np.exp(np.log(low) - 0.1 * a)
        return np.sign(x) * b * (np.exp(a * np.abs(x)) - 1)

    def on_joy(self, msg):

        # self.get_logger().info(f'Received joy msg: {msg}')
        
        ax = np.array(msg.axes)
        if ax[2] != 0:
            self._ax2_init = True
        if not self._ax2_init:
            ax[2] = 1
        if ax[5] != 0:
            self._ax5_init = True
        if not self._ax5_init:
            ax[5] = 1

        t = Twist()
        t.linear.x = self.mapping(ax[1])
        t.linear.y = self.mapping(ax[0])
        t.linear.z = self.mapping((ax[2] - ax[5]) / 2)
        t.angular.x = self.mapping(-ax[3], low=0.01, high=1)
        t.angular.y = self.mapping(ax[4], low=0.01, high=1)
        if msg.buttons[4]:
            t.angular.z = -np.pi / 4
        if msg.buttons[5]:
            t.angular.z = np.pi / 4
        self._twist_pub.publish(t)

        # A button (toggle gripper)
        if msg.buttons[0] and not self._a_pressed:
            self._a_pressed = True
            self.gripper_state_pub.publish(String(data="toggle_gripper"))
        elif not msg.buttons[0]:
            self._a_pressed = False

    
def main(args=None):
    rclpy.init(args=args)
    node = JoyDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()