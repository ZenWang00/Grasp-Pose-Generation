#!/usr/bin/env python3

"""Teleoperate the Lio robot using a gamepad/joystick.
To modify the button/axes mapping as well as the speed of the different modes,
please look for the lio_teleop launch file
"""

import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from fp_core_msgs.srv import MoveJoint, MoveTool
from fp_core_msgs.msg import JointPosition


class TeleopLio(Node):
    def __init__(self):
        super().__init__('teleop_lio')

        self.latest_command = None
        self.stopped = True

        self.declare_and_get_params()
        self.init_ros_connections()

        self.config_mapping = [
            (self.drive_config, self.drive),
            (self.tool_config, self.move_tool),
            (self.joints_config, self.move_joints),
        ]

        self.create_timer(0.01, self.process_command)

    def declare_and_get_params(self):
        self.declare_parameter('resting_axes_values', [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        self.declare_parameter('ignored_axes', [])

        self.global_config = {
            'resting_axes_values': self.get_parameter('resting_axes_values').value,
            'ignored_axes': self.get_parameter('ignored_axes').value,
        }

        # Joint control parameters
        self.joints_config = {
            'buttons': self.get_parameter_or("joint_buttons", {'activation': 6}),
            'axes': self.get_parameter_or("joint_axes", {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5}),
            'scales': self.get_parameter_or("joint_scales", [0.1]*6),
            'velocity': self.get_parameter_or("joint_velocity", 30),
            'acceleration': self.get_parameter_or("joint_acceleration", 100),
            'position': [],
        }

        # Drive control
        self.drive_config = {
            'buttons': self.get_parameter_or("drive_buttons", {'activation': 7, 'boost': 5}),
            'axes': self.get_parameter_or("drive_axes", {'linear': 1, 'angular': 0}),
            'boost_scale': self.get_parameter_or("drive_boost_scale", 0.8),
            'angular_scale': self.get_parameter_or("drive_angular_scale", 1.0),
            'linear_scale': self.get_parameter_or("drive_linear_scale", 1.0),
        }

        # Tool control
        self.tool_config = {
            'buttons': self.get_parameter_or("tool_buttons", {'activation': 4, 'toggle': 0}),
            'axes': self.get_parameter_or("tool_axes", {'position': [0, 1, 2], 'orientation': [3, 4, 5]}),
            'position_scale': self.get_parameter_or("tool_position_scale", [1.0]*3),
            'orientation_scale': self.get_parameter_or("tool_orientation_scale", [1.0]*3),
            'velocity': self.get_parameter_or("tool_velocity", 30),
            'acceleration': self.get_parameter_or("tool_acceleration", 100),
        }

    def get_parameter_or(self, name, default_value):
        self.declare_parameter(name, default_value)
        return self.get_parameter(name).get_parameter_value().string_value or default_value

    def init_ros_connections(self):
        # Publishers
        self.drive_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.joint_pos_sub = self.create_subscription(JointPosition, "/lio/joint_positions", self.joints_callback, 10)

        # Service clients
        self.move_joints_client = self.create_client(MoveJoint, "/lio/core/move_joint")
        self.tool_pick_client = self.create_client(Trigger, "/lio/core/tool_pick")
        self.tool_place_client = self.create_client(Trigger, "/lio/core/tool_place")
        self.move_tool_client = self.create_client(MoveTool, "/lio/core/move_tool")
        self.stop_client = self.create_client(Trigger, "/lio/core/stop")

    def joy_callback(self, msg):
        self.latest_command = msg

    def joints_callback(self, msg):
        self.joints_config['position'] = msg.position

    def joints_to_move(self, data):
        axes = [self.joints_config['axes'][i] for i in self.joints_config['axes']]
        scaled_axes = list(data.axes)
        for index, scale in zip(axes, self.joints_config['scales']):
            scaled_axes[index] *= scale

        actuator_ids = [axes.index(j)+1 for j in axes if scaled_axes[j] != 0]
        positions = [scaled_axes[axes[aid-1]] for aid in actuator_ids]

        return positions, actuator_ids

    def move_joints(self, data, stop=False):
        if stop:
            ids = list(range(1, 7))
            positions = [0.0] * 6
        else:
            positions, ids = self.joints_to_move(data)

        req = MoveJoint.Request()
        req.id = ids
        req.position = positions
        req.velocity = self.joints_config['velocity']
        req.acceleration = self.joints_config['acceleration']
        req.block = False
        req.relative = True

        if self.move_joints_client.wait_for_service(timeout_sec=1.0):
            self.move_joints_client.call_async(req)

    def drive(self, data, stop=False):
        twist = Twist()
        if stop:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
        else:
            scale = self.drive_config['boost_scale'] if data.buttons[self.drive_config['buttons']['boost']] else 1.0
            twist.linear.x = self.drive_config['linear_scale'] * data.axes[self.drive_config['axes']['linear']] * scale
            twist.angular.z = self.drive_config['angular_scale'] * data.axes[self.drive_config['axes']['angular']] * scale

        self.drive_pub.publish(twist)

    def tool(self):
        if self.joints_config['position'] and self.joints_config['position'][-1] < 15:
            self.tool_place_client.call_async(Trigger.Request())
        else:
            self.tool_pick_client.call_async(Trigger.Request())

    def move_tool(self, data, stop=False):
        if stop:
            position = [0.0, 0.0, 0.0]
            orientation = [0.0, 0.0, 0.0]
        else:
            position = [self.tool_config['position_scale'][i] * data.axes[self.tool_config['axes']['position'][i]] for i in range(3)]
            orientation = [self.tool_config['orientation_scale'][i] * data.axes[self.tool_config['axes']['orientation'][i]] for i in range(3)]

        req = MoveTool.Request()
        req.x, req.y, req.z = position
        req.roll, req.pitch, req.yaw = orientation
        req.velocity = self.tool_config['velocity']
        req.acceleration = self.tool_config['acceleration']
        req.block = False
        req.relative = True
        req.frame = "base"

        self.move_tool_client.call_async(req)

    def stop_moving(self):
        if not self.stopped:
            self.drive(self.latest_command, stop=True)
            self.stop_client.call_async(Trigger.Request())
            self.stopped = True

    def command_axes_list(self, command_axes):
        return [item for value in command_axes.values() for item in (value if isinstance(value, list) else [value])]

    def are_axes_resting(self, current_axes_state, axes_to_check):
        axes_to_check = set(axes_to_check) - set(self.global_config['ignored_axes'])
        return all(current_axes_state[i] == self.global_config['resting_axes_values'][i] for i in axes_to_check)

    def process_command(self):
        if not self.latest_command:
            return

        cmd = copy.deepcopy(self.latest_command)

        for config, action in self.config_mapping:
            if cmd.buttons[config['buttons']['activation']]:
                axes_list = self.command_axes_list(config['axes'])
                if not self.are_axes_resting(cmd.axes, axes_list):
                    self.stopped = False
                    action(cmd)
                    return
                else:
                    self.stop_moving()
                    return

        self.stop_moving()

        if cmd.buttons[self.tool_config['buttons']['toggle']]:
            self.tool()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopLio()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()