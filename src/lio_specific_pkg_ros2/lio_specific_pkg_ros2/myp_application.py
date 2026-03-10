#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json
from fp_core_msgs.action import ExecuteFunction
import signal, sys




class ScriptLauncher(Node):
    def __init__(self):
        super().__init__('script_launcher')

        # Path to your script that should run on the robot
        self.script_path = "/home/srajapakshe/ros2_ws/src/lio_specific_pkg_ros2/lio_specific_pkg_ros2/move_joint.py"

        # ROS 2 Action client
        self.cli = ActionClient(self, ExecuteFunction, '/execute_function')

        self.get_logger().info("ScriptLauncher has been started")






    def run(self):
        # read the script file
        with open(self.script_path, 'r') as file:
            code = file.read()

        # wait for the action server
        self.get_logger().info("Waiting for /execute_function action server...")
        if not self.cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No /execute_function action server available")
            return

        # prepare goal
        goal_msg = ExecuteFunction.Goal()
        goal_msg.action = "test_script"
        goal_msg.bridge = "core"
        goal_msg.arguments = json.dumps({
            "script_code": code,
            "script_type": "main",
            "script_id": 0
        })

        # send goal
        send_future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("test_script goal was rejected")
            return

        self.get_logger().info("test_script goal accepted")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f"Raw result: {result}")

        if result.success:
            self.get_logger().info(f"Script executed successfully, return_value={result.return_value}")
        else:
            self.get_logger().error(f"Script failed: {result.error_message}")


def main():
    rclpy.init()
    node = ScriptLauncher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
