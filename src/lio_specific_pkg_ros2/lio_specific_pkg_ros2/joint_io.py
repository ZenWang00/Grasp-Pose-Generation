#!/usr/bin/env python3
import json
import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fp_core_msgs.action import ExecuteFunction

class ExecuteFunctionClient(Node):
    def __init__(self, server_name: str):
        super().__init__('joint_io_client')
        self._client = ActionClient(self, ExecuteFunction, server_name)

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f"Action server not available: {self._client._action_name}")
            return False
        return True

    async def send_goal(self, action: str, bridge: str, arguments: dict):
        goal = ExecuteFunction.Goal()
        goal.action = action
        goal.bridge = bridge
        goal.arguments = json.dumps(arguments)

        self.get_logger().info(
            f"Sending goal: action={action}, bridge={bridge}, arguments={goal.arguments}"
        )

        send_goal_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return None

        self.get_logger().info('Goal accepted; waiting for result…')
        result_future = goal_handle.get_result_async()
        result = await result_future
        return result.result

    def _feedback_cb(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            self.get_logger().info(f"Feedback: {fb.feedback if hasattr(fb, 'feedback') else fb}")
        except Exception:
            pass

def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Read or write joints via fp_core ExecuteFunction action"
    )
    p.add_argument('--server-name', default='/execute_function',
                   help='Action server name (e.g., /execute_function or /robot/execute_function)')

    sub = p.add_subparsers(dest='cmd', required=True)

    # Read: read_actuator_position
    r = sub.add_parser('read', help='Read actuator position(s)')
    r.add_argument('--actuator-ids', type=int, nargs='+', required=True,
                   help='One or more actuator IDs (ints)')
    r.add_argument('--bridge', default='core', help='Bridge name (default: core)')

    # Write: move_joints
    w = sub.add_parser('write', help='Move joint(s) to position')
    w.add_argument('--joints', type=int, nargs='+', required=True,
                   help='One or more joint IDs (ints)')
    # Make single-value optional now (we’ll validate logic ourselves)
    w.add_argument('--joint-position', type=float, required=False,
                   help='Single target value applied to all listed joints')
    # NEW: per-joint list
    w.add_argument('--joint-positions', type=float, nargs='+', required=False,
                   help='One target value per joint (length must match --joints)')
    w.add_argument('--bridge', default='core', help='Bridge name (default: core)')

    return p

async def main_async(argv):
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    node = ExecuteFunctionClient(args.server_name)
    if not node.wait_for_server(10.0):
        # let main() do the shutdown; just exit with error code
        return 1

    if args.cmd == 'read':
        ids = args.actuator_ids if len(args.actuator_ids) > 1 else args.actuator_ids[0]
        result = await node.send_goal(
            action='read_actuator_position',
            bridge=args.bridge,
            arguments={'actuator_ids': ids}
        )

    elif args.cmd == 'write':
        # normalize joints to int or list[int]
        joints = args.joints if len(args.joints) > 1 else args.joints[0]

        # decide which position argument to use
        if args.joint_positions is not None:
            # user supplied list
            if isinstance(joints, list):
                if len(args.joint_positions) == 1:
                    # allow broadcast if they gave a single value
                    joint_position = args.joint_positions * len(joints)
                else:
                    if len(args.joint_positions) != len(joints):
                        node.get_logger().error(
                            f"--joint-positions length ({len(args.joint_positions)}) "
                            f"must match --joints length ({len(joints)})"
                        )
                        return 2
                    joint_position = args.joint_positions
            else:
                # single joint id; take the first value
                joint_position = args.joint_positions[0]
        else:
            # fallback to single value (must be provided)
            if args.joint_position is None:
                node.get_logger().error(
                    "You must provide either --joint-position (single value) "
                    "or --joint-positions (list)."
                )
                return 2
            # if user gave multiple joints, single value is broadcast
            if isinstance(joints, list):
                joint_position = args.joint_position  # server may accept scalar for all
                # If your server needs per-joint list, uncomment next line:
                # joint_position = [args.joint_position] * len(joints)
            else:
                joint_position = args.joint_position

            # Optional: if your server strictly requires list for multiple joints, enforce it here.

        arguments = {'joints': joints, 'joint_position': joint_position}
        result = await node.send_goal(
            action='move_joints',   #tool_pick or tool_place for gripper control
            bridge=args.bridge,
            arguments=arguments
        )

    else:
        node.get_logger().error('Unknown command')
        result = None

    if result is not None:
        try:
            fields = vars(result)
        except TypeError:
            fields = {k: getattr(result, k) for k in dir(result) if not k.startswith('_')}
        node.get_logger().info(f"Result fields: {fields}")

    node.destroy_node()
    return 0

def main():
    rclpy.init()
    try:
        import asyncio
        rc = asyncio.run(main_async(sys.argv[1:]))
        if rc:
            sys.exit(rc)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
