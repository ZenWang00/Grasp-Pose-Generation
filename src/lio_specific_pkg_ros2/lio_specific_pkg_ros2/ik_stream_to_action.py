#!/usr/bin/env python3
import json
import sys
import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from fp_core_msgs.action import ExecuteFunction


def _changed(a, b, eps):
    """Return True if any element differs by more than eps."""
    if a is None or b is None or len(a) != len(b):
        return True
    return any(abs(x - y) > eps for x, y in zip(a, b))


class ExecuteFunctionClient(Node):
    def __init__(self, server_name: str):
        super().__init__('joint_io_client')
        self._client = ActionClient(self, ExecuteFunction, server_name)

        # Streaming state / parameters
        self._bridge = "core"
        self._eps = 0.02          # deg: minimum change to send a new goal
        self._rate_hz = 50.0      # control loop rate (Hz)
        self._max_step_deg = 2.0  # deg per tick: rate limit; None = unlimited

        # Internal state
        self._latest_ik_deg = None  # latest IK target (deg)
        self._last_cmd_deg = None   # last commanded (deg)
        self._goal_handle = None
        self._timer = None

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        ok = self._client.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            self.get_logger().error(f"Action server not available: {self._client._action_name}")
        return ok

    # ---------- STREAMING SETUP ----------
    def start_stream_ik(self, topic: str, eps_deg: float, bridge: str,
                        rate_hz: float, reliable: bool, max_step_deg: float | None):
        self._eps = float(eps_deg)
        self._bridge = bridge
        self._rate_hz = float(rate_hz)
        self._max_step_deg = float(max_step_deg) if max_step_deg is not None else None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Float64MultiArray, topic, self._ik_cb, qos)

        period = 1.0 / self._rate_hz
        self._timer = self.create_timer(period, self._control_tick)

        self.get_logger().info(
            f"Streaming IK {topic}(rad) → {self._client._action_name}(deg), "
            f"eps={self._eps}°, rate={self._rate_hz}Hz, step≤{self._max_step_deg}°, "
            f"QoS={'RELIABLE' if reliable else 'BEST_EFFORT'}"
        )

    def _ik_cb(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 6:
            return
        arm6_deg = [math.degrees(x) for x in data[:6]]
        # Cache the latest IK target; control loop will pick it up
        self._latest_ik_deg = arm6_deg

    # ---------- CONTROL LOOP ----------
    def _rate_limit(self, current, target, max_step):
        if max_step is None or current is None:
            return target
        out = []
        for c, t in zip(current, target):
            d = t - c
            if abs(d) <= max_step:
                out.append(t)
            else:
                out.append(c + max_step * (1 if d > 0 else -1))
        return out

    def _control_tick(self):
        if self._latest_ik_deg is None:
            return

        target = self._latest_ik_deg

        # Optional: log at a lower level each tick (spammy if INFO)
        self.get_logger().debug("IK target (deg): " + ", ".join(f"{v:.3f}" for v in target))

        # Step toward IK target (limits big jumps per tick)
        desired = self._rate_limit(self._last_cmd_deg, target, self._max_step_deg)

        # Only send meaningful updates
        if not _changed(desired, self._last_cmd_deg, self._eps):
            return

        # _last_cmd_deg advances only after the server ACCEPTS the goal (see
        # _on_sent); a rejected/failed step is retried on the next tick instead
        # of being silently skipped, so the stream can never end short of the
        # final IK target.
        self._send_move_joints(desired)

    def _send_move_joints(self, arm6_deg):
        # Do NOT cancel previous goal on every tick; let the server handle preemption if it can.
        payload = {
            "joints": [1, 2, 3, 4, 5, 6],
            "joint_position": arm6_deg
        }

        self.get_logger().info(
            "Sending move_joints (deg): " + ", ".join(f"{v:.3f}" for v in arm6_deg)
        )

        goal = ExecuteFunction.Goal()
        goal.action = "move_joints"
        goal.bridge = self._bridge
        goal.arguments = json.dumps(payload)

        fut = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)

        def _on_sent(fut_):
            try:
                gh = fut_.result()
            except Exception as e:
                self.get_logger().warn(f"Failed to send goal: {e}")
                return
            if not gh.accepted:
                self.get_logger().warn("Goal rejected by server.")
                return
            self._goal_handle = gh
            self._last_cmd_deg = arm6_deg

        fut.add_done_callback(_on_sent)

    def _feedback_cb(self, feedback_msg):
        try:
            fb = feedback_msg.feedback
            # Many servers provide a text or progress field; print generically
            self.get_logger().debug(f"Feedback: {fb.feedback if hasattr(fb, 'feedback') else fb}")
        except Exception:
            pass


# ---------- CLI ----------
def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Stream IK or do one-shot read/write via fp_core ExecuteFunction action"
    )
    p.add_argument('--server-name', default='/execute_function',
                   help='Action server name (e.g., /execute_function or /robot/execute_function)')

    sub = p.add_subparsers(dest='cmd')

    # STREAM-IK (default)
    s = sub.add_parser('stream-ik', help='Subscribe to IK topic and stream to action (deg out)')
    s.add_argument('--topic', default='/panda_ik/output',
                   help='IK output topic (Float64MultiArray in radians)')
    s.add_argument('--bridge', default='core', help='Bridge name (default: core)')
    s.add_argument('--eps', type=float, default=0.02,
                   help='Min change (deg) to trigger a new goal')
    s.add_argument('--rate', type=float, default=50.0,
                   help='Control loop rate (Hz)')
    s.add_argument('--reliable', action='store_true',
                   help='Use RELIABLE QoS for IK topic (default: BEST_EFFORT)')
    s.add_argument('--max-step-deg', type=float, default=2.0,
                   help='Max per-tick change (deg); set to 0 or negative to disable')

    # READ one-shot
    r = sub.add_parser('read', help='Read actuator position(s)')
    r.add_argument('--actuator-ids', type=int, nargs='+', required=True)
    r.add_argument('--bridge', default='core')

    # WRITE one-shot (with per-joint list)
    w = sub.add_parser('write', help='Move joint(s) to position (degrees)')
    w.add_argument('--joints', type=int, nargs='+', required=True)
    w.add_argument('--joint-position', type=float, required=False,
                   help='Single value applied to all joints (deg)')
    w.add_argument('--joint-positions', type=float, nargs='+', required=False,
                   help='One value per joint (deg); length must match --joints')
    w.add_argument('--bridge', default='core')

    # Default when run with no subcommand
    p.set_defaults(
        cmd='stream-ik',
        topic='/panda_ik/output',
        bridge='core',
        eps=0.02,
        rate=50.0,
        reliable=False,
        max_step_deg=2.0,
    )
    return p


# ---------- MAIN ----------
async def main_async(argv):
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    node = ExecuteFunctionClient(args.server_name)
    if not node.wait_for_server(10.0):
        return 1

    # Normalize max_step: allow 0/negative to mean "no limit"
    max_step = None if (not hasattr(args, 'max_step_deg') or args.max_step_deg is None or args.max_step_deg <= 0) else args.max_step_deg

    if args.cmd == 'stream-ik':
        node.start_stream_ik(
            topic=args.topic,
            eps_deg=args.eps,
            bridge=args.bridge,
            rate_hz=args.rate,
            reliable=args.reliable,
            max_step_deg=max_step
        )
        rclpy.spin(node)
        node.destroy_node()
        return 0

    # One-shot modes
    if args.cmd == 'read':
        ids = args.actuator_ids if len(args.actuator_ids) > 1 else args.actuator_ids[0]
        result = await node.send_goal(
            action='read_actuator_position',
            bridge=args.bridge,
            arguments={'actuator_ids': ids}
        )

    elif args.cmd == 'write':
        joints = args.joints if len(args.joints) > 1 else args.joints[0]

        if args.joint_positions is not None:
            if isinstance(joints, list):
                if len(args.joint_positions) == 1:
                    joint_position = args.joint_positions * len(joints)
                else:
                    if len(args.joint_positions) != len(joints):
                        node.get_logger().error(
                            f"--joint-positions length ({len(args.joint_positions)}) must match --joints length ({len(joints)})"
                        )
                        node.destroy_node()
                        return 2
                    joint_position = args.joint_positions
            else:
                joint_position = args.joint_positions[0]
        else:
            if args.joint_position is None:
                node.get_logger().error(
                    "Provide either --joint-position (single) or --joint-positions (list)."
                )
                node.destroy_node()
                return 2
            joint_position = args.joint_position

        # Log outgoing values
        if isinstance(joint_position, list):
            node.get_logger().info(
                f"Write move_joints → joints={joints}, joint_position(deg)={[f'{v:.3f}' for v in joint_position]}"
            )
        else:
            node.get_logger().info(
                f"Write move_joints → joints={joints}, joint_position(deg)={joint_position:.3f}"
            )

        arguments = {'joints': joints, 'joint_position': joint_position}
        result = await node.send_goal(
            action='move_joints',
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
