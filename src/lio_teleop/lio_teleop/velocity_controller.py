#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
import PyKDL
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
# from tf_transformations import euler_from_quaternion

REFERENCE_FRAME = 'panda_link0'

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self._control_frame = 'panda_link0'

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._goal_pub = self.create_publisher(TwistStamped, '/panda_ik/input', 10)
        self._twist_sub = self.create_subscription(Twist, '/commanded_vel', self.on_twist, 10)

        self.get_logger().info("Velocity Controller node started.")

    def on_twist(self, msg):
        try:
            transform = self._tf_buffer.lookup_transform(
                REFERENCE_FRAME,
                self._control_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            return

        t = PyKDL.Twist()
        t[0] = msg.linear.x
        t[1] = msg.linear.y
        t[2] = msg.linear.z
        t[3] = msg.angular.x
        t[4] = msg.angular.y
        t[5] = msg.angular.z

        q = transform.transform.rotation
        rot = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        rpy = rot.GetRPY()
        R = PyKDL.Rotation.RPY(*rpy)

        t = R * t

        twist = TwistStamped()
        # twist.header.frame_id = "panda_gripper_joint"
        twist.header.frame_id = "lio_gripper_joint"
        twist.header.stamp = self.get_clock().now().to_msg()

        twist.twist.linear.x = t[0]
        twist.twist.linear.y = t[1]
        twist.twist.linear.z = t[2]
        twist.twist.angular.x = t[3]
        twist.twist.angular.y = t[4]
        twist.twist.angular.z = t[5]

        self._goal_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()