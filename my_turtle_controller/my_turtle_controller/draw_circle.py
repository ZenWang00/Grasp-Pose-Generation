import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('draw_circle_node')
        # 创建一个发布者，发布到 /turtle1/cmd_vel 话题
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 设置定时器，每 0.1 秒运行一次控制逻辑
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.get_logger().info('海龟自动画圆节点已启动！')

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 2.0   # 线速度（前进）
        msg.angular.z = 1.0  # 角速度（旋转）
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()