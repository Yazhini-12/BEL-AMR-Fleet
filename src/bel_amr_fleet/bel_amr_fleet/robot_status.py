import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class RobotStatus(Node):

    def __init__(self):
        super().__init__('robot_status')

        self.declare_parameter('robot_id', 'AMR-01')
        self.robot_id = self.get_parameter('robot_id').value

        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.w = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Odometry,
            '/model/amr_01/odometry',
            self.odom_callback,
            qos
        )

        self.timer = self.create_timer(2.0, self.publish_status)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def publish_status(self):
        self.get_logger().info(
            f'{self.robot_id}: '
            f'POS=({self.x:.2f},{self.y:.2f}) | '
            f'VELOCITY={self.v:.2f} m/s | '
            f'ANGULAR={self.w:.2f} rad/s | '
            f'DECENTRALIZED MODE'
        )


def main(args=None):
    rclpy.init(args=args)

    node = RobotStatus()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()