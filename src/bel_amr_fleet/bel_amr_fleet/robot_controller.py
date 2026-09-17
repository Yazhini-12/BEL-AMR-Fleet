import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.x = 0.0
        self.y = 0.0

        self.cmd_pub = self.create_publisher(
            Twist,
            '/model/amr_01/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/amr_01/odometry',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control)

        self.start_x = None

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.x

    def control(self):
        if self.start_x is None:
            return

        distance = abs(self.x - self.start_x)

        msg = Twist()

        if distance < 1.0:
            msg.linear.x = 0.2
        else:
            msg.linear.x = 0.0

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = RobotController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()