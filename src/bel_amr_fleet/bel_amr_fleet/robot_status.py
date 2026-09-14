import rclpy
from rclpy.node import Node


class RobotStatus(Node):

    def __init__(self):
        super().__init__('robot_status')

        # Get robot ID from ROS parameter
        self.declare_parameter('robot_id', 'AMR-01')
        self.robot_id = self.get_parameter('robot_id').value

        self.timer = self.create_timer(2.0, self.publish_status)

    def publish_status(self):
        self.get_logger().info(
            f'{self.robot_id}: READY | DECENTRALIZED MODE'
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
