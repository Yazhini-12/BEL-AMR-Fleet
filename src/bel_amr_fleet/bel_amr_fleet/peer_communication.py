import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PeerCommunication(Node):

    def __init__(self):
        super().__init__('peer_communication')

        # Robot parameters
        self.declare_parameter('robot_id', 'AMR-01')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('battery', 100.0)
        self.declare_parameter('task', 'IDLE')
        self.declare_parameter('intent', 'WAIT')

        self.robot_id = self.get_parameter('robot_id').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.battery = self.get_parameter('battery').value
        self.task = self.get_parameter('task').value
        self.intent = self.get_parameter('intent').value

        # P2P fleet communication topic
        self.publisher = self.create_publisher(
            String,
            '/amr_fleet/status',
            10
        )

        self.subscription = self.create_subscription(
            String,
            '/amr_fleet/status',
            self.receive_status,
            10
        )

        # Publish every 2 seconds
        self.timer = self.create_timer(
            2.0,
            self.publish_status
        )

    def publish_status(self):

        msg = String()

        msg.data = (
            f'ROBOT={self.robot_id} | '
            f'POS=({self.x:.1f},{self.y:.1f}) | '
            f'BATTERY={self.battery:.0f}% | '
            f'TASK={self.task} | '
            f'INTENT={self.intent}'
        )

        self.publisher.publish(msg)

        self.get_logger().info(
            f'SENT → {msg.data}'
        )

    def receive_status(self, msg):

        # Ignore our own message
        if not msg.data.startswith(f'ROBOT={self.robot_id}'):
            self.get_logger().info(
                f'RECEIVED ← {msg.data}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = PeerCommunication()

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
