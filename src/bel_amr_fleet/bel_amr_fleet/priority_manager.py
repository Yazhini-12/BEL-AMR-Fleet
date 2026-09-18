import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PriorityManager(Node):

    def __init__(self):
        super().__init__('priority_manager')

        # Store latest status of each AMR
        self.robot_status = {}

        # Receive robot status
        self.subscription = self.create_subscription(
            String,
            '/amr_fleet/status',
            self.receive_status,
            10
        )

        # Publish calculated priority
        self.publisher = self.create_publisher(
            String,
            '/amr_fleet/priority',
            10
        )

        # Calculate priority every 2 seconds
        self.timer = self.create_timer(
            2.0,
            self.calculate_priority
        )

    def receive_status(self, msg):

        data = msg.data

        try:
            parts = data.split(' | ')

            robot_id = parts[0].split('=')[1]
            battery = float(parts[2].split('=')[1].replace('%', ''))
            task = parts[3].split('=')[1]
            intent = parts[4].split('=')[1]

            self.robot_status[robot_id] = {
                'battery': battery,
                'task': task,
                'intent': intent
            }

        except (IndexError, ValueError):
            self.get_logger().warn(
                f'Invalid status received: {data}'
            )

    def calculate_priority(self):

        for robot_id, status in self.robot_status.items():

            score = 0

            # Task importance
            if status['task'] == 'URGENT':
                score += 50
            elif status['task'] != 'IDLE':
                score += 30

            # Battery consideration
            if status['battery'] < 20:
                score += 20
            elif status['battery'] < 50:
                score += 10

            # Robot is ready to move
            if status['intent'] == 'GO':
                score += 10

            # Higher score = higher priority
            priority = max(1, 100 - score)

            msg = String()

            msg.data = (
                f'ROBOT={robot_id} | '
                f'PRIORITY={priority} | '
                f'SCORE={score}'
            )

            self.publisher.publish(msg)

            self.get_logger().info(
                f'SENT → {msg.data}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = PriorityManager()

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
