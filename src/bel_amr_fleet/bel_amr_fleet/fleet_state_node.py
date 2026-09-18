import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .fleet_state_adapter import FleetStateAdapter
from .map_adapter import MapAdapter


class FleetStateNode(Node):
    """
    ROS 2 integration layer for the planning system.

    Receives relative wheel odometry from the three AMRs,
    converts it to warehouse/world coordinates using each
    robot's spawn pose, and updates FleetStateAdapter.
    """

    def __init__(self):
        super().__init__('fleet_state_node')

        # --------------------------------------------------
        # Warehouse / LaCAM map configuration
        # --------------------------------------------------
        # Temporary planning-grid configuration.
        # These values can later be replaced by the final
        # warehouse occupancy-map configuration.
        self.map_adapter = MapAdapter(
            width=100,
            height=100,
            resolution=0.5,
            origin_x=-25.0,
            origin_y=-25.0
        )

        self.fleet_state = FleetStateAdapter(self.map_adapter)

        # --------------------------------------------------
        # Robot spawn poses in Yazhini's current warehouse
        # (x, y, yaw)
        # --------------------------------------------------
        self.spawn_poses = {
            'AMR-01': (13.9, -10.6, 0.0),
            'AMR-02': (13.9, -8.0, 0.0),
            'AMR-03': (13.9, -5.4, 0.0),
        }

        # Default state until battery-management data is
        # connected later.
        self.battery = {
            'AMR-01': 100.0,
            'AMR-02': 100.0,
            'AMR-03': 100.0,
        }

        self.create_subscription(
            Odometry,
            '/model/amr_01/odometry',
            lambda msg: self.odom_callback('AMR-01', msg),
            10
        )

        self.create_subscription(
            Odometry,
            '/model/amr_02/odometry',
            lambda msg: self.odom_callback('AMR-02', msg),
            10
        )

        self.create_subscription(
            Odometry,
            '/model/amr_03/odometry',
            lambda msg: self.odom_callback('AMR-03', msg),
            10
        )

        # Print the current fleet state every 2 seconds.
        self.timer = self.create_timer(
            2.0,
            self.print_fleet_state
        )

        self.get_logger().info(
            'Fleet State Node started for AMR-01, AMR-02 and AMR-03.'
        )

    def odom_callback(self, robot_id, msg):
        """
        Convert robot-relative odometry into warehouse/world
        coordinates.

        General transform:

        world_x = spawn_x + dx*cos(yaw) - dy*sin(yaw)
        world_y = spawn_y + dx*sin(yaw) + dy*cos(yaw)

        The current robots spawn with yaw = 0, but keeping the
        complete transform makes this work if spawn orientation
        changes later.
        """

        relative_x = msg.pose.pose.position.x
        relative_y = msg.pose.pose.position.y

        spawn_x, spawn_y, spawn_yaw = self.spawn_poses[robot_id]

        world_x = (
            spawn_x
            + relative_x * math.cos(spawn_yaw)
            - relative_y * math.sin(spawn_yaw)
        )

        world_y = (
            spawn_y
            + relative_x * math.sin(spawn_yaw)
            + relative_y * math.cos(spawn_yaw)
        )

        try:
            self.fleet_state.update_robot(
                robot_id=robot_id,
                x=world_x,
                y=world_y,
                battery=self.battery[robot_id],
                available=True
            )

        except ValueError as error:
            self.get_logger().error(
                f'{robot_id} state update failed: {error}'
            )

    def print_fleet_state(self):
        """
        Display world and LaCAM grid coordinates for all
        currently tracked robots.
        """

        states = self.fleet_state.get_all_states()

        if not states:
            self.get_logger().warning(
                'Waiting for robot odometry...'
            )
            return

        self.get_logger().info(
            '---------- LIVE FLEET STATE ----------'
        )

        for robot_id in sorted(states):
            state = states[robot_id]

            world_x, world_y = state['world_position']
            grid_x, grid_y = state['grid_position']

            self.get_logger().info(
                f'{robot_id} | '
                f'World=({world_x:.2f}, {world_y:.2f}) | '
                f'Grid=({grid_x}, {grid_y}) | '
                f'Battery={state["battery"]:.1f}%'
            )


def main(args=None):
    rclpy.init(args=args)

    node = FleetStateNode()

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
