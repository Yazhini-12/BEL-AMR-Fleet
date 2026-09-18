import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from .fleet_state_adapter import FleetStateAdapter
from .map_adapter import MapAdapter
from .fleet_coordinator import FleetCoordinator


class FleetStateNode(Node):
    """
    ROS 2 integration layer for the planning system.

    Pipeline:
        Gazebo odometry
            -> world coordinates
            -> LaCAM grid coordinates
            -> task allocation
            -> LaCAM + PIBT planning
            -> conflict validation
            -> world-coordinate paths
    """

    def __init__(self):
        super().__init__('fleet_state_node')

        # --------------------------------------------------
        # TEMPORARY planning grid
        # --------------------------------------------------
        # This is only for integration testing.
        # It is NOT yet the final warehouse occupancy map.
        self.map_adapter = MapAdapter(
            width=100,
            height=100,
            resolution=0.5,
            origin_x=-25.0,
            origin_y=-25.0
        )

        self.fleet_state = FleetStateAdapter(
            self.map_adapter
        )

        self.fleet_coordinator = FleetCoordinator(
            minimum_battery=20.0
        )

        # --------------------------------------------------
        # Temporary LaCAM test map
        # --------------------------------------------------
        self.map_file = "/tmp/bel_live_test.map"

        # --------------------------------------------------
        # Robot spawn poses
        # (x, y, yaw)
        # --------------------------------------------------
        self.spawn_poses = {
            'AMR-01': (13.9, -10.6, 0.0),
            'AMR-02': (13.9, -8.0, 0.0),
            'AMR-03': (13.9, -5.4, 0.0),
        }

        # Temporary battery values.
        # Real battery information will be connected later.
        self.battery = {
            'AMR-01': 100.0,
            'AMR-02': 100.0,
            'AMR-03': 100.0,
        }

        # Prevent the same test task from being planned
        # repeatedly by the timer.
        self.test_task_planned = False

        # --------------------------------------------------
        # ROS subscriptions
        # --------------------------------------------------
        self.create_subscription(
            Odometry,
            '/model/amr_01/odometry',
            lambda msg: self.odom_callback(
                'AMR-01',
                msg
            ),
            10
        )

        self.create_subscription(
            Odometry,
            '/model/amr_02/odometry',
            lambda msg: self.odom_callback(
                'AMR-02',
                msg
            ),
            10
        )

        self.create_subscription(
            Odometry,
            '/model/amr_03/odometry',
            lambda msg: self.odom_callback(
                'AMR-03',
                msg
            ),
            10
        )

        # Check fleet state every 2 seconds.
        self.timer = self.create_timer(
            2.0,
            self.process_fleet
        )

        self.get_logger().info(
            'Fleet State Node started.'
        )

        self.get_logger().info(
            'Waiting for AMR-01, AMR-02 and AMR-03 odometry...'
        )

    def odom_callback(self, robot_id, msg):
        """
        Convert robot-relative wheel odometry into
        warehouse/world coordinates.
        """

        relative_x = msg.pose.pose.position.x
        relative_y = msg.pose.pose.position.y

        spawn_x, spawn_y, spawn_yaw = (
            self.spawn_poses[robot_id]
        )

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
                f'{robot_id} state update failed: '
                f'{error}'
            )

    def process_fleet(self):
        """
        Display live fleet state and run one integration
        planning test after all three AMRs are available.
        """

        states = self.fleet_state.get_all_states()

        if len(states) < 3:
            self.get_logger().warning(
                f'Waiting for all robots... '
                f'Currently received {len(states)}/3'
            )
            return

        # --------------------------------------------------
        # Display live fleet state
        # --------------------------------------------------
        self.get_logger().info(
            '---------- LIVE FLEET STATE ----------'
        )

        for robot_id in sorted(states):
            state = states[robot_id]

            world_x, world_y = (
                state['world_position']
            )

            grid_x, grid_y = (
                state['grid_position']
            )

            self.get_logger().info(
                f'{robot_id} | '
                f'World=({world_x:.2f}, '
                f'{world_y:.2f}) | '
                f'Grid=({grid_x}, {grid_y}) | '
                f'Battery='
                f'{state["battery"]:.1f}%'
            )

        # Only run the test task once.
        if self.test_task_planned:
            return

        self.test_task_planned = True

        self.run_test_task()

    def run_test_task(self):
        """
        Run one end-to-end planning test using the current
        live AMR positions.

        The pickup and delivery positions below are temporary
        grid cells used only for integration testing.
        """

        self.get_logger().info(
            '========== PLANNING TEST START =========='
        )

        if not os.path.isfile(self.map_file):
            self.get_logger().error(
                f'Planning map not found: '
                f'{self.map_file}'
            )
            return

        # Get live robot states in the format expected by
        # TaskAllocator / FleetCoordinator.
        robots = (
            self.fleet_state.get_planner_robots()
        )

        self.get_logger().info(
            'Live robot states passed to FleetCoordinator:'
        )

        for robot in robots:
            self.get_logger().info(
                f'{robot["robot_id"]} | '
                f'Position={robot["position"]} | '
                f'Battery={robot["battery"]:.1f}% | '
                f'Available={robot["available"]}'
            )

        # --------------------------------------------------
        # Temporary integration-test task
        # --------------------------------------------------
        test_task = {
            'task_id': 'LIVE-TEST-01',
            'pickup': (70, 30),
            'goal': (65, 35)
        }

        self.get_logger().info(
            f'Task {test_task["task_id"]}: '
            f'Pickup={test_task["pickup"]}, '
            f'Delivery={test_task["goal"]}'
        )

        try:
            result = (
                self.fleet_coordinator.coordinate_task(
                    task=test_task,
                    robots=robots,
                    map_file=self.map_file,
                    width=100,
                    height=100,
                    time_limit=10
                )
            )

        except Exception as error:
            self.get_logger().error(
                f'Fleet planning failed: {error}'
            )
            return

        selected_robot = (
            result['selected_robot']
        )

        grid_paths = result['paths']

        # Convert LaCAM grid paths back into warehouse/world
        # coordinates.
        world_paths = (
            self.fleet_state
            .convert_planned_paths_to_world(
                grid_paths
            )
        )

        self.get_logger().info(
            '========== PLANNING RESULT =========='
        )

        self.get_logger().info(
            f'Task allocated to: {selected_robot}'
        )

        for robot_id in sorted(grid_paths):
            grid_path = grid_paths[robot_id]
            world_path = world_paths[robot_id]

            self.get_logger().info(
                f'{robot_id} | '
                f'Planned timesteps='
                f'{len(grid_path)}'
            )

            self.get_logger().info(
                f'{robot_id} | '
                f'Grid start={grid_path[0]} | '
                f'Grid goal={grid_path[-1]}'
            )

            self.get_logger().info(
                f'{robot_id} | '
                f'World start='
                f'({world_path[0][0]:.2f}, '
                f'{world_path[0][1]:.2f}) | '
                f'World goal='
                f'({world_path[-1][0]:.2f}, '
                f'{world_path[-1][1]:.2f})'
            )

        self.get_logger().info(
            'LaCAM + PIBT planning completed.'
        )

        self.get_logger().info(
            'Conflict validation completed.'
        )

        self.get_logger().info(
            'NOTE: This test uses a temporary free-space '
            'planning map. Robots are not being commanded '
            'to move yet.'
        )

        self.get_logger().info(
            '========== PLANNING TEST COMPLETE =========='
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
