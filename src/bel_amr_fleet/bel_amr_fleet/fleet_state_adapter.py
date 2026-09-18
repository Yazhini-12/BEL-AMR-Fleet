from .map_adapter import MapAdapter


class FleetStateAdapter:
    """
    Maintains the latest known state of each AMR and converts
    real-world positions into LaCAM grid coordinates.

    This class does not subscribe to ROS topics directly.
    Robot status information can be supplied by the ROS
    communication layer.
    """

    def __init__(self, map_adapter):
        if not isinstance(map_adapter, MapAdapter):
            raise TypeError(
                "map_adapter must be an instance of MapAdapter."
            )

        self.map_adapter = map_adapter
        self.robot_states = {}

    def update_robot(
        self,
        robot_id,
        x,
        y,
        battery=100.0,
        available=True
    ):
        """
        Store or update the latest state of one robot.

        x and y are world coordinates in meters.
        """

        if not robot_id:
            raise ValueError(
                "robot_id cannot be empty."
            )

        grid_position = (
            self.map_adapter.world_to_grid(x, y)
        )

        self.robot_states[robot_id] = {
            "robot_id": robot_id,
            "world_position": (
                float(x),
                float(y)
            ),
            "grid_position": grid_position,
            "battery": float(battery),
            "available": bool(available)
        }

        return self.robot_states[robot_id].copy()

    def update_fleet(self, robots):
        """
        Update multiple robots.

        Input example:

        [
            {
                "robot_id": "AMR-01",
                "position": (-10.0, -7.0),
                "battery": 90.0,
                "available": True
            }
        ]
        """

        updated_states = []

        for robot in robots:

            if "robot_id" not in robot:
                raise ValueError(
                    "Robot is missing robot_id."
                )

            if "position" not in robot:
                raise ValueError(
                    f"{robot['robot_id']} "
                    "is missing position."
                )

            x, y = robot["position"]

            state = self.update_robot(
                robot_id=robot["robot_id"],
                x=x,
                y=y,
                battery=robot.get(
                    "battery",
                    100.0
                ),
                available=robot.get(
                    "available",
                    True
                )
            )

            updated_states.append(state)

        return updated_states

    def get_robot_state(self, robot_id):
        """
        Return the latest state of one AMR.
        """

        if robot_id not in self.robot_states:
            return None

        return self.robot_states[
            robot_id
        ].copy()

    def get_all_states(self):
        """
        Return the latest state of every AMR.
        """

        return {
            robot_id: state.copy()
            for robot_id, state
            in self.robot_states.items()
        }

    def get_planner_robots(self):
        """
        Return robot data in the format expected by
        TaskAllocator and FleetCoordinator.

        Output:
        [
            {
                "robot_id": "AMR-01",
                "position": (grid_x, grid_y),
                "battery": 90.0,
                "available": True
            }
        ]
        """

        planner_robots = []

        for state in self.robot_states.values():

            planner_robots.append(
                {
                    "robot_id":
                        state["robot_id"],

                    "position":
                        state["grid_position"],

                    "battery":
                        state["battery"],

                    "available":
                        state["available"]
                }
            )

        return planner_robots

    def get_current_grid_positions(self):
        """
        Return the format required by Replanner.

        Example:

        {
            "AMR-01": (20, 16),
            "AMR-02": (30, 16),
            "AMR-03": (40, 16)
        }
        """

        return {
            robot_id: state["grid_position"]
            for robot_id, state
            in self.robot_states.items()
        }

    def convert_planned_paths_to_world(
        self,
        grid_paths
    ):
        """
        Convert LaCAM grid paths back into real-world
        coordinates for the navigation/control layer.
        """

        return (
            self.map_adapter
            .convert_fleet_paths_to_world(
                grid_paths
            )
        )

    def remove_robot(self, robot_id):
        """
        Remove a robot from the currently tracked fleet.
        """

        if robot_id in self.robot_states:
            del self.robot_states[robot_id]
            return True

        return False

    def clear(self):
        """
        Clear all currently stored robot states.
        """

        self.robot_states.clear()
