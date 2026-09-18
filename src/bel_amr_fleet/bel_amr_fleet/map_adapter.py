import math


class MapAdapter:
    """
    Converts between real-world warehouse coordinates and
    discrete grid coordinates used by LaCAM.

    World coordinates:
        (x, y) in meters

    Grid coordinates:
        (grid_x, grid_y) integer cells
    """

    def __init__(
        self,
        width,
        height,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0
    ):
        if width <= 0 or height <= 0:
            raise ValueError(
                "Map width and height must be positive."
            )

        if resolution <= 0:
            raise ValueError(
                "Map resolution must be greater than zero."
            )

        self.width = width
        self.height = height
        self.resolution = resolution

        self.origin_x = origin_x
        self.origin_y = origin_y

    def world_to_grid(self, x, y):
        """
        Convert warehouse coordinates in meters to
        LaCAM grid coordinates.

        Example:
            resolution = 0.5 m/cell
            origin = (-20, -15)

            world (0, 0)
                -> grid (40, 30)
        """

        grid_x = math.floor(
            (x - self.origin_x) / self.resolution
        )

        grid_y = math.floor(
            (y - self.origin_y) / self.resolution
        )

        grid_position = (
            int(grid_x),
            int(grid_y)
        )

        if not self.is_valid_grid_position(
            grid_position
        ):
            raise ValueError(
                f"World position ({x}, {y}) converts "
                f"outside the map: {grid_position}"
            )

        return grid_position

    def grid_to_world(self, grid_x, grid_y):
        """
        Convert a LaCAM grid cell back to the center
        of the corresponding warehouse cell.
        """

        grid_position = (
            int(grid_x),
            int(grid_y)
        )

        if not self.is_valid_grid_position(
            grid_position
        ):
            raise ValueError(
                f"Invalid grid position: {grid_position}"
            )

        world_x = (
            self.origin_x
            + (grid_x + 0.5) * self.resolution
        )

        world_y = (
            self.origin_y
            + (grid_y + 0.5) * self.resolution
        )

        return (
            float(world_x),
            float(world_y)
        )

    def is_valid_grid_position(
        self,
        position
    ):
        """
        Check whether a grid cell lies inside the map.
        """

        grid_x, grid_y = position

        return (
            0 <= grid_x < self.width
            and
            0 <= grid_y < self.height
        )

    def convert_robot_positions_to_grid(
        self,
        robots
    ):
        """
        Convert robot world positions into LaCAM
        grid positions.

        Input:
        [
            {
                "robot_id": "AMR-01",
                "position": (-10.0, -7.0),
                ...
            }
        ]

        Output preserves all robot information but
        replaces position with the corresponding
        grid coordinate.
        """

        converted_robots = []

        for robot in robots:

            if "robot_id" not in robot:
                raise ValueError(
                    "Robot is missing robot_id."
                )

            if "position" not in robot:
                raise ValueError(
                    f"{robot['robot_id']} is missing position."
                )

            world_x, world_y = robot["position"]

            grid_position = self.world_to_grid(
                world_x,
                world_y
            )

            converted_robot = robot.copy()

            converted_robot[
                "position"
            ] = grid_position

            converted_robots.append(
                converted_robot
            )

        return converted_robots

    def convert_world_path_to_grid(
        self,
        world_path
    ):
        """
        Convert a sequence of world coordinates
        into LaCAM grid cells.
        """

        return [
            self.world_to_grid(x, y)
            for x, y in world_path
        ]

    def convert_grid_path_to_world(
        self,
        grid_path
    ):
        """
        Convert a LaCAM grid path into real-world
        warehouse coordinates.
        """

        return [
            self.grid_to_world(
                grid_x,
                grid_y
            )
            for grid_x, grid_y in grid_path
        ]

    def convert_fleet_paths_to_world(
        self,
        robot_paths
    ):
        """
        Convert synchronized LaCAM paths for every
        robot into warehouse coordinates.

        Input:
        {
            "AMR-01": [(10, 5), (11, 5)],
            "AMR-02": [(20, 7), (20, 8)]
        }

        Output:
        {
            "AMR-01": [(x1, y1), (x2, y2)],
            "AMR-02": [(x1, y1), (x2, y2)]
        }
        """

        world_paths = {}

        for robot_id, grid_path in (
            robot_paths.items()
        ):

            world_paths[robot_id] = (
                self.convert_grid_path_to_world(
                    grid_path
                )
            )

        return world_paths

    def get_map_info(self):
        """
        Return map configuration information.
        """

        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": (
                self.origin_x,
                self.origin_y
            ),
            "world_width": (
                self.width * self.resolution
            ),
            "world_height": (
                self.height * self.resolution
            )
        }
