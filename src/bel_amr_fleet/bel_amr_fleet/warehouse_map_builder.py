import math
from pathlib import Path

from shapely.geometry import Point, box
from shapely.affinity import rotate, translate
from shapely.ops import unary_union


class WarehouseMapBuilder:
    """
    Builds the static LaCAM/A* occupancy grid corresponding to the
    Gazebo OpenRobotics warehouse used by BEL-AMR-Fleet.

    Dynamic AMRs are intentionally NOT inserted into this static map.
    """

    def __init__(
        self,
        width=100,
        height=100,
        resolution=0.5,
        origin_x=-25.0,
        origin_y=-25.0,
        robot_clearance=0.5,
    ):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.robot_clearance = robot_clearance

    def rectangle(
        self,
        center_x,
        center_y,
        size_x,
        size_y,
        yaw=0.0,
    ):
        shape = box(
            -size_x / 2.0,
            -size_y / 2.0,
            size_x / 2.0,
            size_y / 2.0,
        )

        if yaw != 0.0:
            shape = rotate(
                shape,
                math.degrees(yaw),
                origin=(0, 0),
            )

        return translate(
            shape,
            xoff=center_x,
            yoff=center_y,
        )

    def create_static_geometry(self):
        obstacles = []

        # ---------------------------------------------------------
        # 1. MAIN WAREHOUSE BOUNDARY
        #
        # Warehouse collision mesh bounds:
        # X = -15 .. 15
        # Y = -25 .. 25
        #
        # Represent only the outer physical boundary here instead
        # of filling the complicated STL cross-section loops.
        # ---------------------------------------------------------

        wall_thickness = 0.15

        obstacles.extend([
            self.rectangle(
                0.0,
                -25.0,
                30.0,
                wall_thickness,
            ),
            self.rectangle(
                0.0,
                25.0,
                30.0,
                wall_thickness,
            ),
            self.rectangle(
                -15.0,
                0.0,
                wall_thickness,
                50.0,
            ),
            self.rectangle(
                15.0,
                0.0,
                wall_thickness,
                50.0,
            ),
        ])

        # ---------------------------------------------------------
        # 2. WAREHOUSE STRUCTURAL COLUMNS
        #
        # Derived from the actual warehouse collision STL.
        # ---------------------------------------------------------

        column_positions = [
            (-7.425, 15.0),
            (-7.425, 7.5),
            (-7.425, 0.0),
            (-7.425, -7.5),
            (-7.425, -15.0),

            (7.425, 15.0),
            (7.425, 7.5),
            (7.425, 0.0),
            (7.425, -7.5),
            (7.425, -15.0),
        ]

        for x, y in column_positions:
            obstacles.append(
                self.rectangle(
                    x,
                    y,
                    0.5,
                    0.5,
                )
            )

        # ---------------------------------------------------------
        # 3. STANDARD SHELVES
        #
        # Actual collision:
        # box size = 3.6 x 0.6
        #
        # Model link has local X offset -0.5 m.
        # All current world shelf yaws are zero.
        # ---------------------------------------------------------

        shelves = [
            (-4.41528, -0.690987),
            (-4.41528, 2.30697),
            (-4.41528, 5.30708),
            (-4.41528, 8.34352),

            (5.60144, 8.34352),
            (5.60144, 5.30708),
            (5.60144, -0.690987),
            (5.60144, 2.30697),

            (13.3818, -21.2416),
            (13.3818, -19.0028),
            (13.3818, -16.4478),
            (13.3818, -14.1028),
        ]

        for x, y in shelves:
            obstacles.append(
                self.rectangle(
                    x - 0.5,
                    y,
                    3.6,
                    0.6,
                )
            )

        # ---------------------------------------------------------
        # 4. LARGE SHELVES
        #
        # Actual collision:
        # box size = 2.1 x 18
        #
        # Model link has local X offset -0.5 m.
        # ---------------------------------------------------------

        big_shelves = [
            (-9.34177, -13.5598),
            (13.9821, 15.319),
            (6.19777, -12.9647),
            (0.594376, -12.9647),
            (-5.36284, -12.9647),
        ]

        for x, y in big_shelves:
            obstacles.append(
                self.rectangle(
                    x - 0.5,
                    y,
                    2.1,
                    18.0,
                )
            )

        # ---------------------------------------------------------
        # 5. CHARGING STATIONS
        #
        # Actual collision = 0.1 x 0.55.
        #
        # The model has a -90 degree yaw, so its XY footprint is
        # effectively 0.55 x 0.1 in the world.
        # ---------------------------------------------------------

        charging_stations = [
            (14.7, -10.6),
            (14.7, -8.0),
            (14.7, -5.4),
            (14.7, -2.8),
            (14.7, -0.2),
        ]

        for x, y in charging_stations:
            obstacles.append(
                self.rectangle(
                    x,
                    y,
                    0.55,
                    0.1,
                )
            )

        # ---------------------------------------------------------
        # 6. CART
        #
        # Collision mesh bounds:
        # 0.560 x 1.271 m.
        #
        # Model link yaw = -90 degrees, therefore rotate footprint.
        # ---------------------------------------------------------

        obstacles.append(
            self.rectangle(
                -5.73,
                15.0,
                0.560,
                1.271,
                yaw=-math.pi / 2.0,
            )
        )

        # ---------------------------------------------------------
        # 7. PALLETS
        #
        # Largest pallet collision footprint:
        # 1.22 x 0.8 m.
        # ---------------------------------------------------------

        pallets = [
            (4.4161, 14.6952),
            (4.45415, 13.6212),
            (4.4468, 12.229),
            (-6.11913, 13.7079),
            (14.0222, -24.335),
        ]

        for x, y in pallets:
            obstacles.append(
                self.rectangle(
                    x,
                    y,
                    1.22,
                    0.8,
                )
            )

        geometry = unary_union(obstacles)

        # Inflate all static obstacles by the preliminary robot
        # clearance. This value can later be replaced by the exact
        # Pioneer footprint/costmap inflation agreed with Nav2.
        if self.robot_clearance > 0:
            geometry = geometry.buffer(
                self.robot_clearance
            )

        return geometry

    def cell_center(self, grid_x, grid_y):
        world_x = (
            self.origin_x
            + (grid_x + 0.5) * self.resolution
        )

        world_y = (
            self.origin_y
            + (grid_y + 0.5) * self.resolution
        )

        return world_x, world_y

    def rasterize(self, geometry):
        occupied = set()

        for grid_y in range(self.height):
            for grid_x in range(self.width):

                world_x, world_y = self.cell_center(
                    grid_x,
                    grid_y,
                )

                point = Point(
                    world_x,
                    world_y,
                )

                if (
                    geometry.contains(point)
                    or geometry.touches(point)
                ):
                    occupied.add(
                        (grid_x, grid_y)
                    )

        return occupied

    def write_lacam_map(
        self,
        occupied,
        output_file,
    ):
        output_file = Path(
            output_file
        ).expanduser()

        with output_file.open("w") as file:

            file.write("type octile\n")
            file.write(
                f"height {self.height}\n"
            )
            file.write(
                f"width {self.width}\n"
            )
            file.write("map\n")

            for grid_y in range(self.height):

                row = []

                for grid_x in range(
                    self.width
                ):

                    if (
                        grid_x,
                        grid_y,
                    ) in occupied:
                        row.append("@")
                    else:
                        row.append(".")

                file.write(
                    "".join(row) + "\n"
                )

        return str(output_file)

    def build(
        self,
        output_file="/tmp/bel_warehouse.map",
    ):
        print(
            "Building BEL warehouse "
            "static collision geometry..."
        )

        geometry = (
            self.create_static_geometry()
        )

        print(
            "Rasterizing static obstacles..."
        )

        occupied = self.rasterize(
            geometry
        )

        total_cells = (
            self.width * self.height
        )

        print(
            f"Grid size: "
            f"{self.width} x {self.height}"
        )

        print(
            f"Resolution: "
            f"{self.resolution} m/cell"
        )

        print(
            f"Robot clearance: "
            f"{self.robot_clearance} m"
        )

        print(
            f"Occupied cells: "
            f"{len(occupied)}"
        )

        print(
            f"Free cells: "
            f"{total_cells - len(occupied)}"
        )

        map_file = self.write_lacam_map(
            occupied,
            output_file,
        )

        print(
            f"Planning map saved: "
            f"{map_file}"
        )

        return map_file, occupied


def main():

    builder = WarehouseMapBuilder()

    builder.build(
        output_file="/tmp/bel_warehouse.map"
    )


if __name__ == "__main__":
    main()
