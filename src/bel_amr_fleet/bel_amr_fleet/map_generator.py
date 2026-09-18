class MapGenerator:
    """
    Generates MovingAI-format grid maps for LaCAM.

    Symbols:
        . = free cell
        @ = obstacle
    """

    FREE = "."
    OBSTACLE = "@"

    def __init__(self, width, height):
        if width <= 0 or height <= 0:
            raise ValueError(
                "Map width and height must be positive."
            )

        self.width = int(width)
        self.height = int(height)

        self.grid = [
            [self.FREE for _ in range(self.width)]
            for _ in range(self.height)
        ]

    def is_valid_cell(self, x, y):
        """
        Check whether a cell is inside the map.
        """
        return (
            0 <= x < self.width
            and
            0 <= y < self.height
        )

    def add_obstacle(self, x, y):
        """
        Mark one grid cell as an obstacle.
        """
        x = int(x)
        y = int(y)

        if not self.is_valid_cell(x, y):
            raise ValueError(
                f"Obstacle cell ({x}, {y}) "
                f"is outside the map."
            )

        self.grid[y][x] = self.OBSTACLE

    def remove_obstacle(self, x, y):
        """
        Convert an obstacle cell back to free space.
        """
        x = int(x)
        y = int(y)

        if not self.is_valid_cell(x, y):
            raise ValueError(
                f"Cell ({x}, {y}) is outside the map."
            )

        self.grid[y][x] = self.FREE

    def add_obstacles(self, obstacles):
        """
        Add multiple obstacle cells.

        Example:
            [
                (10, 10),
                (10, 11),
                (10, 12)
            ]
        """
        for x, y in obstacles:
            self.add_obstacle(x, y)

    def add_rectangle(
        self,
        min_x,
        min_y,
        max_x,
        max_y
    ):
        """
        Fill a rectangular region with obstacles.

        Useful for:
            racks
            walls
            pallets
            blocked warehouse areas
        """

        min_x = int(min_x)
        min_y = int(min_y)
        max_x = int(max_x)
        max_y = int(max_y)

        if min_x > max_x or min_y > max_y:
            raise ValueError(
                "Invalid rectangle coordinates."
            )

        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):

                if self.is_valid_cell(x, y):
                    self.grid[y][x] = self.OBSTACLE

    def add_border(self):
        """
        Mark the outer edge of the map as obstacles.

        This prevents LaCAM from routing robots
        outside the warehouse boundary.
        """

        for x in range(self.width):
            self.grid[0][x] = self.OBSTACLE
            self.grid[self.height - 1][x] = (
                self.OBSTACLE
            )

        for y in range(self.height):
            self.grid[y][0] = self.OBSTACLE
            self.grid[y][self.width - 1] = (
                self.OBSTACLE
            )

    def is_obstacle(self, x, y):
        """
        Check whether a cell is occupied.
        """

        if not self.is_valid_cell(x, y):
            return True

        return (
            self.grid[y][x] == self.OBSTACLE
        )

    def is_free(self, x, y):
        """
        Check whether a cell is free.
        """
        return not self.is_obstacle(x, y)

    def clear(self):
        """
        Reset the entire map to free space.
        """

        self.grid = [
            [self.FREE for _ in range(self.width)]
            for _ in range(self.height)
        ]

    def get_obstacles(self):
        """
        Return all obstacle cells.
        """

        obstacles = []

        for y in range(self.height):
            for x in range(self.width):

                if self.grid[y][x] == self.OBSTACLE:
                    obstacles.append((x, y))

        return obstacles

    def save(self, output_file):
        """
        Save the grid using the MovingAI map format
        required by LaCAM.
        """

        with open(
            output_file,
            "w",
            encoding="utf-8"
        ) as file:

            file.write("type octile\n")
            file.write(
                f"height {self.height}\n"
            )
            file.write(
                f"width {self.width}\n"
            )
            file.write("map\n")

            for row in self.grid:
                file.write(
                    "".join(row) + "\n"
                )

        print(
            f"LaCAM map created: {output_file}"
        )

    def get_map_text(self):
        """
        Return the grid as printable text.
        """

        return "\n".join(
            "".join(row)
            for row in self.grid
        )
