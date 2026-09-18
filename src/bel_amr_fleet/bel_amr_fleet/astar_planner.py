import heapq
import math


class AStarPlanner:
    """
    Grid-based A* path planner for individual AMRs.

    Reads a MovingAI/LaCAM map where:
        . = free cell
        @ = obstacle
    """

    def __init__(self, allow_diagonal=True):
        self.allow_diagonal = allow_diagonal

    def load_map(self, map_file):
        with open(map_file, "r") as file:
            lines = [line.rstrip("\n") for line in file]

        if len(lines) < 5:
            raise ValueError("Invalid map file.")

        height = int(lines[1].split()[1])
        width = int(lines[2].split()[1])

        grid = lines[4:]

        if len(grid) != height:
            raise ValueError("Map height does not match grid data.")

        if any(len(row) != width for row in grid):
            raise ValueError("Map width does not match grid data.")

        return grid, width, height

    def heuristic(self, current, goal):
        dx = abs(current[0] - goal[0])
        dy = abs(current[1] - goal[1])

        if self.allow_diagonal:
            # Octile distance
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

        return dx + dy

    def get_neighbors(self, position, grid, width, height):
        x, y = position

        moves = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
        ]

        if self.allow_diagonal:
            diagonal_cost = math.sqrt(2)

            moves.extend([
                (1, 1, diagonal_cost),
                (1, -1, diagonal_cost),
                (-1, 1, diagonal_cost),
                (-1, -1, diagonal_cost),
            ])

        neighbors = []

        for dx, dy, cost in moves:
            nx = x + dx
            ny = y + dy

            if not (0 <= nx < width and 0 <= ny < height):
                continue

            if grid[ny][nx] == "@":
                continue

            # Prevent diagonal corner cutting.
            if dx != 0 and dy != 0:
                if grid[y][nx] == "@" or grid[ny][x] == "@":
                    continue

            neighbors.append(((nx, ny), cost))

        return neighbors

    def reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return path

    def plan(self, map_file, start, goal):
        grid, width, height = self.load_map(map_file)

        sx, sy = start
        gx, gy = goal

        if not (0 <= sx < width and 0 <= sy < height):
            raise ValueError(f"Start outside map: {start}")

        if not (0 <= gx < width and 0 <= gy < height):
            raise ValueError(f"Goal outside map: {goal}")

        if grid[sy][sx] == "@":
            raise ValueError(f"Start cell is blocked: {start}")

        if grid[gy][gx] == "@":
            raise ValueError(f"Goal cell is blocked: {goal}")

        open_set = []

        heapq.heappush(
            open_set,
            (0.0, start)
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        closed_set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal:
                return self.reconstruct_path(
                    came_from,
                    current
                )

            closed_set.add(current)

            for neighbor, movement_cost in self.get_neighbors(
                current,
                grid,
                width,
                height,
            ):
                if neighbor in closed_set:
                    continue

                tentative_g = (
                    g_score[current]
                    + movement_cost
                )

                if tentative_g < g_score.get(
                    neighbor,
                    float("inf"),
                ):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = (
                        tentative_g
                        + self.heuristic(neighbor, goal)
                    )

                    heapq.heappush(
                        open_set,
                        (f_score, neighbor)
                    )

        raise RuntimeError(
            f"A* could not find a path from {start} to {goal}."
        )
