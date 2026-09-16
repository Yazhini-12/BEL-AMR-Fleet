import re


class PathParser:
    """
    Parses a LaCAM result file and converts the timestep-based
    solution into individual paths for each AMR.
    """

    def parse(self, result_file, robot_ids):
        robot_paths = {
            robot_id: []
            for robot_id in robot_ids
        }

        with open(result_file, "r") as file:
            lines = file.readlines()

        if "solved=1\n" not in lines:
            raise RuntimeError("LaCAM did not find a solution.")

        try:
            solution_index = lines.index("solution=\n")
        except ValueError:
            raise RuntimeError(
                "No solution section found in LaCAM result."
            )

        solution_lines = lines[solution_index + 1:]

        for line in solution_lines:
            line = line.strip()

            if not line:
                continue

            # Remove timestep prefix.
            # Example:
            # 0:(11,6),(29,9),
            # becomes:
            # (11,6),(29,9),
            if ":" not in line:
                continue

            _, positions_text = line.split(":", 1)

            positions = re.findall(
                r"\((-?\d+),(-?\d+)\)",
                positions_text
            )

            if len(positions) != len(robot_ids):
                raise ValueError(
                    "Number of positions does not match "
                    "number of robots."
                )

            for index, (x, y) in enumerate(positions):
                robot_id = robot_ids[index]

                robot_paths[robot_id].append(
                    (int(x), int(y))
                )

        if not any(robot_paths.values()):
            raise RuntimeError(
                "No robot paths were found in LaCAM solution."
            )

        return robot_paths
