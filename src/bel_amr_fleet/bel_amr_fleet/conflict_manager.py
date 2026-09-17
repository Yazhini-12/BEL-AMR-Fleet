class ConflictManager:
    """
    Detects conflicts in synchronized multi-AMR paths.

    Supported conflicts:
    1. Vertex conflict:
       Two robots occupy the same cell at the same timestep.

    2. Edge conflict:
       Two robots swap positions between consecutive timesteps.
    """

    def detect_conflicts(self, robot_paths):
        conflicts = []

        if not robot_paths:
            return conflicts

        robot_ids = list(robot_paths.keys())

        max_timesteps = max(
            len(path) for path in robot_paths.values()
        )

        for timestep in range(max_timesteps):

            # Check every pair of robots
            for i in range(len(robot_ids)):
                for j in range(i + 1, len(robot_ids)):

                    robot_a = robot_ids[i]
                    robot_b = robot_ids[j]

                    path_a = robot_paths[robot_a]
                    path_b = robot_paths[robot_b]

                    position_a = self._get_position(
                        path_a,
                        timestep
                    )

                    position_b = self._get_position(
                        path_b,
                        timestep
                    )

                    # Vertex conflict
                    if position_a == position_b:
                        conflicts.append(
                            {
                                "type": "vertex",
                                "timestep": timestep,
                                "robots": [robot_a, robot_b],
                                "position": position_a
                            }
                        )

                    # Edge / swap conflict
                    if timestep > 0:
                        previous_a = self._get_position(
                            path_a,
                            timestep - 1
                        )

                        previous_b = self._get_position(
                            path_b,
                            timestep - 1
                        )

                        if (
                            previous_a == position_b
                            and previous_b == position_a
                            and position_a != position_b
                        ):
                            conflicts.append(
                                {
                                    "type": "edge",
                                    "timestep": timestep,
                                    "robots": [robot_a, robot_b],
                                    "positions": {
                                        robot_a: (
                                            previous_a,
                                            position_a
                                        ),
                                        robot_b: (
                                            previous_b,
                                            position_b
                                        )
                                    }
                                }
                            )

        return conflicts

    def _get_position(self, path, timestep):
        """
        Return the robot position at a timestep.

        If the robot has already reached its goal, it remains
        at its final position for later timesteps.
        """

        if not path:
            raise ValueError("Robot path cannot be empty.")

        if timestep < len(path):
            return path[timestep]

        return path[-1]
