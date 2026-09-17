from .lacam_planner import LaCAMPlanner
from .conflict_manager import ConflictManager


class Replanner:
    """
    Handles dynamic multi-AMR replanning.

    When robot movement is disrupted, the replanner uses
    current robot positions and existing goals to request
    a new collision-free solution from LaCAM.
    """

    def __init__(self):
        self.planner = LaCAMPlanner()
        self.conflict_manager = ConflictManager()

    def replan(
        self,
        map_file,
        width,
        height,
        robots,
        current_positions,
        time_limit=10
    ):
        """
        Generate new paths from the robots' current positions.

        robots format:
        [
            {
                "robot_id": "AMR-01",
                "start": (x, y),
                "goal": (x, y)
            }
        ]

        current_positions format:
        {
            "AMR-01": (x, y),
            "AMR-02": (x, y)
        }
        """

        if not robots:
            raise ValueError("No robots supplied for replanning.")

        replanning_robots = []

        for robot in robots:
            robot_id = robot["robot_id"]

            if robot_id not in current_positions:
                raise ValueError(
                    f"Current position missing for {robot_id}"
                )

            replanning_robots.append(
                {
                    "robot_id": robot_id,
                    "start": current_positions[robot_id],
                    "goal": robot["goal"]
                }
            )

        print("Dynamic replanning requested.")

        new_paths = self.planner.plan(
            map_file=map_file,
            width=width,
            height=height,
            robots=replanning_robots,
            scenario_file="/tmp/bel_replan_scenario.scen",
            result_file="/tmp/bel_replan_result.txt",
            time_limit=time_limit
        )

        conflicts = self.conflict_manager.detect_conflicts(
            new_paths
        )

        if conflicts:
            raise RuntimeError(
                f"Replanned paths contain conflicts: {conflicts}"
            )

        print("Replanning successful.")
        print("No path conflicts detected.")

        return new_paths
