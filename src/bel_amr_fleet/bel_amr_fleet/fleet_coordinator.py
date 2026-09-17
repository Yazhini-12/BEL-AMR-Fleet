from .task_allocator import TaskAllocator
from .lacam_planner import LaCAMPlanner
from .conflict_manager import ConflictManager


class FleetCoordinator:
    """
    Coordinates task allocation and multi-AMR path planning.

    Pipeline:
        Task
          -> Task Allocation
          -> Planning Request
          -> LaCAM + PIBT
          -> Conflict Validation
          -> Synchronized AMR Paths
    """

    def __init__(self, minimum_battery=20.0):
        self.task_allocator = TaskAllocator(
            minimum_battery=minimum_battery
        )
        self.planner = LaCAMPlanner()
        self.conflict_manager = ConflictManager()

    def coordinate_task(
        self,
        task,
        robots,
        map_file,
        width,
        height,
        time_limit=10
    ):
        """
        Allocate a task and generate synchronized fleet paths.

        task format:
        {
            "task_id": "TASK-01",
            "pickup": (x, y),
            "goal": (x, y)
        }

        robot format:
        {
            "robot_id": "AMR-01",
            "position": (x, y),
            "battery": 90.0,
            "available": True
        }
        """

        if not robots:
            raise ValueError("No robots supplied to fleet coordinator.")

        allocation = self.task_allocator.allocate_task(
            task,
            robots
        )

        if allocation is None:
            raise RuntimeError(
                "No suitable AMR available for the task."
            )

        selected_robot_id = allocation["robot_id"]

        print(
            f"Task {task.get('task_id')} allocated to "
            f"{selected_robot_id}"
        )

        planning_robots = []

        for robot in robots:
            robot_id = robot["robot_id"]

            # The selected robot receives the task goal.
            if robot_id == selected_robot_id:
                goal = task["goal"]
            else:
                # Other robots remain at their current locations
                # until they have active task goals of their own.
                goal = robot["position"]

            planning_robots.append(
                {
                    "robot_id": robot_id,
                    "start": robot["position"],
                    "goal": goal
                }
            )

        paths = self.planner.plan(
            map_file=map_file,
            width=width,
            height=height,
            robots=planning_robots,
            scenario_file="/tmp/bel_fleet_scenario.scen",
            result_file="/tmp/bel_fleet_result.txt",
            time_limit=time_limit
        )

        conflicts = self.conflict_manager.detect_conflicts(
            paths
        )

        if conflicts:
            raise RuntimeError(
                f"Fleet paths contain conflicts: {conflicts}"
            )

        print("Fleet coordination successful.")
        print("No vertex or edge conflicts detected.")

        return {
            "allocation": allocation,
            "paths": paths
        }
