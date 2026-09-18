from .task_allocator import TaskAllocator
from .lacam_planner import LaCAMPlanner
from .conflict_manager import ConflictManager


class FleetCoordinator:
    """
    Coordinates task allocation and multi-AMR path planning.

    Pipeline:
        Task
          -> Task Allocation
          -> Plan Fleet to Pickup
          -> Pickup
          -> Plan Fleet to Delivery Goal
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

        Task format:
        {
            "task_id": "TASK-01",
            "pickup": (x, y),
            "goal": (x, y)
        }

        Robot format:
        {
            "robot_id": "AMR-01",
            "position": (x, y),
            "battery": 90.0,
            "available": True
        }
        """

        self._validate_inputs(task, robots)

        # ---------------------------------------------------------
        # STEP 1: Allocate task
        # ---------------------------------------------------------

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

        print(
            f"{selected_robot_id} will travel:"
        )
        print(
            f"  Current position -> Pickup {task['pickup']}"
        )
        print(
            f"  Pickup -> Delivery {task['goal']}"
        )

        # ---------------------------------------------------------
        # STEP 2: PLAN CURRENT POSITION -> PICKUP
        # ---------------------------------------------------------

        pickup_planning_robots = []

        for robot in robots:

            robot_id = robot["robot_id"]

            if robot_id == selected_robot_id:
                goal = task["pickup"]
            else:
                # For now, robots without another active task
                # remain at their current positions.
                goal = robot["position"]

            pickup_planning_robots.append(
                {
                    "robot_id": robot_id,
                    "start": robot["position"],
                    "goal": goal
                }
            )

        print("\nPlanning Stage 1: Fleet -> Pickup")

        pickup_paths = self.planner.plan(
            map_file=map_file,
            width=width,
            height=height,
            robots=pickup_planning_robots,
            scenario_file="/tmp/bel_pickup_scenario.scen",
            result_file="/tmp/bel_pickup_result.txt",
            time_limit=time_limit
        )

        self._validate_paths(
            pickup_paths,
            stage="pickup"
        )

        # ---------------------------------------------------------
        # STEP 3: Determine fleet positions after Stage 1
        # ---------------------------------------------------------

        positions_after_pickup = {}

        for robot in robots:

            robot_id = robot["robot_id"]

            if robot_id not in pickup_paths:
                raise RuntimeError(
                    f"No pickup path generated for {robot_id}."
                )

            if not pickup_paths[robot_id]:
                raise RuntimeError(
                    f"Pickup path for {robot_id} is empty."
                )

            positions_after_pickup[robot_id] = (
                pickup_paths[robot_id][-1]
            )

        # ---------------------------------------------------------
        # STEP 4: PLAN PICKUP -> DELIVERY
        # ---------------------------------------------------------

        delivery_planning_robots = []

        for robot in robots:

            robot_id = robot["robot_id"]

            start_position = positions_after_pickup[
                robot_id
            ]

            if robot_id == selected_robot_id:
                goal = task["goal"]
            else:
                # Other robots remain at the synchronized
                # position reached at the end of Stage 1.
                goal = start_position

            delivery_planning_robots.append(
                {
                    "robot_id": robot_id,
                    "start": start_position,
                    "goal": goal
                }
            )

        print("\nPlanning Stage 2: Pickup -> Delivery")

        delivery_paths = self.planner.plan(
            map_file=map_file,
            width=width,
            height=height,
            robots=delivery_planning_robots,
            scenario_file="/tmp/bel_delivery_scenario.scen",
            result_file="/tmp/bel_delivery_result.txt",
            time_limit=time_limit
        )

        self._validate_paths(
            delivery_paths,
            stage="delivery"
        )

        # ---------------------------------------------------------
        # STEP 5: Combine both synchronized path stages
        # ---------------------------------------------------------

        combined_paths = self._combine_paths(
            pickup_paths,
            delivery_paths
        )

        # ---------------------------------------------------------
        # STEP 6: Final conflict validation
        # ---------------------------------------------------------

        self._validate_paths(
            combined_paths,
            stage="combined"
        )

        print("\nFleet coordination successful.")
        print("Pickup and delivery paths generated.")
        print("No vertex or edge conflicts detected.")

        return {
            "allocation": allocation,
            "selected_robot": selected_robot_id,
            "pickup": task["pickup"],
            "goal": task["goal"],
            "pickup_paths": pickup_paths,
            "delivery_paths": delivery_paths,
            "paths": combined_paths
        }

    def _validate_inputs(self, task, robots):
        """
        Validate task and robot information before planning.
        """

        if not robots:
            raise ValueError(
                "No robots supplied to fleet coordinator."
            )

        if not isinstance(task, dict):
            raise ValueError(
                "Task must be provided as a dictionary."
            )

        required_task_fields = [
            "pickup",
            "goal"
        ]

        for field in required_task_fields:

            if field not in task:
                raise ValueError(
                    f"Task is missing required field: {field}"
                )

        robot_ids = set()

        for robot in robots:

            required_robot_fields = [
                "robot_id",
                "position"
            ]

            for field in required_robot_fields:

                if field not in robot:
                    raise ValueError(
                        f"Robot is missing required field: {field}"
                    )

            robot_id = robot["robot_id"]

            if robot_id in robot_ids:
                raise ValueError(
                    f"Duplicate robot ID detected: {robot_id}"
                )

            robot_ids.add(robot_id)

    def _validate_paths(
        self,
        paths,
        stage="planning"
    ):
        """
        Check generated synchronized paths for conflicts.
        """

        if not paths:
            raise RuntimeError(
                f"No paths generated during {stage} stage."
            )

        conflicts = (
            self.conflict_manager.detect_conflicts(paths)
        )

        if conflicts:
            raise RuntimeError(
                f"Fleet paths contain conflicts during "
                f"{stage} stage: {conflicts}"
            )

        print(
            f"{stage.capitalize()} paths are conflict-free."
        )

    def _combine_paths(
        self,
        first_paths,
        second_paths
    ):
        """
        Combine two synchronized planning stages.

        The first position of the second path is the same
        position as the final position of the first path,
        so it is removed to avoid duplicating the timestep.
        """

        combined_paths = {}

        robot_ids = set(first_paths.keys())

        if robot_ids != set(second_paths.keys()):
            raise RuntimeError(
                "Robot IDs do not match between planning stages."
            )

        for robot_id in first_paths:

            first_path = first_paths[robot_id]
            second_path = second_paths[robot_id]

            if not first_path:
                raise RuntimeError(
                    f"First path is empty for {robot_id}."
                )

            if not second_path:
                raise RuntimeError(
                    f"Second path is empty for {robot_id}."
                )

            if first_path[-1] != second_path[0]:
                raise RuntimeError(
                    f"Path continuity error for {robot_id}: "
                    f"{first_path[-1]} != {second_path[0]}"
                )

            combined_paths[robot_id] = (
                first_path +
                second_path[1:]
            )

        return combined_paths
