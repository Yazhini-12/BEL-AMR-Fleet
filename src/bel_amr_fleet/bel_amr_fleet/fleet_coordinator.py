from .task_allocator import TaskAllocator
from .astar_planner import AStarPlanner
from .lacam_planner import LaCAMPlanner
from .conflict_manager import ConflictManager


class FleetCoordinator:
    """
    Coordinates task allocation, individual A* path planning,
    and multi-AMR coordination using LaCAM + PIBT.

    Pipeline:

        Task
          |
          v
        Task Allocation
          |
          v
        Individual A* Planning
          |
          v
        Conflict Detection
          |
          +---- No Conflict ----> Use A* Paths
          |
          +---- Conflict -------> LaCAM + PIBT
                                   |
                                   v
                           Conflict-Free Paths
          |
          v
        Final Validation
    """

    def __init__(self, minimum_battery=20.0):

        self.task_allocator = TaskAllocator(
            minimum_battery=minimum_battery
        )

        # Individual/static path planner
        self.astar_planner = AStarPlanner(
            allow_diagonal=True
        )

        # Multi-agent coordination planner
        self.lacam_planner = LaCAMPlanner()

        # Final conflict detection / validation
        self.conflict_manager = ConflictManager()

    # =========================================================
    # MAIN TASK COORDINATION
    # =========================================================

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
        Allocate a warehouse task and generate coordinated paths.

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

        self._validate_inputs(
            task,
            robots
        )

        # -----------------------------------------------------
        # STEP 1: TASK ALLOCATION
        # -----------------------------------------------------

        allocation = self.task_allocator.allocate_task(
            task,
            robots
        )

        if allocation is None:
            raise RuntimeError(
                "No suitable AMR available for the task."
            )

        selected_robot_id = allocation["robot_id"]

        print("\n======================================")
        print("TASK ALLOCATION")
        print("======================================")

        print(
            f"Task {task.get('task_id')} allocated to "
            f"{selected_robot_id}"
        )

        print(
            f"{selected_robot_id}: "
            f"Current -> Pickup {task['pickup']} "
            f"-> Delivery {task['goal']}"
        )

        # -----------------------------------------------------
        # STEP 2: CURRENT POSITION -> PICKUP
        # -----------------------------------------------------

        pickup_planning_robots = []

        for robot in robots:

            robot_id = robot["robot_id"]

            if robot_id == selected_robot_id:
                goal = task["pickup"]
            else:
                # Robots without another active task remain
                # at their current positions.
                goal = robot["position"]

            pickup_planning_robots.append(
                {
                    "robot_id": robot_id,
                    "start": robot["position"],
                    "goal": goal
                }
            )

        print("\n======================================")
        print("STAGE 1: CURRENT POSITION -> PICKUP")
        print("======================================")

        pickup_paths, pickup_method = (
            self._plan_and_coordinate(
                planning_robots=pickup_planning_robots,
                map_file=map_file,
                width=width,
                height=height,
                stage="pickup",
                time_limit=time_limit
            )
        )

        # -----------------------------------------------------
        # STEP 3: POSITIONS AFTER PICKUP
        # -----------------------------------------------------

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

        # -----------------------------------------------------
        # STEP 4: PICKUP -> DELIVERY
        # -----------------------------------------------------

        delivery_planning_robots = []

        for robot in robots:

            robot_id = robot["robot_id"]

            start_position = (
                positions_after_pickup[robot_id]
            )

            if robot_id == selected_robot_id:
                goal = task["goal"]
            else:
                goal = start_position

            delivery_planning_robots.append(
                {
                    "robot_id": robot_id,
                    "start": start_position,
                    "goal": goal
                }
            )

        print("\n======================================")
        print("STAGE 2: PICKUP -> DELIVERY")
        print("======================================")

        delivery_paths, delivery_method = (
            self._plan_and_coordinate(
                planning_robots=delivery_planning_robots,
                map_file=map_file,
                width=width,
                height=height,
                stage="delivery",
                time_limit=time_limit
            )
        )

        # -----------------------------------------------------
        # STEP 5: COMBINE PICKUP + DELIVERY
        # -----------------------------------------------------

        combined_paths = self._combine_paths(
            pickup_paths,
            delivery_paths
        )

        # -----------------------------------------------------
        # STEP 6: FINAL CONFLICT VALIDATION
        # -----------------------------------------------------

        self._validate_paths(
            combined_paths,
            stage="combined"
        )

        print("\n======================================")
        print("FLEET COORDINATION SUCCESSFUL")
        print("======================================")

        print(
            f"Pickup planning method: {pickup_method}"
        )

        print(
            f"Delivery planning method: {delivery_method}"
        )

        print(
            "Final vertex/edge conflicts: 0"
        )

        return {
            "allocation": allocation,
            "selected_robot": selected_robot_id,
            "pickup": task["pickup"],
            "goal": task["goal"],
            "pickup_planning_method": pickup_method,
            "delivery_planning_method": delivery_method,
            "pickup_paths": pickup_paths,
            "delivery_paths": delivery_paths,
            "paths": combined_paths
        }

    # =========================================================
    # A* + LaCAM/PIBT COORDINATION PIPELINE
    # =========================================================

    def _plan_and_coordinate(
        self,
        planning_robots,
        map_file,
        width,
        height,
        stage,
        time_limit
    ):
        """
        First generate an individual A* path for every AMR.

        If those paths are already conflict-free, keep them.

        If vertex or edge conflicts are detected, invoke
        LaCAM + PIBT for multi-agent coordination using the
        same starts and goals.
        """

        print("\nRunning individual A* planning...")

        astar_paths = {}

        # -----------------------------------------------------
        # A* INDIVIDUAL PATH PLANNING
        # -----------------------------------------------------

        for robot in planning_robots:

            robot_id = robot["robot_id"]
            start = robot["start"]
            goal = robot["goal"]

            # Robot is holding its current position.
            if start == goal:

                path = [start]

                print(
                    f"{robot_id}: HOLD at {start}"
                )

            else:

                path = self.astar_planner.plan(
                    map_file,
                    start,
                    goal
                )

                print(
                    f"{robot_id}: A* "
                    f"{start} -> {goal} | "
                    f"{len(path)} cells"
                )

            astar_paths[robot_id] = path

        # -----------------------------------------------------
        # CHECK A* PATHS FOR MULTI-AMR CONFLICTS
        # -----------------------------------------------------

        conflicts = (
            self.conflict_manager.detect_conflicts(
                astar_paths
            )
        )

        if not conflicts:

            print(
                "\nA* paths are already conflict-free."
            )

            print(
                "LaCAM + PIBT coordination is not required."
            )

            self._validate_paths(
                astar_paths,
                stage=f"{stage} A*"
            )

            return (
                astar_paths,
                "A*"
            )

        # -----------------------------------------------------
        # CONFLICT FOUND
        # -----------------------------------------------------

        print(
            f"\nDetected {len(conflicts)} "
            f"multi-AMR conflict(s) in A* paths."
        )

        for conflict in conflicts:
            print(
                f"  {conflict}"
            )

        print(
            "\nInvoking LaCAM + PIBT "
            "for multi-AMR conflict resolution..."
        )

        # -----------------------------------------------------
        # LaCAM + PIBT MULTI-AGENT COORDINATION
        # -----------------------------------------------------

        scenario_file = (
            f"/tmp/bel_{stage}_coordinated.scen"
        )

        result_file = (
            f"/tmp/bel_{stage}_coordinated_result.txt"
        )

        coordinated_paths = (
            self.lacam_planner.plan(
                map_file=map_file,
                width=width,
                height=height,
                robots=planning_robots,
                scenario_file=scenario_file,
                result_file=result_file,
                time_limit=time_limit
            )
        )

        # -----------------------------------------------------
        # VALIDATE COORDINATED RESULT
        # -----------------------------------------------------

        self._validate_paths(
            coordinated_paths,
            stage=f"{stage} LaCAM+PIBT"
        )

        print(
            "LaCAM + PIBT successfully resolved "
            "the A* path conflicts."
        )

        return (
            coordinated_paths,
            "A* -> LaCAM+PIBT"
        )

    # =========================================================
    # INPUT VALIDATION
    # =========================================================

    def _validate_inputs(
        self,
        task,
        robots
    ):
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
                        f"Robot is missing required field: "
                        f"{field}"
                    )

            robot_id = robot["robot_id"]

            if robot_id in robot_ids:
                raise ValueError(
                    f"Duplicate robot ID detected: "
                    f"{robot_id}"
                )

            robot_ids.add(robot_id)

    # =========================================================
    # PATH VALIDATION
    # =========================================================

    def _validate_paths(
        self,
        paths,
        stage="planning"
    ):
        """
        Validate synchronized multi-AMR paths.
        """

        if not paths:
            raise RuntimeError(
                f"No paths generated during "
                f"{stage} stage."
            )

        conflicts = (
            self.conflict_manager.detect_conflicts(
                paths
            )
        )

        if conflicts:
            raise RuntimeError(
                f"Fleet paths contain conflicts during "
                f"{stage} stage: {conflicts}"
            )

        print(
            f"{stage.capitalize()} paths are "
            f"conflict-free."
        )

    # =========================================================
    # PATH COMBINATION
    # =========================================================

    def _combine_paths(
        self,
        first_paths,
        second_paths
    ):
        """
        Combine pickup and delivery planning stages.

        The first position of the second path should match
        the final position of the first path.

        Different AMRs may have different path lengths when
        A* is used. Before combining stages, Stage 1 paths
        are padded so that all AMRs begin Stage 2 at the same
        global timestep.
        """

        combined_paths = {}

        robot_ids = set(
            first_paths.keys()
        )

        if robot_ids != set(
            second_paths.keys()
        ):
            raise RuntimeError(
                "Robot IDs do not match between "
                "planning stages."
            )

        # -----------------------------------------------------
        # Synchronize end of Stage 1
        # -----------------------------------------------------

        max_first_length = max(
            len(path)
            for path in first_paths.values()
        )

        for robot_id in first_paths:

            first_path = list(
                first_paths[robot_id]
            )

            second_path = list(
                second_paths[robot_id]
            )

            if not first_path:
                raise RuntimeError(
                    f"First path is empty for "
                    f"{robot_id}."
                )

            if not second_path:
                raise RuntimeError(
                    f"Second path is empty for "
                    f"{robot_id}."
                )

            if (
                first_path[-1]
                != second_path[0]
            ):
                raise RuntimeError(
                    f"Path continuity error for "
                    f"{robot_id}: "
                    f"{first_path[-1]} != "
                    f"{second_path[0]}"
                )

            # If this robot finished Stage 1 early,
            # it waits at its Stage 1 goal until every
            # robot reaches the Stage 2 start timestep.
            while (
                len(first_path)
                < max_first_length
            ):
                first_path.append(
                    first_path[-1]
                )

            combined_paths[robot_id] = (
                first_path
                + second_path[1:]
            )

        return combined_paths
