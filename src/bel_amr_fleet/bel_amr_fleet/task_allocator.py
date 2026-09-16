import math


class TaskAllocator:
    """
    Allocates warehouse tasks to suitable AMRs.

    Selection currently considers:
    1. Robot availability
    2. Minimum battery level
    3. Distance from robot to pickup location

    Priority support can be integrated later with the
    fleet priority-management module.
    """

    def __init__(self, minimum_battery=20.0):
        self.minimum_battery = minimum_battery

    def calculate_distance(self, robot_position, pickup_position):
        """
        Calculate Euclidean distance between the robot
        and the task pickup location.
        """

        robot_x, robot_y = robot_position
        pickup_x, pickup_y = pickup_position

        return math.sqrt(
            (pickup_x - robot_x) ** 2 +
            (pickup_y - robot_y) ** 2
        )

    def allocate_task(self, task, robots):
        """
        Select the most suitable AMR for a task.

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
            "battery": 85.0,
            "available": True
        }
        """

        if not robots:
            raise ValueError("No robots available for task allocation.")

        if "pickup" not in task:
            raise ValueError("Task must contain a pickup location.")

        candidates = []

        for robot in robots:
            if not robot.get("available", False):
                continue

            battery = robot.get("battery", 0.0)

            if battery < self.minimum_battery:
                continue

            distance = self.calculate_distance(
                robot["position"],
                task["pickup"]
            )

            candidates.append(
                {
                    "robot": robot,
                    "distance": distance
                }
            )

        if not candidates:
            return None

        best_candidate = min(
            candidates,
            key=lambda candidate: candidate["distance"]
        )

        selected_robot = best_candidate["robot"]

        allocation = {
            "task_id": task.get("task_id"),
            "robot_id": selected_robot["robot_id"],
            "pickup": task["pickup"],
            "goal": task.get("goal"),
            "distance_to_pickup": best_candidate["distance"]
        }

        return allocation
