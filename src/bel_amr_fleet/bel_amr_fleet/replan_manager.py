from .replanner import Replanner


class ReplanManager:
    """
    Handles events that may require multi-AMR replanning.

    The manager acts as the bridge between robot/Nav2 events
    and the LaCAM-based dynamic replanning system.
    """

    REPLAN_EVENTS = {
        "BLOCKED",
        "PATH_OBSTRUCTED"
    }

    def __init__(self):
        self.replanner = Replanner()

    def handle_event(
        self,
        event,
        robot_id,
        map_file,
        width,
        height,
        robots,
        current_positions,
        time_limit=10
    ):
        """
        Process a fleet event and trigger replanning when required.

        Example:
            event = "BLOCKED"
            robot_id = "AMR-01"
        """

        event = event.upper()

        print(
            f"Event received: {robot_id} -> {event}"
        )

        if event not in self.REPLAN_EVENTS:
            print(
                f"No replanning required for event: {event}"
            )
            return None

        if robot_id not in current_positions:
            raise ValueError(
                f"Current position missing for {robot_id}"
            )

        print(
            f"Replanning triggered by {robot_id}: {event}"
        )

        new_paths = self.replanner.replan(
            map_file=map_file,
            width=width,
            height=height,
            robots=robots,
            current_positions=current_positions,
            time_limit=time_limit
        )

        return {
            "trigger_robot": robot_id,
            "event": event,
            "paths": new_paths
        }
