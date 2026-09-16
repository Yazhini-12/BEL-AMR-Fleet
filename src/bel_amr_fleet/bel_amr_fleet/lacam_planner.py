import os
import subprocess


class LaCAMPlanner:
    """
    Interface between the BEL AMR Fleet system and the LaCAM
    multi-agent pathfinding solver.
    """

    def __init__(self):
        # Location of the compiled LaCAM executable
        self.lacam_executable = os.path.expanduser(
            "~/warehouse_ws/algorithms/lacam0/build/main"
        )

        if not os.path.isfile(self.lacam_executable):
            raise FileNotFoundError(
                f"LaCAM executable not found: {self.lacam_executable}"
            )

        print(f"LaCAM planner ready: {self.lacam_executable}")

    def create_scenario(
        self,
        map_name,
        width,
        height,
        robots,
        output_file
    ):
        """
        Create a LaCAM-compatible MovingAI scenario file.

        robots format:
        [
            {
                "robot_id": "AMR-01",
                "start": (x, y),
                "goal": (x, y)
            }
        ]
        """

        with open(output_file, "w") as file:
            file.write("version 1\n")

            for robot in robots:
                start_x, start_y = robot["start"]
                goal_x, goal_y = robot["goal"]

                file.write(
                    f"0\t"
                    f"{map_name}\t"
                    f"{width}\t"
                    f"{height}\t"
                    f"{start_x}\t"
                    f"{start_y}\t"
                    f"{goal_x}\t"
                    f"{goal_y}\t"
                    f"0\n"
                )

        print(f"Scenario created: {output_file}")

    def run_lacam(
        self,
        map_file,
        scenario_file,
        number_of_robots,
        output_file="/tmp/bel_lacam_result.txt",
        time_limit=10
    ):
        """
        Execute LaCAM and generate a multi-agent path solution.
        """

        command = [
            self.lacam_executable,
            "--map", map_file,
            "--scen", scenario_file,
            "--num", str(number_of_robots),
            "--time_limit_sec", str(time_limit),
            "--output", output_file,
            "--verbose", "1"
        ]

        print("Running LaCAM...")
        print("Command:", " ".join(command))

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=True
            )

        except subprocess.CalledProcessError as error:
            print("LaCAM execution failed.")

            if error.stdout:
                print("STDOUT:")
                print(error.stdout)

            if error.stderr:
                print("STDERR:")
                print(error.stderr)

            raise

        if result.stdout:
            print(result.stdout)

        if result.stderr:
            print(result.stderr)

        if not os.path.isfile(output_file):
            raise FileNotFoundError(
                f"LaCAM did not create the expected result file: {output_file}"
            )

        print(f"LaCAM solution saved: {output_file}")

        return output_file
