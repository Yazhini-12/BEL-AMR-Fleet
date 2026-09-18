import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_share = get_package_share_directory('bel_amr_fleet')

    # Installed warehouse world
    world_file = os.path.join(
        package_share,
        'worlds',
        'tugbot_warehouse_old.sdf'
    )

    # Source model directory in your workspace
    models_dir = os.path.expanduser(
        '~/bel_amr_ws/src/bel_amr_fleet/models'
    )

    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_dir
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': f'-r {world_file}'
            }.items()
        ),
    ])