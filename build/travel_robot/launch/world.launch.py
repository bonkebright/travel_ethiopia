from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    world_path = os.path.join(
        get_package_share_directory("travel_robot"),
        "worlds",
        "ethiopia_relaxed.world"
    )

    gazebo_cmd = ExecuteProcess(
        cmd=["gz", "sim", world_path],
        output="screen"
    )

    return LaunchDescription([
        gazebo_cmd
    ])
