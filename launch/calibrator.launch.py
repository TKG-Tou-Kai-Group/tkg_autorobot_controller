import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    rs_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tkg_autorobot_controller'), 'launch'), '/rs_launch.py']),
            )

    calibrator = Node(
        package="tkg_autorobot_controller",
        executable="calibrator",
        name="calibrator",
        output="log",
    )

    return LaunchDescription([
        rs_launch,
        calibrator,
    ])

