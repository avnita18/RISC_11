import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('box_bot')

    xacro_file = os.path.join(
        package_path,
        'urdf',
        'robot_main.urdf.xacro'
    )

    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )

    ])