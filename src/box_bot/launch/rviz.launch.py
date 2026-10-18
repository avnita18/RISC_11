import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('box_bot')

    urdf_path = os.path.join(
        package_path,
        'urdf',
        'final_bot.urdf'
    )

    with open(urdf_path, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([

        # Publishes the robot's TF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description
                }
            ],
            output='screen'
        ),

        # Starts RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])