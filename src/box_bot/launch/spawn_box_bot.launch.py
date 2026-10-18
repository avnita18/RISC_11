
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('box_bot')

    urdf_path = os.path.join(
        package_path,
        'urdf',
        'box_bot.urdf'
    )

    return LaunchDescription([

        # Start Gazebo server
        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                '-s',
                '-r',
                'empty.sdf'
            ],
            output='screen'
        ),

        # Start Gazebo GUI
        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                '-g'
            ],
            output='screen'
        ),

        # Spawn the box robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'box_bot',
                '-file', urdf_path,
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        )
    ])

