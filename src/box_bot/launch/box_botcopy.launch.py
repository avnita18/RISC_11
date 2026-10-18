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
        'final_bot.urdf'
    )

    return LaunchDescription([

        # Gazebo server
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

        # Gazebo GUI
        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                '-g'
            ],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'final_bot',
                '-file', urdf_path,
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen'
        )

    ])