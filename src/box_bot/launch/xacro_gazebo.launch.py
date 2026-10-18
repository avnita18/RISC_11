import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('box_bot')

    xacro_file = os.path.join(
        package_path,
        'urdf',
        'robot_main_og.xacro'
    )

    # Convert Xacro → URDF
    robot_description = Command([
        'xacro',
        ' ',
        xacro_file
    ])

    # Start Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', 'empty.sdf'],
        output='screen'
    )

    # Start Gazebo GUI
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Publish robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
        output='screen'
    )

    return LaunchDescription([

        gazebo_server,

        gazebo_gui,

        robot_state_publisher,

        # Wait a little for Gazebo to start before spawning
        TimerAction(
            period=3.0,
            actions=[
                spawn_robot
            ]
        )

    ])