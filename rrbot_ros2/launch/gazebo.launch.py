import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

# this is the function launch  system will look for
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_file_name = "rrbot_ros2.urdf"
    package_name = "rrbot_ros2"
    world_file_name = "rrbot.world"

    pkg_path = get_package_share_directory(package_name)
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    urdf_file = os.path.join(pkg_path, "urdf", robot_file_name)
    world_path = os.path.join(pkg_path, "worlds", world_file_name)

    # Start Gazebo server 
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )
    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    jointcmd_publisher = Node(
        package='rrbot_ros2',
        executable='rrbot2_command',
        # output='screen',
    )

    joint_publisher = Node(
        package='rrbot_ros2',
        executable='rrbot2_main',
        # output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        parameters=[],
        arguments=["-entity", "rrbot_ros2", "-file", urdf_file],
        output="screen",
    )

    # create and return launch description object
    return LaunchDescription(
        [
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            spawn_entity,
            joint_publisher,
            jointcmd_publisher
        ]
    )
