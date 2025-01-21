from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os

def generate_launch_description():
    base_path = "/home/toto/ws/ros2_ws/src/colibus_simulator"
    pkg_path = os.path.join(base_path, 'urdf')
    xacro_file = os.path.join(pkg_path, 'vehicle.xacro')
    xacro_file_processed = xacro.process_file(xacro_file)
    robot_desc = xacro_file_processed.toxml()

    # Specify the world file
    world_path = PathJoinSubstitution([
        FindPackageShare("colibus_simulator"),
        'worlds',
        'example.world'
    ])

    return LaunchDescription([
        # Include the Gazebo server with the specified world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ]),
            launch_arguments={
                'world': world_path,
            }.items()
        ),

        # Set the robot description parameter
        SetParameter(name='robot_description', value=robot_desc),

        # Spawn the robot into Gazebo using the content from the 'robot_description'
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            arguments=[
                '-entity', 'vehicle',
                '-x', '0', '-y', '0', '-z', '0',
                '-robot_namespace', '',
                '-topic', '/robot_description'
            ],
            output='screen'
        ),

        # Robot state publisher using the 'robot_description' parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'robot_description': robot_desc}]
        ),
    ])