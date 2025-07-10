import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get directories for the robot description
    description_pkg = get_package_share_directory('turtlebot4_description')
    xacro_file_path = os.path.join(description_pkg, 'urdf', 'standard/turtlebot4.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file_path])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo simulation node
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_path = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    
    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={'gz_args': 'empty.sdf'}.items()
    )

    # Create entity gazebo node
    create_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        output='screen'
    )

    # RViz node
    config_path = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'config_turltebot4.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_path],
        output='screen'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim_node,
        create_entity_node,
        rviz_node
    ])