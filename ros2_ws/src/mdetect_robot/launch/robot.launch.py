import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('mdetect_robot')
    base = os.path.join(share, 'config', 'base.yaml')
    lidar = os.path.join(share, 'config', 'lidar.yaml')
    urdf = os.path.join(share, 'urdf', 'mdetect_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value='/dev/arduino_mdetect'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/coin_d6'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': False}], output='screen'),
        Node(package='mdetect_robot', executable='serial_bridge', name='mdetect_serial_bridge',
             parameters=[base, {'port': LaunchConfiguration('arduino_port')}], output='screen'),
        Node(package='mdetect_robot', executable='coin_d6_lidar', name='coin_d6_lidar',
             parameters=[lidar, {'port': LaunchConfiguration('lidar_port')}], output='screen'),
        Node(package='mdetect_robot', executable='cmd_mux', name='mdetect_cmd_mux',
             parameters=[base], output='screen'),
    ])
