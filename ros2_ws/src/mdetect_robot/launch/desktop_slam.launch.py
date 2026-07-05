# Desktop-side mapping session: slam_toolbox (online async), the Nav2
# navigation stack, and RViz, all configured from this package's YAML files.
# Run alongside robot.launch.py on the robot to build a map while driving.
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('mdetect_robot')
    nav2 = get_package_share_directory('nav2_bringup')
    slam = get_package_share_directory('slam_toolbox')
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(share, 'config', 'slam_toolbox.yaml')
    # Custom RViz view: nav2 default plus the mdetect Goto panel
    # (type x_mm,y_mm,heading_deg to send the robot to a map coordinate).
    rviz_config = os.path.join(share, 'config', 'mdetect_view.rviz')

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'slam_params_file': slam_params}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'params_file': params,
                          'autostart': 'true', 'use_composition': 'False'}.items())
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'rviz_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'rviz_config': rviz_config}.items())

    return LaunchDescription([
        navigation,
        slam_node,
        rviz,
    ])
