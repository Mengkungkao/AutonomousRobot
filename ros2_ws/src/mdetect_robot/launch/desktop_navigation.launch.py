# Desktop-side navigation on a previously saved map: Nav2 localization (AMCL
# + map server, using the required 'map' argument), the navigation stack, and
# RViz. Run alongside robot.launch.py on the robot.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('mdetect_robot')
    nav2 = get_package_share_directory('nav2_bringup')
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    # Custom RViz view: nav2 default plus the mdetect Goto panel
    # (type x_mm,y_mm,heading_deg to send the robot to a map coordinate).
    rviz_config = os.path.join(share, 'config', 'mdetect_view.rviz')
    map_arg = LaunchConfiguration('map')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_arg, 'use_sim_time': 'false', 'params_file': params,
                          'autostart': 'true', 'use_composition': 'False'}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'params_file': params,
                          'autostart': 'true', 'use_composition': 'False'}.items())
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'rviz_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'rviz_config': rviz_config}.items())

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved map YAML'),
        localization,
        navigation,
        rviz,
    ])
