import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros

def generate_launch_description():
    pkg_share = get_package_share_directory('neatonav2')
    nav_config_path = LaunchConfiguration('nav_params')
    rviz_config_path = LaunchConfiguration('rviz_path')
    map_path = LaunchConfiguration('map_path')
    bringup_dir = get_package_share_directory('nav2_bringup')
    
    nav_yaml_params = DeclareLaunchArgument(
        'nav_params',
        default_value=os.path.join(pkg_share, 'config', 'nav_params.yaml'),
        description='.yaml file for navigation config')
    
    rviz_path = DeclareLaunchArgument(
        'rviz_path',
        default_value=os.path.join(pkg_share, 'rviz', 'nav.rviz'),
        description='path for .rviz file')
    
    map_yaml_path = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(pkg_share, 'map', 'map_save.yaml'),
        description='path for .rviz file')


    navigation = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nav2_bringup'), 'launch'),
         '/navigation_launch.py']),
      launch_arguments={'params': nav_config_path}.items()
      )
    localization = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('neatonav2'), 'launch'),
         '/localization_launch.py'])
      )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )
    
    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={'namespace': 'neato',
                              'use_namespace': 'true',
                              'map': map_path,
                              'params_file': nav_config_path}.items()),
        
        launch_ros.actions.Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          arguments=['-d', rviz_config_path]
    )
    ])   

    ld = LaunchDescription()
    ld.add_action(nav_yaml_params)
    ld.add_action(map_yaml_path)
    ld.add_action(rviz_path)
    ld.add_action(bringup_cmd_group)
    
    return ld