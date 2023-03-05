import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='neatonav2').find('neatonav2')
    nav_config_path = os.path.join(pkg_share,'config/nav_params.yaml')
    rviz_config_path = os.path.join(pkg_share,'rviz/nav.rviz')


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

    return LaunchDescription([
      rviz,
      navigation,
      localization,
   ])
