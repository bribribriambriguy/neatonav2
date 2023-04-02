import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='neatonav2').find('neatonav2')
    slam_config_path = os.path.join(pkg_share,'config/mapper_params_online_async.yaml')
    rviz_config_path = os.path.join(pkg_share,'rviz/map.rviz')


    slam = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('slam_toolbox'), 'launch'),
         '/online_async_launch.py']),
      launch_arguments=[{'params': slam_config_path},{'use_sim_time': False}]
      )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )

    robot_steering = launch_ros.actions.Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='robot_steering',
    )   

    return LaunchDescription([
      robot_steering,
      slam,
      rviz
   ])