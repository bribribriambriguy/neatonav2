import launch
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch.actions import ExecuteProcess
import launch_ros
import os
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='neatonav2').find('neatonav2')
    map_path = os.path.join(pkg_share,'map/map_save.yaml')

    map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'use_sim_time': False},{'yaml_filename': map_path}]
    )
    
    amcl = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{'use_sim_time': False},{'base_frame_id': 'base_link'}]
    )
    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}]
            )
    ld = LaunchDescription()
    
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
