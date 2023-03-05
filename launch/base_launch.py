import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='neatonav2').find('neatonav2')
    default_model_path = os.path.join(pkg_share,'urdf/neato.urdf')
    
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='both'
    )
    
    neato_node = launch_ros.actions.Node(
        package='neatonav2',
        executable='neato_node',
        name='neato_node',
        parameters=[{'laser_frame':'sensor_laser'}]
    )

    return launch.LaunchDescription([       
        robot_state_publisher_node,
        neato_node
    ])
