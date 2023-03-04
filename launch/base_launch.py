import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    #pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = 'src/neatonav2/urdf/neato.urdf'
    
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()
            
    #default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='both'
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    neato_node = launch_ros.actions.Node(
        package='neato',
        executable='neato_node',
        name='neato_node',
        parameters=[{'laser_frame':'sensor_laser'}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='laser_frame', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        #launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            #description='Absolute path to rviz config file'),
        
        robot_state_publisher_node,
        rviz_node,
        neato_node
    ])
