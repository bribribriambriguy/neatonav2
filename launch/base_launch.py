import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch import LaunchDescription
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='neatonav2').find('neatonav2')
    default_model_path = os.path.join(pkg_share,'urdf/neato.urdf')
    base_frame_id = LaunchConfiguration('base_frame')
    odom_frame_id = LaunchConfiguration('odom_frame')
    laser_frame_id = LaunchConfiguration('laser_frame')
    neato_port = LaunchConfiguration('neato_port')
    neato_wheel_track = LaunchConfiguration('wheel_track')
    neato_wheel_radius = LaunchConfiguration('wheel_radius')
    neato_max_x = LaunchConfiguration('max_x')
    neato_max_z = LaunchConfiguration('max_z')
    enable_odom = LaunchConfiguration('enable_odom')
    enable_laser = LaunchConfiguration('enable_scan')
    enable_joint = LaunchConfiguration('enable_wheel_joint')
    
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()
        
    base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='neato base frame')
    
    odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='odometric frame')
    
    laser_frame = DeclareLaunchArgument(
        'laser_frame',
        default_value='sensor_laser',
        description='frame for lidar')
    
    port = DeclareLaunchArgument(
        'neato_port',
        default_value='/dev/ttyACM0',
        description='port for neato')
    
    wheel_track = DeclareLaunchArgument(
        'wheel_track',
        default_value = '0.240',
        description='the distance between the two wheels of the robot')
    
    wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0381',
        description='the wheel radius in meters'
    )
    
    max_x_speed = DeclareLaunchArgument(
        'max_x',
         default_value = '0.30',
        description='max speed both motors can move in m/s')
    
    max_z_speed = DeclareLaunchArgument(
        'max_z',
        default_value='0', # unlimited
        description='max theta speed in m/s')
    
    use_odom = DeclareLaunchArgument(
        'enable_odom',
        default_value='True',
        description='enable/disable odometry publishing and broadcasting')
    
    use_laser = DeclareLaunchArgument(
        'enable_scan',
        default_value='True',
        description='enable/disable scan publishing')  
    
    use_joint = DeclareLaunchArgument(
        'enable_wheel_joint',
        default_value='True',
        description='enable/disable live wheel joint publishing'
    )  
    
    bringup_cmd_group = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='both'
        ),
        
        launch_ros.actions.Node(
            condition=IfCondition(enable_joint),
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),
    
        launch_ros.actions.Node(
            package='neatonav2',
            executable='neato_node',
            name='neato_node',
            parameters=[{'base_frame': base_frame_id},
                        {'odom_frame': odom_frame_id},
                        {'laser_frame': laser_frame_id},
                        {'neato_port': neato_port},
                        {'wheel_track': neato_wheel_track},
                        {'wheel_radius':neato_wheel_radius},
                        {'max_x_speed': neato_max_x},
                        {'max_z_speed': neato_max_z},
                        {'enable_odom': enable_odom},
                        {'enable_scan': enable_laser},
                        {'enable_joint': enable_joint}]
        ),
    ])
    
    ld = LaunchDescription()
    ld.add_action(base_frame)
    ld.add_action(odom_frame)
    ld.add_action(laser_frame)
    ld.add_action(port)
    ld.add_action(wheel_track)
    ld.add_action(wheel_radius)
    ld.add_action(max_x_speed)
    ld.add_action(max_z_speed)
    ld.add_action(use_odom)
    ld.add_action(use_laser)
    ld.add_action(use_joint)
    ld.add_action(bringup_cmd_group)

    return ld
