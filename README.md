# neatonav2
A driver for the neato robot vacuum (Tested on ROS2 foxy)

This package includes a driver and launch file for slam using the slam_toolbox package and navigation using the nav_bringup package

Prerequistes:
Neato with USB control capabilities
A USB cable that properly conects to the neato
Ubuntu 20.04 or 22.04 installed
ROS2 Foxy (20.04) or ROS2 Humble (recomended due to some issue with nav2) (22.04) installed
foxy install: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
humble install: ttps://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Package Install:
mkdir -p ~/neato_ws/src
cd ~/neato_ws/src
git clone https://github.com/bribribriambriguy/neatonav2.git -b <ros-distro>-devel
cd ..
colcon build

Use:
Connect to neato with USB cable
cd ~/neato_ws
source install/setup.bash
Know what port the neato is on ex. /dev/ttyACM0

Run Driver:
ros2 launch neatonav2 base_launch.py neato_port:=<your_port> (default /dev/ttyACM0)

Run SLAM:
ros2 launch neatonav2 slam_launch.py

Run Nav2 stack:
ros2 launch neatonav2 nav_launch.py

ps. slam_launch.py and nav_launch.py do not include base_launch.py because it is assumed that the base_launch.py will be run on a 
computer on the robot and slam_launch.py and nav_launch.py are assumed to be running on a remote computer so base_launch.py and slam_launch.py
or nav_launch.py must be run separately

Info for people who want to have a computer on robot:
You can either run everything on the computer on the robot, or you can have a computer in the robot that just runs the base_launch.py
and a remote computer that handles SLAM and nav2

Usally the computer running on the robot ex.RPI will have limited proscessing power for SLAM and navigation algoriths so it is recomended to have a
remote computer do the heavy lifting

Setup for computer on robot:
It is pretty much the same thing as described before to connect the computer on the robot to the neato, but to connect the two computers through ROS2
so they can comunicate you must change the ROS_MASTER_URI on the remote computer, to the computer on the robot or vice versa
more info: http://wiki.ros.org/ROS/Tutorials/MultipleMachines


