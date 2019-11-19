# robodog

Robodog is a ROS package which implements a leader-follower problem, where the leader is a human whereas a follower is a robot.
 
The goal is to detect the human's strides using the robot's LIDAR scan readings.

This package enables the robot to localize itself, handle obstacle avoidance and follow a human being in the environment.

We are assume that there is only one human in the environment, and the environment is flat.

Authors:
- Almas Abdibayev
- Karim Itani
- Kizito Masaba

# How to run:

Delete default ip address from routing table.

sudo ip route del 'default ip address'

ssh into the robot

ssh husarian@'ip address'
password: husarion

cd to catking_ws and source bash file

cd catking_ws

source devel/setup.bash

run ROS master

screen -S core
roscore

run the node for the main board

screen -S tinker
rosrun rosbot_webui serial_bridge.sh
Press ctrl a+d for detaching

run the node for lidar

screen -S lidar
roslaunch rplidar_ros rplidar.launch
Press ctrl a+d for detaching

run the python file

python leader_follower.py
