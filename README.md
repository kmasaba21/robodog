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

```
sudo ip route del 'default ip address'
```


ssh into the robot
```
ssh husarian@'ip address'
```
password: husarion


cd to catkin_ws and source bash file
```
cd catkin_ws
source devel/setup.bash
```



run ROS master
```
screen -S core
roscore &
```
Press Ctrl A+D for detaching


run the node for the main board
```
screen -S tinker
rosrun rosbot_webui serial_bridge.sh
Press ctrl a+d for detaching
```

run the node for lidar
```
screen -S lidar
roslaunch rplidar_ros rplidar.launch
Press ctrl a+d for detaching
```

run the main script for following a person and gathering their data as pickle dumps
```
python leader_follower.py
```

To run without PD
```
python robodog.py
```

Analysis scripts with short descriptions can be found in ./scripts/analysis

Most of the data that used and generated is located in ./data


