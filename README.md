# ** THIS IS THE MAIN BRANCH FOR THE MYAGVS **
# ** FOR THE MYCOBOT, GO TO mycobot/main **

## myagv swarm project setup
This project assumes that the following dependencies are met:
- [myagv_ros] (https://github.com/elephantrobotics/myagv_ros/tree/myagv_ros_2023Pi) (this should be installed by default on the myagv)
- [ROS 1 Noetic] (https://wiki.ros.org/noetic/Installation/Ubuntu) (this should be installed by default on the myagv)
- [ROS 2 Galactic] (https://docs.ros.org/en/galactic/Installation.html) (this should be installed by default on the myagv)
- [ros1_bridge] (https://github.com/ros2/ros1_bridge) (this is NOT installed by default on the myagv)
- [cyclonedds] (https://github.com/kandge1/swarm_project/blob/myagv/main/cyclone_dds_integration_log.md) (this is NOT installed by default on the myagv)

1. On the myagv, open a new ROS2 terminal (or open a generic terminal and enter: `source /opt/ros/galactic/setup.bash`)
2. Ensure that the myagv is connected to the internet (connecting to WiFi@OSU on the robots is surprisingly finicky) [WiFi@OSU Tutorial] (https://github.com/kandge1/swarm_project/blob/myagv/main/WiFi%40OSU_Tutorial.md)
3. Make a new project workspace `mkdir -p ~/swarm_ws/src` `cd ~/swarm_ws/src`
4. Clone this git repo into the src folder `git clone https://github.com/kandge1/swarm_project/myagv/main`
5. 
