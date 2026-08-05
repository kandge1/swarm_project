# ** THIS IS THE MAIN BRANCH FOR THE MYAGVS **
# ** FOR THE MYCOBOT, GO TO mycobot_main **

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
5. Build the package (this may take a while) `cd ~/swarm_ws` `colcon build`

## verify dependencies
### myagv_ros
1. On the myagv desktop, open the AGV_UI app (if the AGV_UI app does not appear on the desktop, you can open it by entering `python3 ~/AGV_UI/operations.py` in the terminal).
2. Click the on button next to Lasar Radar. The LiDAR hat should start spinning and a terminal should open.
3. Click the on button next to Basic Control, Keyboard Control. Another terminal should open. In this terminal, you can drive the motors using the keys (u i o j k l m , .). Make sure that the myagv is suspended on a block to prevent it from driving off of the table.
4. Click the blue Open Build Map button. This should open another terminal which opens RViz. RViz should display the current LiDAR readings and the map it built of the surrounding area.
5. Once you are finished testing the motors and LiDAR, go back to the AGV_UI app and stop the previous commands.
6. Once these are stopped, go down to the bottom left corner of the app by the Test section. Click the dropdown menu which says Motor and change it to 2D Camera. Next, click the blue Start Detection button. You should be able to view the output of the front facing camera.
7. Once you have completed these steps, you have verified that the myagv_ros package is working correctly.

## ros1_bridge
Make sure you have installed the ros1_bridge package. Follow the instructions on the project repo: [ros1_bridge] (https://github.com/ros2/ros1_bridge)


