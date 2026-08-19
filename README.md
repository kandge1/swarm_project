# ** THIS IS THE MAIN BRANCH FOR THE MYAGV **
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

### ros1_bridge
Make sure you have installed the ros1_bridge package. Follow the instructions on the project repo: [ros1_bridge] (https://github.com/ros2/ros1_bridge)

** talker-listener demo **

Terminal A:
`source /opt/ros/noetic/setup.bash`
`roscore`

Terminal B:
`source /opt/ros/noetic/setup.bash`
`source /opt/ros/galactic/setup.bash`
`source ~/ros1_bridge/install/setup.bash`
`export ROS_MASTER_URI=http://localhost:11311`
`ros2 run ros1_bridge dynamic_bridge`

Terminal C:
`source /opt/ros/noetic/setup.bash`
`rosrun rospy_tutorials talker`

Terminal D:
`source /opt/ros/galactic/setup.bash`
`ros2 run demo_nodes_cpp listener`

Now Terminal C should be publishing messages and Terminal D should be receiving them.
If the demo above is functioning as expected, then the ros2_bridge package is working

### cyclonedds
* TODO: insert cyclonedds setup guide here *

### ros1_bridge - cyclonedds demo
This demo assumes that both ros1_bridge and cyclonedds have been installed and have been verified. This demo will show a workstation computer running ROS2 broadcasting /chatter messages over cyclonedds to the myagv. The myagv will then translate the ROS2 /chatter messages into ROS1 using the ros1_bridge. This demo can be modified to transmit any topic between the two devices, and it can be modified to transmit messages in either direction, not just from workstation to agv.

TERMINAL 1, AGV:
`source /opt/ros/noetic/setup.bash`
`source ~/myagv_ros/devel/setup.bash`
`export ROS_MASTER_URI=http://localhost:11311`
`roscore`

TERMINAL 2, AGV:
`source /opt/ros/noetic/setup.bash`
`source ~/myagv_ros/devel/setup.bash`
`export ROS_MASTER_URI=http://localhost:11311`
`rosparam load ~/swarm_ws/bridge.yaml`
`rostopic echo /chatter`

TERMINAL 3, AGV:
`source /opt/ros/noetic/setup.bash`
`source ~/myagv_ros/devel/setup.bash`
`source /opt/ros/galactic/setup.bash`
`source ~/ros1_bridge/install/setup.bash`
`export ROS_MASTER_URI=http://localhost:11311`
`cat > ~/swarm_ws/bridge.yaml <<'EOF' `
`topics: `
`  - topic: /chatter `
`  type: std_msgs/msg/String `
`  queue_size: 10 `
`EOF`
`ros2 run ros1_bridge parameter_bridge`

TERMINAL 4, WORKSTATION:
`source /opt/ros/jammy/setup.bash`
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp `
`export ROS_DOMAIN_ID=42 `
`export CYCLONEDDS_URI="file://$(ros2 pkg prefix swarm_network)/share/swarm_network/config/cyclonedds_jammy.xml"`
`ros2 run demo_nodes_cpp talker`

Terminal 2 on the AGV should be printing out the chatter published on terminal 4 on the workstation. This demo works by creating a configuration file which defines what topics the bridge will translate from ROS1 to ROS2. In this case, it only translates /chatter, but it can be easily modified to translate other topics, like /cmd_vel. The parameter_bridge will only translate the topics listed in the configuration file. The dynamic_bridge does not work for this demo as it will scan for nodes and services which do not exist, which causes it to quickly crash.
