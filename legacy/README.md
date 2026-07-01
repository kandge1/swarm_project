###

# Swarm Research

## Running the controller

Run the controller with:

```bash
ros2 launch control setpoint_controller.launch.py
```

## Visualizing in RViz2

### Run RViz2 with simulation time

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

### Add TF to the RViz2 display

* Click **Add**
* Select **TF**

### Set a goal using 2D Goal Pose

* Click **2D Goal Pose** in the top toolbar
* Click on the map to set position and drag to set orientation

###
