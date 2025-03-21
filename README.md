# ros2_web_control

This ROS 2 package provides a local web server that runs on the robot, offering a web-based interface for remote control and sensor monitoring. The frontend allows users to send velocity commands and view real-time data such as battery status, camera feed, and occupancy maps.

## Features
### 🚀 Remote Velocity Control
Control the robot’s movement in real-time via the web interface.

### 🗺️ Live OccupancyGrid Visualization
View the robot’s mapped environment dynamically.

### 📷 Live Camera Feed
Stream real-time video from the robot’s camera.

### 🔋 Live Battery Monitoring
Monitor the robot’s battery status directly from the web interface.

## Basic Usige

Clone Repository<br>
`git clone git@github.com:ivoheck/ros2_web_control.git`

Build Package<br>
`colcon build --packages-select ros2_web_control`

Start Web Server<br>
`ros2 launch ros2_web_control start_web_server.launch.py`

The web server is hosted on the robot's local IP at port 8000. The interface can be accessed via the following endpoint: /cmd_vel.
`http://<local-ip>:8000/cmd_vel/`

## Configuration<br>
The following values can be specified in the `web_control_config.yaml` file:<br>

Topic for publishing Twist messages (defaults to `/cmd_vel`):<br>
`twist_topic: /cmd_vel`<br>
<br>
Speed for the Twist Controller (defaults to `0.5`):<br>
`twist_speed: 0.5`<br>
<br>
Topic for receiving `BatteryState` messages (defaults to `/battery_state`)<br>
`battery_state_topic: /battery_state`<br>
<br>
Topic for receiving `OccupancyGrid` messages (defaults to `/map`):<br>
`occupancy_grid_topic: /map`<br>
<br>
Topic for receiving `Image` messages (defaults to `/camera/image_raw`)<br>
`image_topic: /camera/image_raw`<br>
<br>
