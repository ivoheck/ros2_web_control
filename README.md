# ros2_web_control

This ROS 2 package provides a local web server that runs on the robot, offering a web-based interface for remote control and sensor monitoring. The frontend allows users to send velocity commands and view real-time data such as battery status, camera feed, and occupancy maps.
  
## Features
### 🚀 Remote Velocity Control
- Control the robot’s movement in real-time via the web interface.
<br>

<div align="center">
    <img src="https://github.com/user-attachments/assets/cb975c30-fc79-4488-9135-2ba50efbeb55" alt="Screenshot_20250322-200413" width="400"/>
</div>

### 🗺️ Live OccupancyGrid Visualization
- View the robot’s mapped environment dynamically.
<br>

<div align="center">
<img src="https://github.com/user-attachments/assets/99624b03-7653-4046-a14d-083fde5c4d3c" alt="Screenshot_20250322-200413" width="400"/>
</div>

### 📷 Live Camera Feed
- Stream real-time video from the robot’s camera.
<br>

<div align="center">
<img src="https://github.com/user-attachments/assets/7551de70-cca0-446d-ab81-b2d5ad37774b" alt="Screenshot_20250322-200413" width="400"/>
</div>

### 🔋 Live Battery Monitoring
- Monitor the robot’s battery status directly from the web interface.
<br>

<div align="center">
<img src="https://github.com/user-attachments/assets/b7ae1e3e-df43-46d4-b413-3e179dcc4db1" alt="Screenshot_20250322-200413" width="400"/>
</div>

<br>

## Basic Usige

Clone Repository<br>
`git clone git@github.com:ivoheck/ros2_web_control.git`

Build Package<br>
`colcon build --packages-select ros2_web_control`

Install Required Packages<br>
`sudo apt install python3-fastapi`<br>
`sudo apt install python3-netifaces`<br>
`sudo apt install python3-opencv`<br>

Start Web Server<br>
`ros2 launch ros2_web_control start_web_server.launch.py`

The web server is hosted on the robot's local IP at port 8000. The interface can be accessed via the following endpoint: /cmd_vel
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
