from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import threading
import uvicorn
from contextlib import asynccontextmanager

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import OccupancyGrid


#ROS 2 Node
class WebBrige(Node):
    def __init__(self):
        super().__init__('web_bridge')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel_unstamped', 1)

        # self.subscriber_battery_state = self.create_subscription(BatteryState, '/battery_state', self.battery_state_callback, 1)
        self.battery_state: BatteryState = None

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.map: OccupancyGrid = None

        self.logger = self.get_logger()

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def battery_state_callback(self, msg: BatteryState) -> None:
        self.battery_state = msg

    def publish_message(self, msg: Twist) -> None:
        self.publisher_cmd_vel.publish(msg)

    def get_twist_msg(self, buttonKey, speed) -> Twist:
        msg = Twist()
            
        if buttonKey == 1:
            msg.linear.x = speed
            msg.angular.z = speed

        elif buttonKey == 2:
            msg.linear.x = speed

        elif buttonKey == 3:
            msg.linear.x = speed
            msg.angular.z = -speed

        elif buttonKey == 4:
            msg.angular.z = speed

        elif buttonKey == 6:
            msg.angular.z = -speed

        elif buttonKey == 7:
            msg.linear.x = -speed
            msg.angular.z = speed

        elif buttonKey == 8:
            msg.linear.x = -speed

        elif buttonKey == 9:
            msg.linear.x = -speed
            msg.angular.z = -speed
        
        return msg


# API Models
class CmdVelButtonKey(BaseModel):
    key: int

class BatteryState(BaseModel):
    voltage: float
    percentage: float
    current: float
    charge: float
    capacity: float
    design_capacity: float

class Map(BaseModel):
    data: list
    width: int
    height: int
    resolution: float

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic
    rclpy.init()
    ros_node = WebBrige()

    def ros_thread():
        rclpy.spin(ros_node)

    thread = threading.Thread(target=ros_thread)
    thread.start()

    yield

    # Shutdown logic
    rclpy.shutdown()


# Fast API
class Backend():
    def __init__(self):
        self.app = FastAPI()

        app_path = "./../frontend/dist"
        self.app.mount("/", StaticFiles(directory=app_path, html=True), name="react-app")

        origins = [
            "http://localhost:5173",  
        ]

        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  
            allow_credentials=True,
            allow_methods=["*"], 
            allow_headers=["*"],  
        )

        self.ros_node = None

        # Get battery state from backend
        @self.app.get("/get_battery_state")
        async def read_root():

            batteryState = BatteryState(
                voltage=self.ros_node.battery_state.voltage or 0.0,
                percentage=self.ros_node.battery_state.percentage or 0.0,
                current=self.ros_node.battery_state.current or 0.0,
                charge=self.ros_node.battery_state.charge or 0.0,
                capacity=self.ros_node.battery_state.capacity or 0.0,
                design_capacity=self.ros_node.battery_state.design_capacity or 0.0
            )

            return batteryState
        
        # Get map from backend
        @self.app.get("/get_map")
        async def read_root():

            map = Map(
                data=self.ros_node.map.data,
                width=self.ros_node.map.info.width,
                height=self.ros_node.map.info.height,
                resolution=self.ros_node.map.info.resolution
            )

            return map

        # Get cmd vel commands from frontend
        @self.app.post("/cmd_vel_button_key/")
        async def read_root(cmd_vel_button_key: CmdVelButtonKey):
            self.ros_node.logger.info(f"Received key: {cmd_vel_button_key.key}")
            self.ros_node.publish_message(self.ros_node.get_twist_msg(cmd_vel_button_key.key, 0.5))
            # return {"message": "Received key successfully", "key": cmd_vel_button_key.key}


    def run(self, host="0.0.0.0", port=8000):
        # Starten des FastAPI Servers
        uvicorn.run(self.app, host=host, port=port)


def main():
    backend = Backend()
    backend.run()

# Run the FastAPI app
if __name__ == "__main__":
    main()