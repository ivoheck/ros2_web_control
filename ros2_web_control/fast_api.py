from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import threading
import uvicorn
from contextlib import asynccontextmanager

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import OccupancyGrid

from get_ip import get_local_ip

#ROS 2 Node
class WebBrige(Node):
    def __init__(self):
        super().__init__('web_bridge')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel_unstamped', 1)

        self.subscriber_battery_state = self.create_subscription(BatteryState, '/battery_state', self.battery_state_callback, 1)
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

class CmdVelButtonKeyModel(BaseModel):
    key: int

class BatteryStateModel(BaseModel):
    voltage: float
    percentage: float
    current: float
    charge: float
    capacity: float
    design_capacity: float

class MapModel(BaseModel):
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

    app.state.ros_node = ros_node

    yield

    # Shutdown logic
    rclpy.shutdown()

class Backend:
    def __init__(self):
        self.app = FastAPI(lifespan=lifespan)

        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  
            allow_credentials=True,
            allow_methods=["*"], 
            allow_headers=["*"],  
        )

        app_path = "./../frontend/dist"
        self.app.mount("/page/", StaticFiles(directory=app_path, html=True), name="react-app")


        # Routen in der Klasse definieren
        @self.app.post("/cmd_vel_button_key/")
        async def read_root(cmd_vel_button_key: CmdVelButtonKeyModel):
            ros_node = self.app.state.ros_node
            ros_node.logger.info(f"Received key: {cmd_vel_button_key.key}")
            ros_node.publish_message(ros_node.get_twist_msg(cmd_vel_button_key.key, 0.5))
            return {"message": "Received key successfully", "key": cmd_vel_button_key.key}
        

        # Get map from backend
        @self.app.get("/get_map/")
        async def read_root():
            map = self.app.state.ros_node.map
            return_map = None

            if map is not None:
                
                return_map = MapModel(
                    data=map.data,
                    width=map.info.width or 0,
                    height=map.info.height or 0,
                    resolution=map.info.resolution or 0.0
                )

            return return_map
        
        # Get battery state from backend
        @self.app.get("/get_battery_state/")
        async def read_root():
            battery_state = self.app.state.ros_node.battery_state
            return_battery_state = None

            if battery_state is not None:
            
                return_battery_state = BatteryStateModel(
                    voltage=battery_state.voltage or 0.0,
                    percentage=battery_state.percentage or 0.0,
                    current=battery_state.current or 0.0,
                    charge=battery_state.charge or 0.0,
                    capacity=battery_state.capacity or 0.0,
                    design_capacity=battery_state.design_capacity or 0.0
                )

            return return_battery_state
        

    def run(self, host="0.0.0.0", port=8000):
        print(f"running on: {get_local_ip()} port: {port} /page/")
        uvicorn.run(self.app, host=host, port=port)
        


def main():
    backend = Backend()
    backend.run()

if __name__ == "__main__":
    main()
