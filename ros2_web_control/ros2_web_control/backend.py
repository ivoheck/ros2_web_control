from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
import threading
import uvicorn

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

#ROS 2
class WebBrige(Node):
    def __init__(self):
        super().__init__('fastapi_ros_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_unstamped', 1)

    def publish_message(self, msg: Twist):
        self.publisher.publish(msg)



class CmdVelButtonKey(BaseModel):
    key: int

class Backend():
    def __init__(self):
        self.app = FastAPI()

        origins = [
            "http://localhost:5173",  
        ]

        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=origins,  
            allow_credentials=True,
            allow_methods=["*"], 
            allow_headers=["*"],  
        )

        self.ros_node = None

        @self.app.on_event("startup")
        def startup_event():
            rclpy.init()  
            self.ros_node = WebBrige()

            def ros_thread():
                rclpy.spin(self.ros_node)

            thread = threading.Thread(target=ros_thread)
            thread.start()

        @self.app.on_event("shutdown")
        def shutdown_event():
            rclpy.shutdown() 

        @self.app.post("/cmd_vel_button_key")
        async def read_root(cmd_vel_button_key: CmdVelButtonKey):
            print(cmd_vel_button_key.key)
            msg = Twist()
            msg.linear.x = 0.1
            self.ros_node.publish_message(msg)
            # return {"message": "Received key successfully", "key": cmd_vel_button_key.key}


# Run the FastAPI app
if __name__ == "__main__":
    backend = Backend() 
    uvicorn.run(backend.app, host="0.0.0.0", port=8000)