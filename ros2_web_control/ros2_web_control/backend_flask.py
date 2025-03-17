from flask import Flask, jsonify, request
import threading
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import OccupancyGrid


#ROS 2 Node
class WebBridge(Node):
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
class CmdVelButtonKey():
    key: int

class BatteryState():
    voltage: float
    percentage: float
    current: float
    charge: float
    capacity: float
    design_capacity: float

class Map():
    data: list
    width: int
    height: int
    resolution: float

class Backend():
    def __init__(self):
        self.app = Flask(__name__)
        self.ros_node = None
        
        # Wir verschieben den Decorator außerhalb der __init__-Methode
        self.app.before_first_request(self.startup_event)
        self.app.teardown_appcontext(self.shutdown_event)

        # Get battery state from backend
        @self.app.route("/get_battery_state", methods=["GET"])
        def get_battery_state():
            battery_state = BatteryState(
                voltage=self.ros_node.battery_state.voltage or 0.0,
                percentage=self.ros_node.battery_state.percentage or 0.0,
                current=self.ros_node.battery_state.current or 0.0,
                charge=self.ros_node.battery_state.charge or 0.0,
                capacity=self.ros_node.battery_state.capacity or 0.0,
                design_capacity=self.ros_node.battery_state.design_capacity or 0.0
            )
            return jsonify(battery_state.dict())

        # Get map from backend
        @self.app.route("/get_map", methods=["GET"])
        def get_map():
            map_data = Map(
                data=self.ros_node.map.data,
                width=self.ros_node.map.info.width,
                height=self.ros_node.map.info.height,
                resolution=self.ros_node.map.info.resolution
            )
            return jsonify(map_data.dict())

        # Get cmd vel commands from frontend
        @self.app.route("/cmd_vel_button_key/", methods=["POST"])
        def cmd_vel_button_key():
            try:
                # Validieren und in CmdVelButtonKey umwandeln
                cmd_vel_button_key = CmdVelButtonKey(**request.json)
            except ValidationError as e:
                return jsonify({"error": e.errors()}), 400
            
            self.ros_node.logger.info(f"Received key: {cmd_vel_button_key.key}")
            self.ros_node.publish_message(self.ros_node.get_twist_msg(cmd_vel_button_key.key, 0.5))
            return jsonify({"message": "Received key successfully", "key": cmd_vel_button_key.key})

        # Serve frontend static files (optional)
        self.app.static_folder = "./../frontend/dist"
        self.app.add_url_rule("/", endpoint="index", view_func=self.index)

    def startup_event(self):
        rclpy.init()
        self.ros_node = WebBridge()  # WebBridge-Klasse anpassen
        def ros_thread():
            rclpy.spin(self.ros_node)
        thread = threading.Thread(target=ros_thread)
        thread.start()

    def shutdown_event(self, exception=None):
        rclpy.shutdown()

    def index(self):
        return self.app.send_static_file('index.html')


def main():
    backend = Backend()
    backend.app.run(host="0.0.0.0", port=8000)


# Run the FastAPI app
if __name__ == "__main__":
    main()