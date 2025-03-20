import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.publish_image)  # Alle 1 Sekunde ein Bild senden

    def publish_image(self):
        # Lade ein Beispielbild (ändern auf eigenen Pfad)
        image_path = "test.jpg"
        frame = cv2.imread(image_path)

        if frame is None:
            self.get_logger().error(f"Bild konnte nicht geladen werden: {image_path}")
            return
        
        # Konvertiere das OpenCV-Bild zu einer ROS2-Image-Nachricht
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Veröffentliche das Bild
        self.publisher_.publish(ros_image)
        self.get_logger().info("Bild veröffentlicht!")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
