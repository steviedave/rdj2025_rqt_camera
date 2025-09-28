#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self, topic="/camera/image_raw", rate_hz=10, device=0):
        super().__init__('camera_publisher')
        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device)
        self.timer = self.create_timer(1.0 / float(rate_hz), self.timer_callback)

    def timer_callback(self):
        if not self.cap or not self.cap.isOpened():
            self.get_logger().warn("Camera not open")
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame read failed")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
