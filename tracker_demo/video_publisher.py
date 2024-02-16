import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


VIDEO_OF_INTEREST = "demo.mp4"
FRAMES_PER_SECOND = 30

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.timer_ = self.create_timer(1.0 / FRAMES_PER_SECOND, self.publish_frame)
        self.cv_bridge_ = CvBridge()
        self.load_video_capture()
        self.get_logger().info("VideoPublisher has been started.")

    def publish_frame(self):
        ret, frame = self.video_capture_.read()
        if ret:
            image_msg = self.cv_bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_msg)
            return
        self.get_logger().info("Re-load video capture.")
        self.load_video_capture();     
    def load_video_capture(self):
        # ! getcwd relative to terminal current path 
        self.video_capture_ = cv2.VideoCapture(os.path.join(os.getcwd(), "resources", VIDEO_OF_INTEREST)) 

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
