import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, BoundingBox2D, Pose2D, Point2D
from cv_bridge import CvBridge

INITIAL_BOUNDING_BOX = (458, 101, 105, 60)

class VideoSubscriberNode(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.tracker = cv2.TrackerMIL_create()
        self.tracker_was_init = False
        self.new_bbox = None

        self.image_subscription = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10)
        
        self.detection_subscription = self.create_subscription(
            Detection2D, 
            "detection",
            self.detection_callback, 
            10)

        self.detection_publisher = self.create_publisher(Detection2D, 'detection/tracker', 10)
        self.cv_bridge_ = CvBridge()
        self.get_logger().info("VideoSubscriber has been started.")

    def publish_detection(self, bbox):
        x, y, w, h = bbox 
        detection_msg = Detection2D()
        bbox_msg = BoundingBox2D()

        pose_msg = Pose2D()
        point_msg = Point2D()

        # Convert bbox coordinates to integers
        x, y, w, h = int(x), int(y), int(w), int(h)

        # Cx bbox
        point_msg.x = float(x + w // 2)

        # Cy bbox
        point_msg.y = float(y + h // 2)

        pose_msg.position = point_msg
        bbox_msg.center = pose_msg  

        # Width of bbox
        bbox_msg.size_x = float(w)

        # Height of bbox
        bbox_msg.size_y = float(h)

        detection_msg.bbox = bbox_msg
        detection_msg.id = str(4)  # Change to integer if appropriate

        self.detection_publisher.publish(detection_msg)

    def image_callback(self, msg):
        try:
            frame = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if not self.tracker_was_init:
                self.tracker.init(frame, INITIAL_BOUNDING_BOX)
                self.tracker_was_init = True
            if self.new_bbox:
                self.tracker.init(frame, self.new_bbox)
                self.new_bbox = None

            else:
                ok, bbox = self.tracker.update(frame)
                if ok:
                    cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), 
                                  (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])), 
                                  (0, 255, 0), 2)
                    self.publish_detection(bbox)
                else:
                    self.get_logger().info("Tracking failure")

            cv2.imshow('Video Subscriber', frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))

    def detection_callback(self, msg):
        if msg.bbox:
            self.new_bbox = (msg.bbox.center.x, msg.bbox.center.y, 
                             msg.bbox.size_x, msg.bbox.size_y)

def main(args=None):
    rclpy.init(args=args)
    node = VideoSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
