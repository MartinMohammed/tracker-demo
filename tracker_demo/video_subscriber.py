import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, BoundingBox2D, Pose2D, Point2D
from cv_bridge import CvBridge
import time
from typing import Tuple

INITIAL_BOUNDING_BOX = (458, 101, 105, 60)


def get_xywh_from_bbox_2d(bbox: BoundingBox2D) -> Tuple[int, int, int, int]:
    """
    Calculate the coordinates and dimensions of a 2D bounding box.

    Parameters:
    - bbox (BoundingBox2D): The 2D bounding box object containing center position and size.

    Returns:
    - Tuple[int, int, int, int]: A tuple containing four integers representing the x-coordinate,
      y-coordinate, width, and height of the bounding box.
    """
    center = bbox.center.position
    size_x = bbox.size_x
    size_y = bbox.size_y

    x = int(center.x - (size_x / 2))
    y = int(center.y - (size_y / 2))
    w = int(size_x)
    h = int(size_y)

    return x, y, w, h


class VideoSubscriberNode(Node):
    def __init__(self):
        super().__init__("video_subscriber")
        self.tracker = cv2.TrackerMIL_create()
        self.tracker_was_init = False
        self.new_bbox: Tuple[int, int, int, int] = None
        self.prev_time = time.time()
        self.fps = 0

        self.image_subscription = self.create_subscription(
            Image, "image", self.image_callback, 10
        )

        self.detection_subscription = self.create_subscription(
            Detection2D, "detection/detector", self.detection_callback, 10
        )

        self.detection_publisher = self.create_publisher(
            Detection2D, "detection/tracker", 10
        )
        self.cv_bridge_ = CvBridge()
        self.get_logger().info("VideoSubscriber has been started.")

    def publish_detection(self, bbox: Tuple[int, int, int, int]):
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
            frame = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if not self.tracker_was_init:
                self.tracker.init(frame, INITIAL_BOUNDING_BOX)
                self.tracker_was_init = True
            if self.new_bbox:
                self.tracker.init(frame, self.new_bbox)
                self.new_bbox = None

            ok, tracker_bbox = self.tracker.update(frame)
            if ok:
                pt1 = (tracker_bbox[0], tracker_bbox[1])
                pt2 = (
                    tracker_bbox[0] + tracker_bbox[2],
                    tracker_bbox[1] + tracker_bbox[3],
                )
                cv2.rectangle(frame, pt1=pt1, pt2=pt2, color=(0, 255, 0), thickness=2)
                self.publish_detection(bbox=tracker_bbox)
            else:
                self.get_logger().info("Tracking failure")

            # Calculate FPS
            current_time = time.time()
            self.fps = 1 / (current_time - self.prev_time)
            self.prev_time = current_time

            # Display FPS on frame
            cv2.putText(
                frame,
                f"FPS: {int(self.fps)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Video Subscriber", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))

    def detection_callback(self, msg: Detection2D):
        if msg.bbox:
            self.get_logger().info("Got detection from detector.")
            self.new_bbox = get_xywh_from_bbox_2d(bbox=msg.bbox)


def main(args=None):
    rclpy.init(args=args)
    node = VideoSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
