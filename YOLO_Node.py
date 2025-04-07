import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from std_msgs.msg import Header

import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLO_Node(Node):
    def __init__(self):
            super().__init__("yolo_prediction_node")

            self.bridge = CvBridge()
            self.model = YOLO('model/best.pt')

            self.subscription = self.create_subscription(
                Image,
                'image_data',
                self.listener_callback(),
                10 # history size
            )

            self.publisher = self.create_publisher(
                Detection2DArray,
                'yolo_detections',
                10)

        def listener_callback(self, msg):

            # init our output message
            detection_msg = Detection2DArray()
            detection_msg.header = msg.header

            # convert to cv2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # create predictions
            results = self.model.predict(cv_image, conf=0.5, verbose=False)

            # add each detection to our message
            for result in results:
                for r in result.boxes.data.tolist():
                    x1, y1, x2, y2, score, class_id = r

                    # add fields to our detection
                    detection = Detection2D()
                    detection.bbox.center.x = float((x1 + x2) / 2.0)
                    detection.bbox.center.y = float((y1 + y2) / 2.0)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = str(class_id)
                    hypothesis.score = score
                    detection.results.append(hypothesis)

                    detection_msg.detections.append(detection)

            self.publisher.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLO_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
