import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort

class DeepSORT_Node(Node):
    def __init__(self):
            super().__init__("deepsort_tracking_node")
            
            self.tracker = DeepSort(max_age=30, nn_budget=100)

            self.subscription = self.create_subscription(
                Detection2DArray,
                'yolo_detections',
                self.listener_callback(),
                10 # history size
            )

            # Note, we publish our class and tracking id in form class_id:tracking_id
            self.publisher = self.create_publisher(
                Detection2DArray,
                'object_tracking',
                10)
    
    def listener_callback(self, msg):

        # init our output message
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header

        detections = []

        # for each detection in our input array from yolo
        for det in msg.detections:
            # decode our det into deepsort format
            x_cen = det.bbox.center.x
            y_cen = det.bbox.center.y
            width = detection.bbox.size_x
            height = detection.bbox.size_y

            x1 = x_cen - (width / 2.0)
            y1 = y_cen - (height / 2.0)

            # class label stuff if exists
            if detection.results:
                class_id = int(detection.results[0].id)      
                score = detection.results[0].score           
            else:
                class_id = -1
                score = 0.0

            # add detection to array
            detections.append([[x1, y1, width, height], score, int(class_id)])

        
        # now add our detections to our tracker
        tracks = tracker.update_tracks(detections, frame=frame)

        # For each valid track add to our message
        for track in tracks:        
            # If track hasn't been updated recently, we do not care about it for this instance
            if not track.is_confirmed() or track.time_since_update > 1:
                    continue        
            
            # Get Information From Track and Bounding Box Coordinates
            track_id = track.track_id
            x1, y1 width, height = track.to_ltwh()

            # add fields to our detection
            detection = Detection2D()
            detection.bbox.center.x = float(x1 + (width / 2.0))
            detection.bbox.center.y = float(y1 + (height / 2.0))
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)

            # get class_id, and score
            class_id = str(track.det_class)
            score = track.det_conf

            # store our combined class_id : tracking id, and our score
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(f'{class_id}:{track_id}')  
            hypothesis.score = confidence
            detection.results.append(hypothesis)

            detection_msg.detections.append(detection)

        self.publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeepSORT_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
            
