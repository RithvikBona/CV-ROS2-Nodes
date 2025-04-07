import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image  # Point Cloud and Image messages
import sensor_msgs.point_cloud2 as pc2  # pointcloud util


import numpy as np
from cv_bridge import CvBridge


class PointCloudToRGBD(Node):
    def __init__(self):
        super().__init__("point_cloud_to_rgbd_node")
        self.bridge = CvBridge()

        self.rgb_pub = self.create_publisher(Image, "rgb_data", 10)
        self.depth_pub = self.create_publisher(Image, "depth_data", 10)

        self.subscription = self.create_subscription(
            PointCloud2,
            'point_data',
            self.listener_callback(),
            10 # history size
        )

    def listener_callback(self, msg):

        # camera params
        width, height = 640, 480  # Image resolution
        fx, fy = 525.0, 525.0  # Focal length in pixels
        x0, y0 = width / 2, height / 2  # principle point

        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)))

        # if no points to process don't publish anything
        if not np.size(points):
            return
        
        # init images, can change number type
        depth_img = np.zeros((height, width), dtype=np.float32)
        rgb_img = np.zeros((height, width, 3), dtype=np.uint8)

        # iterate through points and extract data
        for point in points:
            x, y, z, color = point
            
            # valid depth
            if z > 0:

                # project 3D x,y,z to 2D U,V using camera calibration eqs
                u = int(x0 + fx * x / z)
                v = int(y0 + fy * y / z)
                
                # make sure within actual image bounds
                if 0 <= u < width and 0 <= v < height:
                    depth_img[v, u] = z
                    # extract color channels
                    r = (int(color) >> 16) & 0xFF
                    g = (int(color) >> 8) & 0xFF
                    b = int(color) & 0xFF
                    rgb_img[v, u] = [r, g, b]
        
        # publish messages, change the encoding if we change number type
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="32FC1")


                


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToRGBD()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
