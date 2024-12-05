import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from mmdet3d.apis import init_model, inference_detector
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import tf_transformations 
import torch
import math
from time import sleep
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
class LiDARDetectionNode(Node):
    def __init__(self):
        super().__init__('lidar_detection_node')

        # Initialize MMDetection3D model
        self.config_file = 'pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py'  # Path to MMDetection3D config
        self.checkpoint_file = 'hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth'  # Path to checkpoint file
        self.device = 'cuda:0'  # Device for inference (use 'cpu' if no GPU available)
        self.publisher = self.create_publisher(Marker, 'bounding_box_edges', 10)
        # self.timer = self.create_timer(1.0, self.publish_bounding_box_edges)
        # Initialize the model
        self.model = init_model(self.config_file, self.checkpoint_file, device=self.device)

        # ROS2 subscriber for LiDAR data (using the updated topic '/front_3d_lidar/point_cloud')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  # Updated LiDAR topic
            self.lidar_callback,
            10
        )

        # Preprocess and inference
        self.bridge = None  # No need for CvBridge here
        # ROS2 Publisher for bounding boxes as markers
        self.marker_pub = self.create_publisher(MarkerArray, 'lidar_bounding_boxes', 10)

    def lidar_callback(self, msg):
        self.get_logger().info("Received point cloud data")
        try:
            # Convert PointCloud2 message to numpy array
            boxes = None
            scores = None
            labels = None
            points = self.point_cloud2_to_array(msg)

            # Check if points are empty
            if len(points) == 0:
                self.get_logger().warn("Received emptypoint cloud!")
                return

            # self.get_logger().info(f"Point cloud size: {points.shape}")

            # Ensure we have the correct shape for inference (N, 3) for XYZ coordinates
            # if len(points.shape) != 2 or points.shape[1] != 3:
            #     raise ValueError(f"Point cloud data is not in the expected shape: {points.shape}")

            # Run inference on the point cloud
            self.get_logger().info("Running inference...")
            result = inference_detector(self.model, points)

            self.get_logger().info("Inference completed.")

            # Extract the 3D bounding boxes, scores, and labels
            # pred_instances = result[0].pred_instances_3d
            # boxes = pred_instances.bboxes_3d      # 3D bounding boxes (N, 7) - x,y,z,l,w,h,yaw
            # scores = pred_instances.scores_3d     # Confidence scores (N,)
            # labels = pred_instances.labels_3d     # Class labels (N,)
            if hasattr(result[0], 'pred_instances_3d'):
                pred_instances = result[0].pred_instances_3d

                # Check if the pred_instances_3d object has the attributes 'bboxes_3d', 'scores_3d', and 'labels_3d'
                if hasattr(pred_instances, 'bboxes_3d'):
                    boxes = pred_instances.bboxes_3d  # 3D bounding boxes (N, 7) - x, y, z, l, w, h, yaw

                if hasattr(pred_instances, 'scores_3d'):
                    scores = pred_instances.scores_3d  # Confidence scores (N,)

                if hasattr(pred_instances, 'labels_3d'):
                    labels = pred_instances.labels_3d  # Labels (Class IDs)
            print(boxes)
            if(boxes):
                self.publish_bounding_box_edges(boxes)
            # Print the results (boxes, scores, and labels)
            self.get_logger().info("Bounding Boxes (x, y, z, l, w, h, yaw):")
            self.get_logger().info(str(boxes))

            self.get_logger().info("Confidence Scores:")
            self.get_logger().info(str(scores))

            self.get_logger().info("Labels (Class IDs):")
            self.get_logger().info(str(labels))
            print("###############################")
        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")
            sleep(10)

    def point_cloud2_to_array(self, msg):
        """ Convert ROS2 PointCloud2 message to numpy array """
        # Read the points from the PointCloud2 message
        points_list = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            # Add intensity as 0 since it's not in the original data
            point = [p[0], p[1], p[2], 0.0]
            points_list.append(point)
        pc_array = np.array(points_list, dtype=np.float32)
        return pc_array
    def compute_rotated_corners(self, x, y, z, l, w, h, yaw):
        """Compute the 3D corners of the bounding box."""
        # Half dimensions
        dx = l / 2
        dy = w / 2

        # Corners in local frame before rotation
        corners = [
            [dx, dy],  # Front-right
            [-dx, dy],  # Front-left
            [-dx, -dy],  # Back-left
            [dx, -dy]   # Back-right
        ]

        # Rotate corners around yaw angle
        rotated_corners = [
            [
                x + c[0] * math.cos(yaw) - c[1] * math.sin(yaw),  # x-coordinate
                y + c[0] * math.sin(yaw) + c[1] * math.cos(yaw),  # y-coordinate
                z                                               # z-coordinate
            ]
            for c in corners
        ]

        # Add the top and bottom corners
        bottom_corners = [Point(x=c[0], y=c[1], z=z) for c in rotated_corners]
        top_corners = [Point(x=c[0], y=c[1], z=z + h) for c in rotated_corners]
        return bottom_corners, top_corners

    def publish_bounding_box_edges(self, points):
        # Example LiDAR data: x, y, z (center), l, w, h (dimensions), yaw (orientation)
        # x, y, z, l, w, h, yaw =  2.1005,  1.7373, -0.9114,  3.3336,  1.5157,  1.4120,  4.6763

        print("HEYHEYHEY")
        box = points.tensor[0]  # First bounding box
        x, y, z = box[0].item()/2, box[1].item()/2, box[2].item()/2  # Center coordinates
        l, w, h = box[3].item()/2, box[4].item()/2, box[5].item()/2  # Dimensions
        yaw = box[6].item()  # Orientation angle in radians
        # Get the corners of the bounding box
        print(x,y,z,l,w,h,yaw)
        bottom_corners, top_corners = self.compute_rotated_corners(x,y,z,l,w,h,yaw)
        print("HeyHey")
        # Define edges
        edges = [
            # Bottom edges
            (bottom_corners[0], bottom_corners[1]),
            (bottom_corners[1], bottom_corners[2]),
            (bottom_corners[2], bottom_corners[3]),
            (bottom_corners[3], bottom_corners[0]),
            # Top edges
            (top_corners[0], top_corners[1]),
            (top_corners[1], top_corners[2]),
            (top_corners[2], top_corners[3]),
            (top_corners[3], top_corners[0]),
            # Vertical edges
            (bottom_corners[0], top_corners[0]),
            (bottom_corners[1], top_corners[1]),
            (bottom_corners[2], top_corners[2]),
            (bottom_corners[3], top_corners[3])
        ]

        # Create the Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bounding_box"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Add edges to the marker
        for edge in edges:
            marker.points.extend(edge)

        # Set the color and scale
        marker.scale.x = 0.05  # Line thickness
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.publisher.publish(marker)

    def publish_bounding_boxes(self, boxes, scores, labels):
        # Create a MarkerArray to publish to RViz
        marker_array = MarkerArray()

        for idx, box in enumerate(boxes):
            # Create a Marker for each box
            marker = Marker()
            marker.header.frame_id = "base_link"  # or whatever your fixed frame is
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "bounding_boxes"
            marker.id = idx
            marker.type = Marker.CUBE

            # Set position and orientation
            marker.pose.position.x = float(box[0]/2)
            marker.pose.position.y = float(box[1]/2)
            marker.pose.position.z = float(box[2]/2)
            marker.pose.orientation.w = 1.0  # Assuming no rotation (this may need adjustment based on your model's yaw)

            # Set scale (length, width, height)
            marker.scale.x = float(box[3]/2 ) # length
            marker.scale.y = float(box[4]/2)  # width
            marker.scale.z = float(box[5]/2)  # height

            # Set color (you can use a color map based on the score or label)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Transparency

            # Set lifetime
            marker.lifetime = Duration(seconds=5).to_msg()

            # Add marker to MarkerArray
            marker_array.markers.append(marker)

        # Publish the MarkerArray to RViz
        self.marker_pub.publish(marker_array)


def main(args=None):
    print("in main")
    rclpy.init(args=args)
    node = LiDARDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

