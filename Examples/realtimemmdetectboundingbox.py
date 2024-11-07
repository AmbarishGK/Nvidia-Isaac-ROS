import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from mmdet3d.apis import init_model, inference_detector
import numpy as np
import torch

class LiDARDetectionNode(Node):
    def __init__(self):
        super().__init__('lidar_detection_node')

        # Initialize MMDetection3D model
        self.config_file = 'pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py'  # Path to MMDetection3D config
        self.checkpoint_file = 'hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth'  # Path to checkpoint file
        self.device = 'cuda:0'  # Device for inference (use 'cpu' if no GPU available)
        
        # Initialize the model
        self.model = init_model(self.config_file, self.checkpoint_file, device=self.device)

        # ROS2 subscriber for LiDAR data (using the updated topic '/front_3d_lidar/point_cloud')
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/front_3d_lidar/point_cloud',  # Updated LiDAR topic
            self.lidar_callback,
            10
        )

        # Preprocess and inference
        self.bridge = None  # No need for CvBridge here

    def lidar_callback(self, msg):
        self.get_logger().info("Received point cloud data")
        try:
            # Convert PointCloud2 message to numpy array

            points = self.point_cloud2_to_array(msg)

            # Check if points are empty
            if len(points) == 0:
                self.get_logger().warn("Received empty point cloud!")
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
            pred_instances = result[0].pred_instances_3d
            boxes = pred_instances.bboxes_3d      # 3D bounding boxes (N, 7) - x,y,z,l,w,h,yaw
            scores = pred_instances.scores_3d     # Confidence scores (N,)
            labels = pred_instances.labels_3d     # Class labels (N,)

            # Print the results (boxes, scores, and labels)
            self.get_logger().info("Bounding Boxes (x, y, z, l, w, h, yaw):")
            self.get_logger().info(str(boxes))

            self.get_logger().info("Confidence Scores:")
            self.get_logger().info(str(scores))

            self.get_logger().info("Labels (Class IDs):")
            self.get_logger().info(str(labels))

        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")

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

def main(args=None):
    print("in main")
    rclpy.init(args=args)
    node = LiDARDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
