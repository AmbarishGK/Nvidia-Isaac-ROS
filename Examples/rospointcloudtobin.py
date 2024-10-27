#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import os
from datetime import datetime
from sensor_msgs_py import point_cloud2
import time

class KittiConverter(Node):
    def __init__(self):
        super().__init__('kitti_converter')
        # Create output directory
        self.output_dir = 'kitti_format/velodyne'
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize storage for accumulated points
        self.all_points = []
        self.frame_count = 0
        self.max_frames = 500  # Adjust this number based on how many frames you want to save
        
        # Add timing control
        self.last_process_time = time.time()
        self.min_delay = 0.1  # Minimum delay between processing frames (in seconds)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/front_3d_lidar/point_cloud',
            self.pointcloud_callback,
            10)
        self.get_logger().info('Listening to /front_3d_lidar/point_cloud topic...')

    def pointcloud_callback(self, msg):
        try:
            current_time = time.time()
            # Check if enough time has passed since last processing
            if current_time - self.last_process_time < self.min_delay:
                return  # Skip this frame if not enough time has passed
            
            # Convert PointCloud2 to numpy array
            points_list = []
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                # Add intensity as 0 since it's not in the original data
                point = [p[0], p[1], p[2], 0.0]
                points_list.append(point)
            
            points = np.array(points_list, dtype=np.float32)
            self.all_points.append(points)
            self.frame_count += 1
            
            # Update last process time
            self.last_process_time = current_time
            
            self.get_logger().info(f"Processed frame {self.frame_count} at time {current_time:.2f}")
            
            # Save all points when we reach max_frames
            if self.frame_count >= self.max_frames:
                self.save_accumulated_points()
                self.destroy_node()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def save_accumulated_points(self):
        try:
            # Concatenate all points
            all_points_array = np.concatenate(self.all_points, axis=0)
            
            # Save to single bin file
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'all_points_{timestamp}.bin')
            all_points_array.tofile(filename)
            
            # Print information
            self.get_logger().info(f"Saved all points to {filename}")
            self.get_logger().info(f"Total points: {len(all_points_array)}")
            self.get_logger().info(f"Total frames: {self.frame_count}")
            self.get_logger().info(f"Average points per frame: {len(all_points_array)/self.frame_count:.2f}")
            print("\nFirst 5 points from final file (x, y, z, intensity):")
            print(all_points_array[:5])

        except Exception as e:
            self.get_logger().error(f'Error saving accumulated points: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    converter = KittiConverter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        converter.get_logger().info('Saving accumulated points before shutdown...')
        converter.save_accumulated_points()
    except Exception as e:
        converter.get_logger().error(f'Error: {str(e)}')
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
