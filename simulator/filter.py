import numpy as np
import rclpy
import yaml
import time
import os
import struct
from sensor_msgs.msg import PointCloud2
from rclpy.executors import MultiThreadedExecutor


class PointCloudFilter:
    def __init__(self):
        self.node = rclpy.create_node('pointcloud_filter')
        self.subscriber = self.node.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.callback,
            1
        )
        self.publisher = self.node.create_publisher(PointCloud2, '/lidar/filtered', 30)
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        params_file = os.path.join(pkg_dir, '../params.yaml')
        if not os.path.exists(params_file):
            raise FileNotFoundError(f"Il file YAML dei parametri non esiste: {params_file}")
    
        with open(params_file, 'r') as yaml_file:
            params = yaml.safe_load(yaml_file)

        filter_params = params.get('filter', {})
        required_keys = ['vertical_zones']
        for key in required_keys:
            if key not in filter_params:
                raise ValueError(f"Parametro '{key}' mancante nel file YAML.")

        # Define vertical zones as list of dictionaries:
        # Each zone has 'start' (0.0-1.0), 'end' (0.0-1.0), and 'downsample' factor (int)
        self.vertical_zones = filter_params['vertical_zones']

    def callback(self, msg):
        start_time = time.time()

        # Step 1: Point cloud data conversion
        point_cloud_data_start = time.time()
        point_cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        height, width = msg.height, msg.width
        point_cloud_data = point_cloud_data.reshape(height, width, -1)
        point_cloud_data_duration = time.time() - point_cloud_data_start

        # Step 2: Vertical zones filtering
        vertical_filter_start = time.time()
        selected_rows = []
        for zone in self.vertical_zones:
            start_row = int(zone['start'] * height)
            end_row = int(zone['end'] * height)
            end_row = min(end_row, height)
            step = zone['downsample']
            if step < 1:
                step = 1
            rows = np.arange(start_row, end_row, step)
            selected_rows.append(rows)
        selected_rows = np.unique(np.concatenate(selected_rows))
        vertical_filter_duration = time.time() - vertical_filter_start

        # Step 3: Data filtering
        filter_data_start = time.time()
        filtered_data = point_cloud_data[selected_rows, :, :]
        new_height = filtered_data.shape[0]
        filter_data_duration = time.time() - filter_data_start

        # Step 4: Publishing
        publish_start = time.time()
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = new_height
        filtered_msg.width = msg.width
        filtered_msg.fields = msg.fields.copy() if hasattr(msg, 'fields') else []
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = msg.row_step * (msg.height // new_height) if new_height != 0 else 0
        filtered_msg.is_dense = msg.is_dense

        filtered_data = filtered_data.astype(np.float16)
        filtered_msg.data = filtered_data.tobytes()
        #filtered_msg.data = filtered_data.reshape(-1).tobytes()
        constuction_duration = time.time() - publish_start

        self.publisher.publish(filtered_msg)
        publish_duration = time.time() - publish_start


        total_duration = time.time() - start_time

        # Log the times for each part
        self.node.get_logger().info(f"Total callback duration: {total_duration:.4f}s")
        self.node.get_logger().info(f"PointCloud conversion: {point_cloud_data_duration:.4f}s")
        self.node.get_logger().info(f"Vertical filter duration: {vertical_filter_duration:.4f}s")
        self.node.get_logger().info(f"Filtering data duration: {filter_data_duration:.4f}s")
        self.node.get_logger().info(f"Constuction duration: {constuction_duration:.4f}s")
        self.node.get_logger().info(f"Publishing duration: {publish_duration:.4f}s")



def main(args=None):
    rclpy.init(args=args)
    pointcloud_filter = PointCloudFilter()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(pointcloud_filter.node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_filter.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()