import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
import os
import yaml


class PointCloudFilter:
    def __init__(self):
        self.node = rclpy.create_node('pointcloud_filter')
        self.subscriber = self.node.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.callback,
            1
        )
        self.publisher = self.node.create_publisher(PointCloud2, '/lidar/filtered', 10)
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
        # Convert PointCloud2 data to numpy array
        point_cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        height, width = msg.height, msg.width
        point_cloud_data = point_cloud_data.reshape(height, width, -1)

        # Calculate rows to keep for each zone
        selected_rows = []
        for zone in self.vertical_zones:
            start_row = int(zone['start'] * height)
            end_row = int(zone['end'] * height)
            end_row = min(end_row, height)  # Ensure within bounds
            step = zone['downsample']
            if step < 1:
                step = 1  # Prevent invalid step
            # Generate row indices for this zone
            rows = np.arange(start_row, end_row, step)
            selected_rows.append(rows)

        # Combine and sort the selected rows
        selected_rows = np.unique(np.concatenate(selected_rows))

        # Filter the data
        filtered_data = point_cloud_data[selected_rows, :, :]
        new_height = filtered_data.shape[0]

        # Create the filtered PointCloud2 message
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = new_height
        filtered_msg.width = msg.width
        filtered_msg.fields = msg.fields.copy() if hasattr(msg, 'fields') else []
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = msg.row_step * (msg.height // new_height) if new_height != 0 else 0
        filtered_msg.is_dense = msg.is_dense
        filtered_msg.data = filtered_data.reshape(-1).tobytes()

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    pointcloud_filter = PointCloudFilter()
    try:
        rclpy.spin(pointcloud_filter.node)
    except KeyboardInterrupt:
        pass
    finally:
        pointcloud_filter.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()