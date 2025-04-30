import os
import yaml
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class OdomFilter:
    def __init__(self):
        self.node = rclpy.create_node('odom_filter')
        self.subscriber = self.node.create_subscription(
            Odometry,
            '/model/vehicle_blue/odometry',
            self.callback,
            1
        )
        
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        params_file = os.path.join(pkg_dir, '../params.yaml')
        if not os.path.exists(params_file):
            raise FileNotFoundError(f"Il file YAML dei parametri non esiste: {params_file}")
    
        with open(params_file, 'r') as yaml_file:
            params = yaml.safe_load(yaml_file)

        filter_params = params.get('filter', {})
        required_keys = ['odom_topic', 'fixed_frame']
        for key in required_keys:
            if key not in filter_params:
                raise ValueError(f"Parametro '{key}' mancante nel file YAML.")
        odom_topic = filter_params['odom_topic']
        self.fixed_frame = filter_params['fixed_frame']

        self.publisher = self.node.create_publisher(Odometry, odom_topic, 30)
        

    def callback(self, msg):
        start_time = time.time()

        if msg.child_frame_id == "vehicle_blue/base_footprint":
            msg.header.frame_id = self.fixed_frame  #"vehicle_blue/chassis/lidar"
            publish_duration = time.time() - start_time
            # create a new message for yaw
            msg.pose.pose.position.z = 0.0
            # Reverse the yaw in the quaternion
            self.publisher.publish(msg)
            # Log the times for each part
            self.node.get_logger().info(f"Odom filter publishing duration: {publish_duration:.3}s")

class PointCloudFilter:
    def __init__(self):
        self.node = rclpy.create_node('pointcloud_filter')
        self.subscriber = self.node.create_subscription(
            PointCloud2,
            '/model/points',
            self.callback,
            1
        )
        
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        params_file = os.path.join(pkg_dir, '../params.yaml')
        if not os.path.exists(params_file):
            raise FileNotFoundError(f"Il file YAML dei parametri non esiste: {params_file}")
    
        with open(params_file, 'r') as yaml_file:
            params = yaml.safe_load(yaml_file)

        filter_params = params.get('filter', {})
        required_keys = ['pcl_topic', 'fixed_frame']
        for key in required_keys:
            if key not in filter_params:
                raise ValueError(f"Parametro '{key}' mancante nel file YAML.")
        pcl_topic = filter_params['pcl_topic']
        self.fixed_frame = filter_params['fixed_frame']

        self.publisher = self.node.create_publisher(PointCloud2, pcl_topic, 1)
        

    def callback(self, msg):
        start_time = time.time()

        msg.header.frame_id = self.fixed_frame  #"vehicle_blue/chassis/lidar"
        publish_duration = time.time() - start_time
        self.publisher.publish(msg)

        self.node.get_logger().info(f"PCL filter publishing duration: {publish_duration:.3}s")


def main(args=None):
    rclpy.init(args=args)

    odom_filter = OdomFilter()
    pcl_filter = PointCloudFilter()

    executor = MultiThreadedExecutor()
    executor.add_node(odom_filter.node)
    executor.add_node(pcl_filter.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        odom_filter.node.destroy_node()
        pcl_filter.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()