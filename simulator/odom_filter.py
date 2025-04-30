import numpy as np
import rclpy
import yaml
import time
import os
from nav_msgs.msg import Odometry


class OdomFilter:
    def __init__(self):
        self.node = rclpy.create_node('odom_filter')
        self.subscriber = self.node.create_subscription(
            Odometry,
            '/model/vehicle_blue/odometry',
            self.callback,
            1
        )
        self.publisher = self.node.create_publisher(Odometry, '/model/vehicle_blue/odometry_filtered', 30)
        

    def callback(self, msg):
        start_time = time.time()

        if msg.child_frame_id == "vehicle_blue/base_footprint":
            msg.header.frame_id = "vehicle_blue/chassis/lidar"
            publish_duration = time.time() - start_time
            # create a new message for yaw
            msg.pose.pose.position.z = 0.0
            # Reverse the yaw in the quaternion
            self.publisher.publish(msg)
            # Log the times for each part
            self.node.get_logger().info(f"Odom filter publishing duration: {publish_duration:.3}s")


def main(args=None):
    rclpy.init(args=args)
    odom_filter = OdomFilter()
    try:
        rclpy.spin(odom_filter.node)
    except KeyboardInterrupt:
        pass
    finally:
        odom_filter.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()