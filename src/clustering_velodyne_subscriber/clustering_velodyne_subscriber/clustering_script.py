#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class ClusteringVelodyneSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("clustering_velo_subscriber")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_local_position_subscriber = self.create_subscription(
            PointCloud2,
            "velodyne_points",
            self.handle_point_data,
            qos_profile,
        )

    def handle_point_data(self, msg):
        self.get_logger().info(f"{msg.data}")


def main(args=None) -> None:
    print("Hej hej")
    rclpy.init(args=args)
    test_sub = ClusteringVelodyneSubscriber()
    rclpy.spin(test_sub)
    test_sub.destroy_node()
    rclpy.shutdown()
