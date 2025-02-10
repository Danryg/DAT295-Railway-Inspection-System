import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import csv
import os
import time

class PoseToCSVNode(Node):
    def __init__(self):
        super().__init__('pose_to_csv_node')
        self.start_time = time.time()
        # Subscribers
        self.gt_pose_sub = self.create_subscription(
            PoseStamped, '/waffle/pose', self.gt_pose_callback, 10)
        self.est_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.est_pose_callback, 10)

        # CSV file paths
        self.gt_csv_file = 'ground_truth_pose.csv'
        self.est_csv_file = 'estimated_pose.csv'

        # Initialize CSV files
        self.init_csv(self.gt_csv_file, ['timestamp', 'x', 'y', 'qx', 'qy', 'qz', 'qw'])
        self.init_csv(self.est_csv_file, ['timestamp', 'x', 'y', 'qx', 'qy', 'qz', 'qw'])

    def init_csv(self, file_path, header):
        """
        Initialize a CSV file with the given header.
        """
        if not os.path.exists(file_path):
            with open(file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(header)

    def gt_pose_callback(self, msg):
        """
        Callback for /waffle/pose (ground truth).
        """
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.save_pose_to_csv_pose(self.gt_csv_file, timestamp, msg)

    def est_pose_callback(self, msg):
        """
        Callback for /pose (estimated pose).
        """
        relative_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        unix_timestamp = self.start_time + relative_timestamp
        self.save_pose_to_csv_pose_stamped(self.est_csv_file, unix_timestamp, msg)

    def save_pose_to_csv_pose(self, file_path, timestamp, msg):
        """
        Save pose data to a CSV file.
        """
        x = msg.pose.position.x
        y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, x, y, qx, qy, qz, qw])

    def save_pose_to_csv_pose_stamped(self, file_path, timestamp, msg):
        """
        Save pose data to a CSV file.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, x, y, qx, qy, qz, qw])

def main(args=None):
    rclpy.init(args=args)
    node = PoseToCSVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()