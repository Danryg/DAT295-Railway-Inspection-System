#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np

class RailCrackDetectionNode(Node):
    def __init__(self):
        super().__init__('rail_crack_detection_laserscan_node')

        # Set a threshold (in meters) for determining cracks/deformations, which needs to be tuned
        self.crack_threshold = 0.1

        # Left and right railroad track coordinates (x,y) for first frame reference
        self.left_rail_baseline = None
        self.right_rail_baseline = None

        # Has the baseline been initialized?
        self.baseline_initialized = False

        # In order to align during dithering, we need to record the left and right rail positions 
        # from the previous frame
        self.prev_left_rail = None
        self.prev_right_rail = None

        # Create Subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/lidar_sensor_1',
            self.scan_callback,
            10  # QoS
        )

        self.get_logger().info("Rail crack detection node started.")

    def scan_callback(self, scan_msg: LaserScan):
        # Extracting laser scanning information
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        num_points = len(ranges)

        # 1) Filter out invalid or distant readings (if required)
        # Further filtering can be done by range_min, range_max depending on the specific hardware.
        valid_mask = np.isfinite(ranges)  # remove inf
        ranges = ranges[valid_mask]
        indices = np.nonzero(valid_mask)[0]  # Retain original index

        if len(ranges) < 2:
            self.get_logger().warning("Not enough valid points in this scan.")
            return

        # 2) Convert to xy plane coordinates
        angles = angle_min + indices * angle_increment
        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        # 3) Split left and right points
        # Assumptions: y>0 is considered left, y<0 is considered right
        # Modify as appropriate if actual LIDAR installation is different
        left_mask = (y_coords > 0)
        right_mask = (y_coords < 0)

        left_points = np.column_stack((x_coords[left_mask], y_coords[left_mask]))
        right_points = np.column_stack((x_coords[right_mask], y_coords[right_mask]))

        if len(left_points) == 0 or len(right_points) == 0:
            self.get_logger().warning("Cannot find left or right rail points in this frame.")
            return

        # 4) In the left/right respective point clouds, take “the point closest to the LIDAR” 
        # (a minimalist approach).
        # You can switch to more advanced algorithms.
        # - fit a line to the y>0 point cloud (e.g. with RANSAC).
        # - Find the line most likely to represent the railroad tracks.
        # - take the point where it's the smallest from the radar (0,0) or take the center.
        left_dists = np.linalg.norm(left_points, axis=1)  # 每个点到(0,0)的距离
        right_dists = np.linalg.norm(right_points, axis=1)

        left_min_idx = np.argmin(left_dists)
        right_min_idx = np.argmin(right_dists)

        left_rail_xy = left_points[left_min_idx]   # (xL, yL)
        right_rail_xy = right_points[right_min_idx]# (xR, yR)

        # 5) Calculate the “overall lateral offset” of this frame with respect to the previous one, 
        # and correct for it.
        # Here's an example: if both left and right rails existed in the previous frame, we can estimate 
        # the offset = the difference between the centers of the two frames (left + right)
        if self.prev_left_rail is not None and self.prev_right_rail is not None:
            prev_center = (self.prev_left_rail + self.prev_right_rail) / 2.0
            current_center = (left_rail_xy + right_rail_xy) / 2.0
            # This is the difference between the center of the railroad track in the last frame and 
            # this frame in the laser coordinate system
            shift_vector = current_center - prev_center

            # We can use shift_vector to “correct” the data for this frame (or update the baseline)
            # For example: left_rail_xy_corrected = left_rail_xy - shift_vector
            # right_rail_xy_corrected = right_rail_xy - shift_vector
            # Then compare to the baseline
            # This is just an example, the origin will be used in the later stages of detection.

        # Update prev_rail
        self.prev_left_rail = left_rail_xy
        self.prev_right_rail = right_rail_xy

        # 6) If the baseline is not initialized, use the rail position in this frame as the baseline.
        if not self.baseline_initialized:
            self.left_rail_baseline = left_rail_xy
            self.right_rail_baseline = right_rail_xy
            self.baseline_initialized = True
            self.get_logger().info("Baseline initialized with current rail positions.")
            return

        # 7) Crack/anomaly detection
        # Here is a minimalist approach: directly compare the difference between the left and right rail 
        # and the baseline (distance) If it is greater than a certain threshold, it is considered a possible 
        # crack or deformation.
        diff_left = np.linalg.norm(left_rail_xy - self.left_rail_baseline)
        diff_right = np.linalg.norm(right_rail_xy - self.right_rail_baseline)

        # You can refine: compare x and y components or compare more nearby points 
        # than just the minimum distance point. Here's a simple threshold judgment.
        if diff_left > self.crack_threshold or diff_right > self.crack_threshold:
            # If the difference between one of the sides exceeds the threshold, 
            # an alarm is raised
            self.get_logger().warning(
                f"Possible crack or deformation detected! (left diff={diff_left:.3f}, right diff={diff_right:.3f})"
            )
        else:
            self.get_logger().info("No significant crack detected in this scan.")

def main(args=None):
    rclpy.init(args=args)
    node = RailCrackDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()