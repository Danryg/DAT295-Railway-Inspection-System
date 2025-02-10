#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

#Resolve with approving PR
# from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseArray
from docking.controller import PID

class OffboardControl(Node):
    """Node for docking a drone in offboard mode."""

    def __init__(self) -> None:
        super().__init__('drone_docking')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.aruco_pose_subscriber = self.create_subscription(
                PoseArray, '/aruco_poses', self.aruco_pose_callback, 10)
        # self.aruco_marker_subscriber = self.create_subscription(
        #         ArucoMarkers, '/aruco_markers', self.aruco_marker_callback, qos_profile)
        
        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.aruco_poses = PoseArray
        self.offboard_setpoint_counter = 0

        #This sort of scales the controllers outputs
        self.OFFSET_X = 0.0
        self.OFFSET_Y = 0.0
        self.RAND_OFFSET = -2

        # Create a timer to publish control commands
        self.timer = self.create_timer(1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z] # This is wrong on so many levels (how this miss?)
        msg.yaw = 0.0 #1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def aruco_pose_callback(self, msg) -> None:
        """ Callback function for aruco pose"""  
        self.aruco_poses = msg
        
    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        ##Subscribe to ArUco /aruco_poses
        # self.aruco_pose_subscriber = self.create_subscription(PoseArray, '/aruco_poses', self.aruco_pose_callback, 10)        

        ## PID Loop and land
        '''
            Get vehicle pose 'self.vehicle_local_position' or 'self.vehicle_global_position'
            compare aruco pose and do pid
            correct position and land
        '''
        controller_x = PID()
        controller_y = PID()

        if self.offboard_setpoint_counter == 10:
            # self.publish_position_setpoint(-(round(self.controller_y.output*1000, 3) - self.RAND_OFFSET), -(round(self.controller_x.output*1000, 3) - self.RAND_OFFSET), 0.160)
            self.engage_offboard_mode()
            # time.sleep(5)
            exit(0)

        controller_x.update(self.aruco_poses.poses[-1].position.x + self.OFFSET_X)
        controller_y.update(self.aruco_poses.poses[-1].position.y + self.OFFSET_Y) 

        #x and y interchanged in drone convention
        #-2 offest in both x and y direction ( i donno why, i dont want to) doesnt affect this for now
        # self.publish_position_setpoint(-(round(self.controller_y.output*100, 3) - RAND_OFFSET), round(self.controller_x.output*1000, 3) + RAND_OFFSET, 0.16)
        self.publish_position_setpoint(-(round(controller_y.output*1000, 3) - self.RAND_OFFSET), -(round(controller_x.output*1000, 3) - self.RAND_OFFSET), 0.160)
        
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1



def main(args=None) -> None:
    print('Starting drone docking node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
