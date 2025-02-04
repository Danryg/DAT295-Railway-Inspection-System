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
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

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
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.aruco_poses = PoseArray
        self.takeoff_height = -5.0 #not need this

        # Create a timer to publish control commands
        self.timer = self.create_timer(1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

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
        msg.position = [x, y, z]
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
        self.aruco_pose_subscriber = self.create_subscription(PoseArray, '/aruco_poses', self.aruco_pose_callback, 10)        

        ## PID Loop and land
        '''
            Get vehicle pose 'self.vehicle_local_position' or 'self.vehicle_global_position'
            compare aruco pose and do pid
            correct position and land
        '''
        controller_x = PID()
        controller_y = PID()
        controller_z = PID()
        
        z_val = -2.5

        time_when_state_last_steady = 0

        #fix this
        OFFSET_X = 50
        OFFSET_Y = 3
        OFFSET_Z = 2

        ERROR_MARGIN = 50 #fix
            
        if (
            abs(self.aruco_poses.poses[-1].position.x - self.vehicle_local_position.x) < ERROR_MARGIN #Change to aruco_pose and modify accordingly
            and abs(self.aruco_poses.poses[-1].position.y - self.vehicle_local_position.y) < ERROR_MARGIN
        ):
            time_when_state_last_steady = time.time()

        controller_x.update(self.aruco_poses.poses[-1].position.x + OFFSET_X) #fix the offsets
        controller_y.update(self.aruco_poses.poses[-1].position.y + OFFSET_Y) #fix the offsets
        controller_z.update(self.aruco_poses.poses[-1].position.z + OFFSET_Z) #fix the offsets


        if time.time() - time_when_state_last_steady < 1:
            z_val += 0.004 #might have to change sign to account for descent/land

        self.publish_position_setpoint(round(controller_x.output, 3), round(controller_y.output, 3), round(controller_z.output, 3))

        if abs(round(controller_z.output, 3)) <= 0.06:
            self.land()
            exit(0)


def main(args=None) -> None:
    print('Starting offboard control node...')
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
