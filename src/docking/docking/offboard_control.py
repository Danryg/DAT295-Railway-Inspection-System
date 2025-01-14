#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

#Resolve with approving PR
#from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseArray
from controller import PID

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
                PoseArray, '/aruco_poses', self.aruco_pose_callback, qos_profile)
        # self.aruco_marker_subscriber = self.create_subscription(
        #         ArucoMarkers, '/aruco_markers', self.aruco_marker_callback, qos_profile)
        
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        # self.aruco_pose = ArucoMarkers
        self.takeoff_height = -5.0 #not need this

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

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
        msg.yaw = 0 #1.57079  # (90 degree)
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
        self.aruco_pose = msg #msg.poses.position.x, .y, .z

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        ##Subscribe to ArUco /aruco_poses
        self.get_logger().info(f"Aruco pose {self.aruco_pose}")

        ## PID Loop and land
        '''
            Get vehicle pose 'self.vehicle_local_position' or 'self.vehicle_global_position'
            compare aruco pose and do pid
            correct position and land
        '''
        controller_x = PID()
        controller_y = PID()

        z_val = -2

        time_when_state_last_steady = 0

        OFFSET_X = 50
        OFFSET_Y = 3

        ERROR_MARGIN = 50
        while True:
            estimate = self._estimateQueue.get() #Change to aruco_pose and modify accordingly

            if (
                abs(estimate[1][0] - OFFSET_X) < ERROR_MARGIN #Change to aruco_pose and modify accordingly
                and abs(estimate[1][1] - OFFSET_Y) < ERROR_MARGIN
            ):
                time_when_state_last_steady = time.time()

            controller_x.update(estimate[1][0] + OFFSET_X) #Change to aruco_pose and modify accordingly
            controller_y.update(estimate[1][1] + OFFSET_Y) #Change to aruco_pose and modify accordingly

            print("x:", estimate[1][0], " ,y:", estimate[1][1], ",z:", z_val)

            if time.time() - time_when_state_last_steady < 1:
                z_val += 0.004

            # await self._drone.offboard.set_position_ned(
            #     PositionNedYaw(controller_y.output, -controller_x.output, z_val, 1.57)
            # )

            self.publish_position_setpoint(controller_x.output, controller_y.output, z_val)

        # if self.offboard_setpoint_counter == 10:
        #     self.engage_offboard_mode()
        #     self.arm()

        # if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        # elif self.vehicle_local_position.z <= self.takeoff_height:
        #     self.land()
        #     exit(0)

        # if self.offboard_setpoint_counter < 11:
        #     self.offboard_setpoint_counter += 1


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
