#!/usr/bin/env python3
import sys
import rclpy
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

#https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example/blob/master/px4_offboard/px4_offboard/control.py


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
        self.speed = 0.5
        self.turn = .2
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0.0
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        self.yaw_val = 0.0

        self.moveBindings = {
                            'w': (0, 0, -1, 0), #Z+
                            's': (0, 0, 1, 0),#Z-
                            'a': (0, 0, 0, -1), #Yaw+
                            'd': (0, 0, 0, 1),#Yaw-
                            '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
                            '\x1b[B' : (0, -1, 0, 0), #Down Arrow
                            '\x1b[C' : (-1, 0, 0, 0), #Right Arrow
                            '\x1b[D' : (1, 0, 0, 0),  #Left Arrow
                        }

        self.speedBindings = {
                            'q': (1.1, 1.1),
                            'z': (.9, .9),
                            'w': (1.1, 1),
                            'x': (.9, 1),
                            'e': (1, 1.1),
                            'c': (1, .9),
                            }
        
        self.msg = """
                    This node takes keypresses from the keyboard and publishes them
                    as Twist messages. 
                    Using the arrow keys and WASD you have Mode 2 RC controls.
                    W: Up
                    S: Down
                    A: Yaw Left
                    D: Yaw Right
                    Up Arrow: Pitch Forward
                    Down Arrow: Pitch Backward
                    Left Arrow: Roll Left
                    Right Arrow: Roll Right

                    Press SPACE to arm/disarm the drone
                    """
        
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

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.settings = self.saveTerminalSettings()

        # Create a timer to publish control commands
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer = self.create_timer(0.1, self.teleop)


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

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

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z, yaw]}")

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

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)

        elif self.vehicle_local_position.z <= self.takeoff_height:
            self.land()
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.get_logger().info("Updating")
            self.offboard_setpoint_counter += 1
    
    def getKey(self, settings):
        
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars  # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)
    
    def restoreTerminalSettings(old_settings):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def teleop(self):
        
        try:
            self.publish_offboard_control_heartbeat_signal()
            
            if self.offboard_setpoint_counter == 10:
                self.get_logger().info(self.msg)
                while True:
                    key = self.getKey(self.settings)
                    if key in self.moveBindings.keys():
                        x = self.moveBindings[key][0]
                        y = self.moveBindings[key][1]
                        z = self.moveBindings[key][2]
                        th = self.moveBindings[key][3]
                    
                    else:
                        x = 0.0
                        y = 0.0
                        z = 0.0
                        th = 0.0
                        if (key == '\x03'):
                            self.land()
                            exit(0)
                            break

                    if key == ' ':  # ASCII value for space
                        self.engage_offboard_mode()
                        self.arm()

                    self.x_val = (x * self.speed) + self.x_val
                    self.y_val = (y * self.speed) + self.y_val
                    self.z_val = (z * self.speed) + self.z_val
                    self.yaw_val = (th * self.turn) + self.yaw_val
                    
                    if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                        self.publish_position_setpoint(self.x_val, self.y_val, self.z_val, self.yaw_val)
            
            if self.offboard_setpoint_counter < 11:
                self.get_logger().info("Updating")
                self.offboard_setpoint_counter += 1    

        except Exception as e:
            self.get_logger().info(f"{e}")

        # finally:
        #     # self.publish_position_setpoint(self.x_val, self.y_val, self.z_val, self.yaw_val)
        #     self.land()
        #     exit(0)
        #     restoreTerminalSettings(self.settings)


def main(args=None) -> None:
    print('Starting teleop node...')
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