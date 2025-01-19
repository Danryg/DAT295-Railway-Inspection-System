import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
class OdometryConverter(Node):

    def __init__(self):
        super().__init__('odometry_converter')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )


        self.subscription = self.create_subscription(
            VehicleOdometry,
            'fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
            )
        self.publisher = self.create_publisher(Odometry, 'odom', 10)

    def odometry_callback(self, msg):
        odometry_msg = Odometry()
        
        # Fill in the header information
        # Convert the timestamp
        time_msg = Time()
        time_msg.sec = int(msg.timestamp // 1_000_000_000)
        time_msg.nanosec = int(msg.timestamp % 1_000_000_000)
        odometry_msg.header.stamp = time_msg

        odometry_msg.header.frame_id = 'odom'

        # Convert position
        odometry_msg.pose.pose.position.x = float(msg.position[0])
        odometry_msg.pose.pose.position.y = float(msg.position[1])
        odometry_msg.pose.pose.position.z = float(msg.position[2])

        # Convert orientation (assuming msg.q is a quaternion [w, x, y, z])
        odometry_msg.pose.pose.orientation.w = float(msg.q[0])
        odometry_msg.pose.pose.orientation.x = float(msg.q[1])
        odometry_msg.pose.pose.orientation.y = float(msg.q[2])
        odometry_msg.pose.pose.orientation.z = float(msg.q[3])

        # Convert linear velocities
        odometry_msg.twist.twist.linear.x = float(msg.velocity[0])
        odometry_msg.twist.twist.linear.y = float(msg.velocity[1])
        odometry_msg.twist.twist.linear.z = float(msg.velocity[2])

        # Convert angular velocities
        odometry_msg.twist.twist.angular.x = float(msg.angular_velocity[0])
        odometry_msg.twist.twist.angular.y = float(msg.angular_velocity[1])
        odometry_msg.twist.twist.angular.z = float(msg.angular_velocity[2])

        self.publisher.publish(odometry_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
