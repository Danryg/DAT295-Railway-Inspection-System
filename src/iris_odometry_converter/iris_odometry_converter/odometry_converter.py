import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry

class OdometryConverter(Node):

    def __init__(self):
        super().__init__('odometry_converter')
        self.subscription = self.create_subscription(
            VehicleOdometry,
            'vehicle_odometry',
            self.odometry_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'converted_odometry', 10)

    def odometry_callback(self, msg):
        odometry_msg = Odometry()
        
        # Fill in the header information
        odometry_msg.header.stamp = msg.timestamp
        odometry_msg.header.frame_id = 'odom'

        # Convert position
        odometry_msg.pose.pose.position.x = msg.position[0]
        odometry_msg.pose.pose.position.y = msg.position[1]
        odometry_msg.pose.pose.position.z = msg.position[2]

        # Convert orientation (assuming msg.q is a quaternion [w, x, y, z])
        odometry_msg.pose.pose.orientation.w = msg.q[0]
        odometry_msg.pose.pose.orientation.x = msg.q[1]
        odometry_msg.pose.pose.orientation.y = msg.q[2]
        odometry_msg.pose.pose.orientation.z = msg.q[3]

        # Convert linear velocities
        odometry_msg.twist.twist.linear.x = msg.velocity[0]
        odometry_msg.twist.twist.linear.y = msg.velocity[1]
        odometry_msg.twist.twist.linear.z = msg.velocity[2]

        # Convert angular velocities
        odometry_msg.twist.twist.angular.x = msg.angular_velocity[0]
        odometry_msg.twist.twist.angular.y = msg.angular_velocity[1]
        odometry_msg.twist.twist.angular.z = msg.angular_velocity[2]

        self.publisher.publish(odometry_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
