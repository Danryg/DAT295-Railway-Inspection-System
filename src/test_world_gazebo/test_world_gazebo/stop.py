import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SmoothVelocityPublisher(Node):
    def __init__(self):
        super().__init__('smooth_velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/tb3/cmd_vel', 10)
        self.target_velocity = 0.0 
        self.acceleration = 0.5
        self.current_velocity = 0.0
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        # Gradually increase the velocity
        if self.current_velocity < self.target_velocity:
            self.current_velocity += self.acceleration * 0.1
            if self.current_velocity > self.target_velocity:
                self.current_velocity = self.target_velocity
        
        twist = Twist()
        twist.linear.x = self.current_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    
    rclpy.init(args=args)
    node = SmoothVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
