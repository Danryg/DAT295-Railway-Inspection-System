import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SmoothVelocityPublisher(Node):
    def __init__(self):
        super().__init__('smooth_velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/tb3/cmd_vel', 10)
        self.target_velocity = 10.0
        self.acceleration = 1.0
        self.current_velocity = 0.0
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.reached_target = False


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
        
        if self.current_velocity >= self.target_velocity and not self.reached_target:
            self.get_logger().info(f'Reached target velocity {self.target_velocity}')
            self.reached_target = True
            # Flag set to true to stop the spin loop.

def main(args=None):
    velocity = float(input("Enter the desired velocity: "))
    rclpy.init(args=args)
    node = SmoothVelocityPublisher()
    node.target_velocity = velocity

    # Keep spinning until target velocity is reached
    while rclpy.ok() and not node.reached_target:
        rclpy.spin_once(node)
    
    # After the target velocity is reached, shutdown ROS 2
    node.get_logger().info('Shutting down node...')
    rclpy.shutdown()

if __name__ == '__main__':
    main()

