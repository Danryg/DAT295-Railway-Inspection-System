import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SmoothVelocityPublisher(Node):
    def __init__(self):
        super().__init__('smooth_velocity_publisher')
        self.publisher_ = self.create_publisher(Float32, '/amp_robot/speed', 10)
        #self.input = velocity
        self.timer = self.create_timer(1, self.publish_velocity)


    def publish_velocity(self):
        inputVelocity = Float32()
        #inputVelocity = 
        inputVelocity.data = float(input("Enter the desired velocity: "))
        self.publisher_.publish(inputVelocity)

def main(args=None):
    
    rclpy.init(args=args)
    publisher = SmoothVelocityPublisher()

    rclpy.spin(publisher)

    # shutdown ROS 2
    publisher.get_logger().info('Shutting down input publisher...')
    rclpy.shutdown()

if __name__ == '__main__':
    main()

