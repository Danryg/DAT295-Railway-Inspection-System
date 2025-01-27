import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SpeedController(Node):
	def __init__(self):
		super().__init__('speed_controller')
		self.subscription = self.create_subscription(Float32, '/amp_robot/speed', self.change_velocity, 10)
		self.subscription  # prevent unused variable warning
		self.publisher_ = self.create_publisher(Twist, '/amp_robot/cmd_vel', 10)
		self.target_velocity = 0.0
		self.acceleration = 1.0
		self.current_velocity = 0.0
		self.timer = self.create_timer(0.1, self.publish_velocity)

	def change_velocity(self, target):
		self.target_velocity = target.data
		
	def publish_velocity(self):
        # Gradually increase the velocity
		if self.target_velocity == 0.0:
			self.current_velocity = 0.0
		elif self.current_velocity < self.target_velocity:
			self.current_velocity += self.acceleration * 0.1
			if self.current_velocity > self.target_velocity:
				self.current_velocity = self.target_velocity
		elif self.current_velocity > self.target_velocity:
			self.current_velocity -= self.acceleration * 0.1
			if self.current_velocity < self.target_velocity:
				self.current_velocity = self.target_velocity
		else:
			self.current_velocity = self.target_velocity
		
		twist = Twist()
		twist.linear.x = self.current_velocity
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
		self.publisher_.publish(twist)
		
		self.get_logger().info(f'Current speed is: {self.target_velocity} \t Target speed is: {self.target_velocity}')

def main(args=None):
	print("Started speed controller node")
	rclpy.init(args=args)
	node = SpeedController()
	rclpy.spin(node)

    # Maybe add a shutdown trigger when q is pressed or something
	node.get_logger().info('Shutting down node...')
	rclpy.shutdown()

if __name__ == '__main__':
	main()

