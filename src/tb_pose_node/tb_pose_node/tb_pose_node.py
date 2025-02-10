import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

class WafflePosePublisher(Node):
    def __init__(self):
        super().__init__('waffle_pose_publisher')
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states_demo',
            self.model_states_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/waffle/pose', 10)
        self.get_logger().info("Waffle Pose Publisher Node Started")

    def model_states_callback(self, msg: ModelStates):
        try:
            index = msg.name.index("waffle")  # Find index of "waffle" in model names
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"  # Change frame_id as needed
            pose_stamped.pose = msg.pose[index]
            
            self.publisher.publish(pose_stamped)
            self.get_logger().info("Published pose of 'waffle'")
        except ValueError:
            self.get_logger().warn("Model 'waffle' not found in /gazebo/model_states_demo")


def main(args=None):
    rclpy.init(args=args)
    node = WafflePosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()