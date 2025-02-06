import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import Pose
import os
import subprocess
import sys
import math

class DockWaffle(Node):
    def __init__(self, target_model='waffle'):
        super().__init__('model_pose_copier')

        self.target_model = target_model
        self.odom_subscriber = self.create_subscription(Odometry, '/amp_robot/odom', self.odom_callback, 10)
        self.tb3_odom_subscriber = self.create_subscription(Odometry, '/odom', self.tb3_odom_callback, 10)
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info(f'Waiting for /delete_entity service to delete {self.target_model}...')
        while not self.delete_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().info('Still waiting for /delete_entity service...')

        self.amp_pose = None
        self.tb3_pose = None
        self.model_moved = False  # To ensure we move the model only once

    def odom_callback(self, msg):
        self.amp_pose = msg.pose.pose
        self.check_and_move()

    def tb3_odom_callback(self, msg):
        self.tb3_pose = msg.pose.pose
        self.check_and_move()

    def calculate_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 +
            (pose1.position.y - pose2.position.y) ** 2
        )

    def check_and_move(self):
        if self.amp_pose and self.tb3_pose and not self.model_moved:
            distance = self.calculate_distance(self.amp_pose, self.tb3_pose)
            self.get_logger().info(f'Distance between AMP robot and TurtleBot3: {distance:.2f} meters.')

            if distance <= 3.0:
                self.move_turtlebot3()
                self.model_moved = True
                self.shutdown_node()
            else:
                self.get_logger().info('TurtleBot3 is more than 3 meters away. No movement will be performed.')

    def move_turtlebot3(self):
        if self.amp_pose is None:
            self.get_logger().error('No odometry data received yet.')
            return

        # Delete the specified TurtleBot3 model
        delete_req = DeleteEntity.Request()
        delete_req.name = self.target_model

        self.get_logger().info(f'Attempting to delete {self.target_model}...')
        future = self.delete_client.call_async(delete_req)


        # Use the launch file to spawn TurtleBot3
        launch_file_dir = os.path.join(os.getcwd(), 'src', 'test_world_gazebo', 'launch')
        launch_file = os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')

        tb_x_pose = str(self.amp_pose.position.x)
        tb_y_pose = str(self.amp_pose.position.y-0.2)
        tb_z_pose = str(self.amp_pose.position.z+0.2)

        try:
            subprocess.run([
                'ros2', 'launch', launch_file,
                'use_sim_time:=true',
                f'x_pose:={tb_x_pose}',
                f'y_pose:={tb_y_pose}',
                f'z_pose:={tb_z_pose}'
            ], check=True)
            self.get_logger().info(f'Successfully spawned {self.target_model} at the new pose.')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to spawn {self.target_model}: {e}')

    def shutdown_node(self):
        self.get_logger().info('Shutting down node after completing the operation.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    target_model = sys.argv[1] if len(sys.argv) > 1 else 'waffle'
    copier = DockWaffle(target_model=target_model)

    while rclpy.ok() and not copier.model_moved:
        rclpy.spin_once(copier)

    copier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
