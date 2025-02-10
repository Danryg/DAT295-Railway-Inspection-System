import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import Pose
import os
import subprocess
import sys
import math

class UndockWaffle(Node):
    def __init__(self, target_model='waffle'):
        super().__init__('undock_waffle')

        self.target_model = target_model
        self.tb3_odom_subscriber = self.create_subscription(Odometry, '/odom', self.tb3_odom_callback, 10)
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info(f'Waiting for /delete_entity service to delete {self.target_model}...')
        while not self.delete_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().info('Still waiting for /delete_entity service...')

        self.tb3_pose = None
        self.model_moved = False  # To ensure we move the model only once


    def tb3_odom_callback(self, msg):
        self.tb3_pose = msg.pose.pose
        self.check_and_move()



    def check_and_move(self):
        if self.tb3_pose and not self.model_moved:

            self.move_turtlebot3()
            self.model_moved = True

    def move_turtlebot3(self):

        # Delete the specified TurtleBot3 model
        delete_req = DeleteEntity.Request()
        delete_req.name = self.target_model

        self.get_logger().info(f'Attempting to delete {self.target_model}...')
        future = self.delete_client.call_async(delete_req)


        # Use the launch file to spawn TurtleBot3
        launch_file_dir = os.path.join(os.getcwd(), 'src', 'test_world_gazebo', 'launch')
        launch_file = os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')

        tb_x_pose = str(self.tb3_pose.position.x+2.0)
        tb_y_pose = str(self.tb3_pose.position.y)
        tb_z_pose = str(self.tb3_pose.position.z)

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

def main(args=None):
    rclpy.init(args=args)

    target_model = sys.argv[1] if len(sys.argv) > 1 else 'waffle'
    undocker = UndockWaffle(target_model=target_model)

    while rclpy.ok() and not undocker.model_moved:
        rclpy.spin_once(undocker)

    undocker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
