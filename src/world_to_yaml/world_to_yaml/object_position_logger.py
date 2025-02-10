import math
import os

import rclpy
import yaml
import re
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Define the path to the locations folder inside the package
package_dir = get_package_share_directory("tb3_autonomy")
locations_dir = os.path.join(package_dir, "locations")
os.makedirs(locations_dir, exist_ok=True)  # Ensure the directory exists

default_location_file = os.path.join(locations_dir, "locations.yaml")


class ObjectPositionLogger(Node):

    def __init__(self):
        super().__init__("object_position_logger")

        self.subscription = self.create_subscription(
            ModelStates,
            "/gazebo/model_states_demo",
            self.gazebo_model_pose_callback,
            10,
        )

        self.positions = {}
        self.subscription_2 = self.create_subscription(
            Odometry,
            "/odom",
            self.tb_pose_callback,
            10,
        )

        self.tb_position = [0, 0, 0]
        self.radius = 20 

    def tb_pose_callback(self, msg):
        self.get_logger().info("Received odometry message")
        position = msg.pose.pose.position
        self.tb_position = [position.x, position.y, position.z]


    def gazebo_model_pose_callback(self, msg):
        self.get_logger().info("Received model odom message")
        self.obj_positions = {}
        self.get_tb_pose(msg)

        for i, name in enumerate(msg.name):
            distance = self.calculate_distance(msg.pose[i].position, self.tb_position)
            if distance < self.radius:
                self.add_obj("lamp", name, msg.pose[i].position, i)

        self.get_logger().info(f"Positions: {self.positions}")

    def get_tb_pose(self,msg):
        index = msg.name.index("waffle")
        position = msg.pose[index]

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose = position

        
    
    def add_obj(self, obj_name, cur_name, obj_position, index):
        if re.search(rf"{obj_name}", cur_name):
            self.positions[f"location{index}"] = [
                obj_position.x,
                obj_position.y,
                obj_position.z,
            ]

    def calculate_distance(self, position, tb_position):
        """Calculates Euclidean distance between two points"""
        distance = math.sqrt((position.x-tb_position[0])**2 + (position.y-tb_position[1])**2)

        return distance

    def write_to_yaml(self):
        with open(default_location_file, "w") as file:
            yaml.dump(self.positions, file, default_flow_style=True)
        self.get_logger().info(f"Updated YAML file: {default_location_file}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
