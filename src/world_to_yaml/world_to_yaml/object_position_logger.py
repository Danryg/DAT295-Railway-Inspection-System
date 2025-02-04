import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node

default_location_file = os.path.join(
    get_package_share_directory("tb3_autonomy"), "maps", "sim_house_locations.yaml"
)

class ObjectPositionLogger(Node):
    def __init__(self):
        super().__init__("object_position_logger")
        self.subscription = self.create_subscription(
            ModelStates,
            "/gazebo/model_states_demo",  # Adjust if using a different topic
            self.listener_callback,
            10,
        )
        self.positions = {}

    def listener_callback(self, msg):
        print("Received message")
        self.positions = {}
        for i, name in enumerate(msg.name):
            position = msg.pose[i].position
            self.positions[f"location{i + 1}"] = [position.x, position.y, position.z]
        self.write_to_yaml()

    def write_to_yaml(self):
        print("Writing to YAML file to: ", default_location_file)
        yaml_file = os.path.expanduser(default_location_file)
        with open(yaml_file, "w") as file:
            yaml.dump(self.positions, file, default_flow_style=True)
        self.get_logger().info(f"Updated YAML file: {yaml_file}")


def main(args=None):
    print("Starting node...")
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
