import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_path
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class CameraService(Node):
    def __init__(self):
        super().__init__("camera_service_node")
        self.declare_parameter("save_path", os.getcwd() + "/temp")
        self.save_path = (
            self.get_parameter("save_path").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.camera_callback, 10
        )
        self.service = self.create_service(
            Trigger, "take_picture", self.handle_take_picture
        )
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_index = 0
        self.get_logger().info("Camera service node started.")

    def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def handle_take_picture(self, request, response):
        if self.latest_image is not None:
            # Save the latest image

            pic_name = "pic-" + str(self.image_index) + ".jpg"
            self.image_index = self.image_index + 1
            file_name = os.path.join(self.save_path, pic_name)
            cv2.imwrite(file_name, self.latest_image)
            self.get_logger().info(f"Picture saved to {file_name}")
            response.success = True
            response.message = f"Picture saved to {file_name}"
        else:
            response.success = False
            response.message = "No image received from the camera topic yet."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
