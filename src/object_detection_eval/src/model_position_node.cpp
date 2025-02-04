#include "object_detection_eval/model_position.hpp"
#include "object_detection_eval/models_info.hpp"
#include <chrono>
#include <vector>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelPosition>();

    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()) {
        auto models_info = node->getAllModelPositions();
         for (const auto& model : models_info) {
            RCLCPP_INFO(node->get_logger(), "Model: %s, Position -> x: %.2f, y: %.2f, z: %.2f, Width: %.2f, Depth: %.2f, Height: %.2f",
                        model.model_name.c_str(), model.model_pose.position.x, model.model_pose.position.y, model.model_pose.position.z,
                        model.width, model.depth, model.height);
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


