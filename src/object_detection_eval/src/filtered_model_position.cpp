#include "object_detection_eval/filtered_model_position.hpp"
#include "object_detection_eval/model_position.hpp"
#include <cmath>

FilteredModelPosition::FilteredModelPosition(std::shared_ptr<ModelPosition> model_position, const std::string& reference_model)
    : model_position_(model_position), reference_model_(reference_model) {}

std::vector<model_info_t> FilteredModelPosition::getFilteredModelPositions(const std::vector<std::string>& exclude_models) {
    auto models = model_position_->getAllModelPositions();
    std::vector<model_info_t> filtered_models;

    geometry_msgs::msg::Pose reference_pose;
    for (const auto& model : models) {
        if (model.model_name == reference_model_) {
            reference_pose = model.model_pose;
            break;
        }
    }

    for (const auto& model : models) {
        if (std::find(exclude_models.begin(), exclude_models.end(), model.model_name) != exclude_models.end()) {
            continue;
        }
        double dx = model.model_pose.position.x - reference_pose.position.x;
        double dy = model.model_pose.position.y - reference_pose.position.y;
        double dz = model.model_pose.position.z - reference_pose.position.z;
        model_info_t updated_model = model;
        updated_model.distance_from_base = std::sqrt(dx * dx + dy * dy + dz * dz);
        filtered_models.push_back(updated_model);
    }

    return filtered_models;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto model_position = std::make_shared<ModelPosition>();
    FilteredModelPosition filtered_model_position(model_position, "waffle");
    
    std::vector<std::string> exclude_models = {"ground_plane"};
    
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()) {
        auto filtered_models = filtered_model_position.getFilteredModelPositions(exclude_models);
        for (const auto& model : filtered_models) {
            RCLCPP_INFO(rclcpp::get_logger("FilteredModel"), "Model: %s, Distance: %.2f", model.model_name.c_str(), model.distance_from_base);
        }
        rclcpp::spin_some(model_position);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
