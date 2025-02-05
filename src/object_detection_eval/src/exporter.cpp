#include "object_detection_eval/exporter.hpp"
#include "object_detection_eval/filtered_model_position.hpp"
#include "object_detection_eval/models_info.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

Exporter::Exporter(std::shared_ptr<FilteredModelPosition> filtered_model_position, const std::string &output_file, double timeout)
    : Node("exporter_node"), filtered_model_position_(filtered_model_position), timeout_(timeout) {
    detection_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        "/detected_objects", 10,
        std::bind(&Exporter::detectionCallback, this, std::placeholders::_1));

    output_file_.open(output_file, std::ios::out | std::ios::app);
    if (!output_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Logging results to: %s", output_file.c_str());
    }
    start_time_ = this->now();
}

Exporter::~Exporter() {
    if (output_file_.is_open()) {
        output_file_.close();
    }
}

void Exporter::detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
    double elapsed_time = (this->now() - start_time_).seconds();
    if (elapsed_time >= timeout_) {
        RCLCPP_INFO(this->get_logger(), "Timeout reached, stopping logging.");
        rclcpp::shutdown();
        return;
    }
    
    auto filtered_models = filtered_model_position_->getFilteredModelPositions({});
    writeToFile(msg, filtered_models, elapsed_time);
}

void Exporter::writeToFile(const vision_msgs::msg::Detection3DArray::SharedPtr &detections, 
                           std::vector<model_info_t> &filtered_models, double timestamp) {
    if (!output_file_.is_open()) {
        return;
    }

    model_info_t refrence_pose;
    if (filtered_models.size() > 0) {
        refrence_pose = filtered_models.at(0);
        filtered_models.erase(filtered_models.begin());
    } else {
        return;
    }
    
    output_file_ << "Timestamp: " << timestamp << "\n";
    output_file_ << "Detected Objects:\n";
    for (const auto &detection : detections->detections) {
        output_file_ << "  Center: (" << detection.bbox.center.position.x + refrence_pose.model_pose.position.x << ", "
                     << detection.bbox.center.position.y + refrence_pose.model_pose.position.y << ", "
                     << detection.bbox.center.position.z + refrence_pose.model_pose.position.z << ")\n";
        output_file_ << "  Size: (" << detection.bbox.size.x << ", "
                     << detection.bbox.size.y << ", "
                     << detection.bbox.size.z << ")\n";
    }
    
    output_file_ << "Filtered Models (Ground Truth):\n";
    for (const auto &model : filtered_models) {
        output_file_ << "  Model: " << model.model_name << ", Distance: " << model.distance_from_base
                     << ", Width: " << model.width << ", Height: " << model.height
                     << ", Depth: " << model.depth << "\n";
        output_file_ << "  Center: (" << model.model_pose.position.x << ", "
                     << model.model_pose.position.y << ", "
                     << model.model_pose.position.z << ")\n";
    }
    output_file_ << "====================================\n";
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto model_position = std::make_shared<ModelPosition>();
    auto filtered_model_position = std::make_shared<FilteredModelPosition>(model_position, "waffle");
    auto exporter = std::make_shared<Exporter>(filtered_model_position, "output_data.txt", 60.0);
    
    rclcpp::spin(exporter);
    rclcpp::shutdown();
    return 0;
}
