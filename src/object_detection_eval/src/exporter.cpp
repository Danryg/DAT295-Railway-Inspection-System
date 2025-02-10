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
#include <iostream>

Exporter::Exporter(std::shared_ptr<FilteredModelPosition> filtered_model_position, 
                   const std::string &output_file, double timeout, const std::string &detection_topic)
    : Node("exporter_node"), filtered_model_position_(filtered_model_position), timeout_(timeout),
      output_file_path_(output_file), detection_topic_(detection_topic), models_logged_(false), log_ready_(false), entered_ready_(false) {
    
    detection_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        detection_topic_, 10,
        std::bind(&Exporter::detectionCallback, this, std::placeholders::_1));
    
    output_file_.open(output_file_path_, std::ios::out | std::ios::app);
    if (!output_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_file_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Logging results to: %s", output_file_path_.c_str());
    }
    start_time_ = this->now();
}

Exporter::~Exporter() {
    if (output_file_.is_open()) {
        output_file_.close();
    }
}

void Exporter::detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
    if (!models_logged_) {
        auto filtered_models = filtered_model_position_->getFilteredModelPositions({});
        if (!filtered_models.empty()) {
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved filtered models.");
            models_logged_ = true;
            log_ready_ = true; // Enable processing once models are logged
        }
    }
    
    if (!log_ready_) {
        return; // Wait until models are logged before proceeding
    }
    
    if (!entered_ready_) {
        std::cout << "Press Enter to log detections...";
        std::cin.get();
        start_time_ = this->now();
        entered_ready_ = true;
    }
    
    double elapsed_time = (this->now() - start_time_).seconds();
    if (elapsed_time >= timeout_) {
        RCLCPP_INFO(this->get_logger(), "Timeout reached, stopping logging.");
        rclcpp::shutdown();
        return;
    }
    
    auto filtered_models = filtered_model_position_->getFilteredModelPositions({"big_green_plane", "big_transparent_plane", "waffle"});
    writeToFile(msg, filtered_models, elapsed_time);
}

void Exporter::writeToFile(const vision_msgs::msg::Detection3DArray::SharedPtr &detections, 
                           std::vector<model_info_t> &filtered_models, double timestamp) {
    if (!output_file_.is_open()) {
        start_time_ = this->now();
        return;
    }

    model_info_t refrence_pose;
    if (!filtered_models.empty()) {
        refrence_pose = filtered_models.at(0);
        filtered_models.erase(filtered_models.begin());
    } else {
        start_time_ = this->now();
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
    std::string output_file = "output_data.txt";
    std::string detection_topic = "/detected_objects";
    
    if (argc > 1) {
        output_file = argv[1];
    }
    if (argc > 2) {
        detection_topic = argv[2];
    }
    
    auto model_position = std::make_shared<ModelPosition>();
    auto filtered_model_position = std::make_shared<FilteredModelPosition>(model_position, "waffle");
    auto exporter = std::make_shared<Exporter>(filtered_model_position, output_file, 60.0, detection_topic);
    
    rclcpp::spin(exporter);
    rclcpp::shutdown();
    return 0;
}
