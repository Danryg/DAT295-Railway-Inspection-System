#ifndef EXPORTER_HPP
#define EXPORTER_HPP

#include "object_detection_eval/filtered_model_position.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <string>
#include <vector>

class Exporter : public rclcpp::Node {
public:
    Exporter(std::shared_ptr<FilteredModelPosition> filtered_model_position, const std::string &output_file, double timeout);
    ~Exporter();

private:
    void detectionCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
    void writeToFile(const vision_msgs::msg::Detection3DArray::SharedPtr &detections, std::vector<model_info_t> &filtered_models, double timestamp);
    
    std::shared_ptr<FilteredModelPosition> filtered_model_position_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscriber_;
    std::ofstream output_file_;
    rclcpp::Time start_time_;
    double timeout_;
};

#endif // EXPORTER_HPP
