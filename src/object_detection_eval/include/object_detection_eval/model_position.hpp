#ifndef MODEL_POSITION_HPP
#define MODEL_POSITION_HPP

#include "object_detection_eval/models_info.hpp"
#include "object_detection_eval/msg/model_bounding_box.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/get_model_list.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>
#include <unordered_map>

class ModelPosition : public rclcpp::Node {
public:
    ModelPosition();
    std::vector<model_info_t> getAllModelPositions();

private:
    void boundingBoxCallback(const object_detection_eval::msg::ModelBoundingBox::SharedPtr msg);
    void updateModelCache(const std::string &model_name);
    
    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr model_list_client_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr entity_state_client_;
    rclcpp::Subscription<object_detection_eval::msg::ModelBoundingBox>::SharedPtr bounding_box_subscriber_;
    
    std::unordered_map<std::string, model_info_t> model_cache_;
};

#endif // MODEL_POSITION_HPP
