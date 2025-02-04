#ifndef MODEL_POSITION_HPP
#define MODEL_POSITION_HPP

#include "object_detection_eval/models_info.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/get_model_list.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/srv/get_link_properties.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <vector>

class ModelPosition : public rclcpp::Node {
public:
    ModelPosition();
    std::vector<model_info_t> getAllModelPositions();

private:
    rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr model_list_client_;
    rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr entity_state_client_;
    rclcpp::Client<gazebo_msgs::srv::GetLinkProperties>::SharedPtr link_properties_client_;
};

#endif // MODEL_POSITION_HPP

