#include "object_detection_eval/model_position.hpp"
#include "object_detection_eval/models_info.hpp"
#include <chrono>
#include <vector>

ModelPosition::ModelPosition() : Node("model_position_node") {
    model_list_client_ = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
    entity_state_client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
    link_properties_client_ = this->create_client<gazebo_msgs::srv::GetLinkProperties>("/gazebo/get_link_properties");
}

std::vector<model_info_t> ModelPosition::getAllModelPositions() {
    auto model_list_request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();

    if (!model_list_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Service /get_model_list not available.");
        return {};
    }


    auto model_list_result = model_list_client_->async_send_request(model_list_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), model_list_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get model list.");
        return {};
    }

    auto model_names = model_list_result.get()->model_names;
    std::vector<model_info_t> models_info;

    for (const auto &model_name : model_names) {
        auto entity_state_request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        entity_state_request->name = model_name;


        if (!entity_state_client_->wait_for_service(std::chrono::milliseconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Service /gazebo/get_entity_state not available.");
            return {};
        }

        auto entity_state_result = entity_state_client_->async_send_request(entity_state_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), entity_state_result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto pose = entity_state_result.get()->state.pose;
            model_info_t model_info;
            model_info.model_pose = pose;
            model_info.model_name = model_name;
            
            auto link_properties_request = std::make_shared<gazebo_msgs::srv::GetLinkProperties::Request>();
            link_properties_request->link_name = model_name + "::base_link";
            
            if (link_properties_client_->wait_for_service(std::chrono::milliseconds(3))) {
                auto link_properties_result = link_properties_client_->async_send_request(link_properties_request);
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), link_properties_result) ==
                    rclcpp::FutureReturnCode::SUCCESS) {
                    
                    // Get dimensions from the response
                    model_info.width = link_properties_result.get()->com.position.x * 2;
                    model_info.depth = link_properties_result.get()->com.position.y * 2;
                    model_info.height = link_properties_result.get()->com.position.z * 2;
                }
            }
            models_info.push_back(model_info);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to get position for model: %s", model_name.c_str());
        }
    }
    return models_info;
}
