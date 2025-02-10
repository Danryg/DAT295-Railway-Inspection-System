#include "object_detection_eval/model_position.hpp"
#include "object_detection_eval/models_info.hpp"
#include "object_detection_eval/msg/model_bounding_box.hpp"
#include <chrono>
#include <vector>

ModelPosition::ModelPosition() : Node("model_position_node") {
    model_list_client_ = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
    entity_state_client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");

    // Subscribe to the bounding box topic
    bounding_box_subscriber_ = this->create_subscription<object_detection_eval::msg::ModelBoundingBox>(
        "/gazebo/all_models_bounding_boxes", 10,
        std::bind(&ModelPosition::boundingBoxCallback, this, std::placeholders::_1));
}

void ModelPosition::boundingBoxCallback(const object_detection_eval::msg::ModelBoundingBox::SharedPtr msg) {
    for (const auto &model : msg->models) {
        model_info_t model_info;
        model_info.width = model.width;
        model_info.depth = model.depth;
        model_info.height = model.height;
        model_info.model_name = model.name;
        model_cache_[model.name] = model_info;
    }
}

void ModelPosition::updateModelCache(const std::string &model_name) {
    if (model_cache_.find(model_name) == model_cache_.end()) {
        RCLCPP_WARN(this->get_logger(), "Model %s not found in cache. Waiting for bounding box update.", model_name.c_str());
    }
}

std::vector<model_info_t> ModelPosition::getAllModelPositions() {
    auto model_list_request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
    if (!model_list_client_->wait_for_service(std::chrono::milliseconds(3))) {
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
        updateModelCache(model_name);

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
            if (model_cache_.find(model_name) != model_cache_.end()) {
                model_cache_[model_name].model_pose = pose;
                models_info.push_back(model_cache_[model_name]);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to get position for model: %s", model_name.c_str());
        }
    }
    return models_info;
}

