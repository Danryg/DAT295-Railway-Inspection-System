#include "object_detection_eval/model_position.hpp"

ModelPosition::ModelPosition() : Node("model_position_node") {
    // Create service clients
    model_list_client_ = this->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
    entity_state_client_ = this->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");
}

void ModelPosition::getAllModelPositions() {
    auto model_list_request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();

    // Wait for the service to be available
    if (!model_list_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Service /get_model_list not available.");
        return;
    }

    // Send request and get the response
    auto model_list_result = model_list_client_->async_send_request(model_list_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), model_list_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get model list.");
        return;
    }

    auto model_names = model_list_result.get()->model_names;

    for (const auto &model_name : model_names) {
        auto entity_state_request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        entity_state_request->name = model_name;

        if (!entity_state_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Service /gazebo/get_entity_state not available.");
            return;
        }

        auto entity_state_result = entity_state_client_->async_send_request(entity_state_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), entity_state_result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto pose = entity_state_result.get()->state.pose;
            RCLCPP_INFO(this->get_logger(), "Model: %s, Position -> x: %.2f, y: %.2f, z: %.2f",
                        model_name.c_str(), pose.position.x, pose.position.y, pose.position.z);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to get position for model: %s", model_name.c_str());
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelPosition>();

    rclcpp::Rate loop_rate(1); // Run at 1 Hz
    while (rclcpp::ok()) {
        node->getAllModelPositions();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

