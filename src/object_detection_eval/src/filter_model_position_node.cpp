#include "object_detection_eval/filtered_model_position.hpp"
#include "object_detection_eval/model_position.hpp"
#include "object_detection_eval/models_info.hpp"
#include <chrono>
#include <vector>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto model_position = std::make_shared<ModelPosition>();
  FilteredModelPosition filtered_model_position(model_position, "waffle");

  std::vector<std::string> exclude_models = {"big_green_plane",
                                             "big_transparent_plane", "waffle"};

  rclcpp::Rate loop_rate(1);
  while (rclcpp::ok()) {
    auto filtered_models =
        filtered_model_position.getFilteredModelPositions(exclude_models);
    for (const auto &model : filtered_models) {
      RCLCPP_INFO(
          rclcpp::get_logger("FilteredModel"),
          "Model: %s, Distance: %.2f, Width: %.2f, Height: %.2f, Depth: %.2f",
          model.model_name.c_str(), model.distance_from_base, model.width, model.height, model.depth);
    }
    rclcpp::spin_some(model_position);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
