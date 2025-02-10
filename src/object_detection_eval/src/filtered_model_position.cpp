#include "object_detection_eval/filtered_model_position.hpp"
#include "object_detection_eval/model_position.hpp"
#include <cmath>

FilteredModelPosition::FilteredModelPosition(
    std::shared_ptr<ModelPosition> model_position,
    const std::string &reference_model)
    : model_position_(model_position), reference_model_(reference_model) {}

std::vector<model_info_t> FilteredModelPosition::getFilteredModelPositions(
    const std::vector<std::string> &exclude_models) {
  auto models = model_position_->getAllModelPositions();
  std::vector<model_info_t> filtered_models;

  geometry_msgs::msg::Pose reference_pose;
  model_info_t reference_model_info;
  bool reference_found = false;

  for (const auto &model : models) {
    if (model.model_name == reference_model_) {
      reference_pose = model.model_pose;
      reference_model_info = model;
      reference_found = true;
      break;
    }
  }

  if (reference_found) {
    filtered_models.push_back(reference_model_info); // Push reference model first
  }
  
  for (const auto &model : models) {
    if (std::find(exclude_models.begin(), exclude_models.end(),
                  model.model_name) != exclude_models.end()) {
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

