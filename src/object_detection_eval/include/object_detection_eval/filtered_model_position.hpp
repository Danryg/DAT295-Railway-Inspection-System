#ifndef FILTERED_MODEL_POSITION_HPP
#define FILTERED_MODEL_POSITION_HPP

#include "object_detection_eval/model_position.hpp"
#include "object_detection_eval/models_info.hpp"
#include <vector>

class FilteredModelPosition {
public:
    FilteredModelPosition(std::shared_ptr<ModelPosition> model_position, const std::string& reference_model);
    std::vector<model_info_t> getFilteredModelPositions(const std::vector<std::string>& exclude_models);

private:
    std::shared_ptr<ModelPosition> model_position_;
    std::string reference_model_;
};

#endif // FILTERED_MODEL_POSITION_HPP
