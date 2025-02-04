#ifndef MODELS_INFO_H_
#define MODELS_INFO_H_

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <string>

typedef struct {
    geometry_msgs::msg::Pose model_pose;
    float distance_from_base;
    float width;
    float height;
    float depth;
    std::string model_name;
} model_info_t;

#endif /* MODELS_INFO_H_ */

