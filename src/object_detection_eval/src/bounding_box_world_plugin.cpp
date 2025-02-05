#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include "object_detection_eval/msg/model_bounding_box.hpp"
#include <ignition/math/AxisAlignedBox.hh>
#include <unordered_map>

namespace gazebo
{
    class BoundingBoxWorldPlugin : public WorldPlugin
    {
    public:
        BoundingBoxWorldPlugin() : WorldPlugin() {}

        void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
        {
            this->world = _world;
            this->node = std::make_shared<rclcpp::Node>("bounding_box_publisher");
            this->publisher = this->node->create_publisher<object_detection_eval::msg::ModelBoundingBox>("/gazebo/all_models_bounding_boxes", 10);

            // Lower frequency to cache results
            this->update_rate = 0.1; // Publish every 10 seconds
            this->last_publish_time = this->world->SimTime().Double();

            // Start a ROS 2 thread
            this->rosThread = std::thread(std::bind(&BoundingBoxWorldPlugin::PublishBoundingBoxes, this));
        }

        void PublishBoundingBoxes()
        {
            rclcpp::Rate rate(0.1); // Publish every 10 seconds
            while (rclcpp::ok())
            {
                double current_time = this->world->SimTime().Double();
                if (current_time - this->last_publish_time < this->update_rate)
                {
                    rate.sleep();
                    continue;
                }
                this->last_publish_time = current_time;

                object_detection_eval::msg::ModelBoundingBox msg;
                msg.models.clear();

                for (auto model : this->world->Models())
                {
                    // Directly get the model's bounding box
                    ignition::math::AxisAlignedBox boundingBox = model->BoundingBox();

                    // Compute width, depth, and height from min/max corners
                    object_detection_eval::msg::BoundingBoxModel model_msg;
                    model_msg.name = model->GetName();
                    model_msg.width = boundingBox.Max().X() - boundingBox.Min().X();
                    model_msg.depth = boundingBox.Max().Y() - boundingBox.Min().Y();
                    model_msg.height = boundingBox.Max().Z() - boundingBox.Min().Z();

                    msg.models.push_back(model_msg);
                }

                this->publisher->publish(msg);
                rate.sleep();
            }
        }

    private:
        physics::WorldPtr world;
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Publisher<object_detection_eval::msg::ModelBoundingBox>::SharedPtr publisher;
        std::thread rosThread;
        double update_rate;
        double last_publish_time;
    };

    GZ_REGISTER_WORLD_PLUGIN(BoundingBoxWorldPlugin)
}
