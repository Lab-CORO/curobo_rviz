#pragma once

#ifndef ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_
#define ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <curobo_msgs/srv/add_object.hpp>
#include <QtWidgets>

namespace add_objects_display
{
    class AddObjectsDisplay : public rviz_common::RosTopicDisplay<curobo_msgs::srv::AddObject_Request>
    {
        Q_OBJECT
    public:
        explicit AddObjectsDisplay();
        ~AddObjectsDisplay();
    protected:
        void onInitialize() override;
        void onEnable() override;
        void onDisable() override;
        void processMessage(const curobo_msgs::srv::AddObject_Request::ConstSharedPtr request) override;
        void updateStyle();

        std::unique_ptr<rviz_rendering::Shape> shape_;
        std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
    
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscriber<curobo_msgs::srv::AddObject_Request>::SharedPtr add_object_subscriber_;
    };
} // namespace add_objects_display

#endif  // ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_