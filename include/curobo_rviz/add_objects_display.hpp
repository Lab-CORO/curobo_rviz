#pragma once

#ifndef ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_
#define ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <curobo_msgs/srv/add_object.hpp>
#include <curobo_msgs/srv/remove_object.hpp>
#include <QtWidgets>

namespace add_objects_display
{
    class AddObjectsDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        explicit AddObjectsDisplay();
        ~AddObjectsDisplay();

    private Q_SLOTS:
        void updateStyle();

    protected:
        void onInitialize() override;
        void onAddUpdate(const curobo_msgs::srv::AddObject_Request::ConstSharedPtr request);
        void onRemoveUpdate(const curobo_msgs::srv::RemoveObject_Request::ConstSharedPtr request);

        std::unique_ptr<rviz_rendering::Shape> shape_;
        std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
    
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<curobo_msgs::srv::AddObject_Request>::SharedPtr add_object_subscriber_;
        rclcpp::Subscription<curobo_msgs::srv::RemoveObject_Request>::SharedPtr remove_object_subscriber_;
    };
} // namespace add_objects_display

#endif  // ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_