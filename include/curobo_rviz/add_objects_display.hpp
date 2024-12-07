#pragma once

#ifndef ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_
#define ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_

#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <curobo_msgs/srv/add_object.hpp>
#include <curobo_msgs/srv/remove_object.hpp>
#include <curobo_msgs/msg/object_parameters.hpp>
#include "std_msgs/msg/string.hpp"
#include <QtWidgets>

namespace add_objects_display
{
    class AddObjectsDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        explicit AddObjectsDisplay();
        ~AddObjectsDisplay();


    protected:
        void onInitialize() override;
        void onAddUpdate(const curobo_msgs::msg::ObjectParameters & request);
        void onRemoveUpdate(const std_msgs::msg::String & request);
        rviz_rendering::Shape::Type getShapeType(const int& type);
    
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<curobo_msgs::msg::ObjectParameters>::SharedPtr add_object_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr remove_object_subscriber_;
        std::unordered_map<std::string, std::unique_ptr<rviz_rendering::Shape>> shapeMap_;
    };
} // namespace add_objects_display

#endif  // ADD_OBJECTS_DISPLAY__ADD_OBJECTS_DISPLAY_HPP_