#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <QtWidgets>

namespace add_objects
{
    class AddObjectsDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        explicit AddObjectsDisplay();
        ~AddObjectsDisplay();
    };
} // add_objects

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects::AddObjectsDisplay, rviz_common::Display)