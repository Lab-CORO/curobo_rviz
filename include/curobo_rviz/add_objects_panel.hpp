#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <curobo_msgs/srv/add_object.hpp>
#include <QtWidgets>

# include <ui_curobo_rviz_panel.h>

namespace add_objects
{
    class AddObjectsPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public: 
        explicit AddObjectsPanel(QWidget *parent = nullptr);
        ~AddObjectsPanel();

    private:
        std::unique_ptr<Ui::gui> ui_;
        rclcpp::Node::SharedPtr node_;
    };
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects::AddObjectsPanel, rviz_common::Panel)