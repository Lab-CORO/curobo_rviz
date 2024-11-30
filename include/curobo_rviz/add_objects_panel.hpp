#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <curobo_msgs/srv/add_object.hpp>
#include <QtWidgets>

# include <ui_add_object_panel.h>

namespace add_objects_panel
{
    class AddObjectsPanel : public rviz_common::Panel
    {
        Q_OBJECT
    public: 
        explicit AddObjectsPanel(QWidget *parent = nullptr);
        ~AddObjectsPanel();

    private Q_SLOTS:
        void on_pushButtonAdd_clicked();
        void on_pushButtonRemove_clicked();

    private:
        std::unique_ptr<Ui::gui> ui_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<curobo_msgs::srv::AddObject>::SharedPtr add_object_client_;
        rclcpp::Publisher<curobo_msgs::srv::AddObject_Request>::SharedPtr add_object_publisher_;
    };
}