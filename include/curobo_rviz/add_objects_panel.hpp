#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <curobo_msgs/srv/add_object.hpp>
#include <curobo_msgs/srv/remove_object.hpp>
#include <curobo_msgs/msg/object_parameters.hpp>
#include "std_msgs/msg/string.hpp"
#include <QtWidgets>

#include <ui_add_object_panel.h>

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

    protected:
        void displayMessage(std::string msg);

    private:
        std::unique_ptr<Ui::gui_objects> ui_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<curobo_msgs::srv::AddObject>::SharedPtr add_object_client_;
        std::shared_ptr<curobo_msgs::srv::AddObject_Request> add_object_request_;
        rclcpp::Client<curobo_msgs::srv::RemoveObject>::SharedPtr remove_object_client_;
        std::shared_ptr<curobo_msgs::srv::RemoveObject_Request> remove_object_request_;
        rclcpp::Publisher<curobo_msgs::msg::ObjectParameters>::SharedPtr add_object_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr remove_object_publisher_;
        QTimer *timerMessage_;

        void sendObjectParameters();
    };
}