#pragma once

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <curobo_msgs/srv/add_object.hpp>
#include <curobo_msgs/srv/remove_object.hpp>
#include <curobo_msgs/msg/object_parameters.hpp>
#include "std_msgs/msg/string.hpp"
#include <QtWidgets>

#include "curobo_rviz/node_spinner.hpp"

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
        void pollPlannerReady();

    protected:
        void displayMessage(std::string msg);

    private:
        // Runs fn on the Qt GUI thread -- see curobo_rviz::RvizArgsPanel::runOnGuiThread
        // for why this is required once node_ is spun on a background thread.
        void runOnGuiThread(std::function<void()> fn);
        void setPlannerReady(bool ready);

        std::unique_ptr<Ui::gui_objects> ui_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<curobo_msgs::srv::AddObject>::SharedPtr add_object_client_;
        std::shared_ptr<curobo_msgs::srv::AddObject_Request> add_object_request_;
        rclcpp::Client<curobo_msgs::srv::RemoveObject>::SharedPtr remove_object_client_;
        std::shared_ptr<curobo_msgs::srv::RemoveObject_Request> remove_object_request_;
        rclcpp::Publisher<curobo_msgs::msg::ObjectParameters>::SharedPtr add_object_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr remove_object_publisher_;
        QTimer *timerMessage_;

        // Readiness poll against the same planner node's "node_is_available"
        // parameter used by curobo_rviz::RvizArgsPanel -- see that class for why
        // service discovery alone isn't enough during the planner's GPU warmup.
        rclcpp::AsyncParametersClient::SharedPtr param_client_;
        bool planner_ready_;
        bool planner_poll_in_flight_;

        void sendObjectParameters();

        // Declared LAST so it is destroyed FIRST (see curobo_rviz::NodeSpinner).
        std::unique_ptr<curobo_rviz::NodeSpinner> spinner_;
    };
}