#pragma once
#include <iostream>
#include <functional>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

// Projet
#include "curobo_rviz/arrow_interaction.hpp"
#include "curobo_rviz/arrow_interaction_display.hpp"
#include "curobo_msgs/srv/trajectory_generation.hpp"
#include "curobo_msgs/action/send_trajectory.hpp"
#include "curobo_msgs/srv/get_voxel_grid.hpp"
// #include "curobo_msgs/srv/set_robot_strategy.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

// RVIZ2
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>
// Qt
#include <QtWidgets>
// STL
#include <memory>
/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_curobo_rviz_panel.h>

namespace curobo_rviz
{
  class RvizArgsPanel : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    explicit RvizArgsPanel(QWidget *parent = nullptr);
    ~RvizArgsPanel();

    /// Load and save configuration data
    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;

    /// Event filter to detect when user starts editing pose spinboxes
    bool eventFilter(QObject *obj, QEvent *event) override;

  private Q_SLOTS:
    void updateTimeDilationFactor(double value);
    void updateVoxelSize(double value);
    void updateParameters();
    void on_confirmPushButton_clicked();
    void on_sendTrajectory_clicked();
    void on_generateTrajectory_clicked();
    void on_generateAndSend_clicked();
    void on_stopRobot_clicked();
    void result_callback(const rclcpp_action::ClientGoalHandle<curobo_msgs::action::SendTrajectory>::WrappedResult & result);
    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<curobo_msgs::action::SendTrajectory>> goal_handle);
      // void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<actionfaces::action::Fibonacci>::SharedPtr> future)

    // Marker control slots
    void updateMarkerPoseDisplay();
    void findArrowInteractionDisplay();
    void applyPoseFromSpinboxes();

    // Obstacle update slots
    void on_pushButtonUpdateObstacles_clicked();
    void updateObstacleFrequency(double value);
    void updateObstaclesFromTimer();

    // Robot strategy slots
    void on_comboBoxRobotStrategy_currentTextChanged(const QString &text);

    // Helper methods for quaternion <-> Euler conversion
    void quaternionToEuler(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);
    void eulerToQuaternion(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion& q);

  private:
    std::unique_ptr<Ui::gui_parameters> ui_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::SyncParametersClient::SharedPtr param_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr motion_gen_config_client_;
    std::shared_ptr<std_srvs::srv::Trigger::Request> motion_gen_config_request_;
    rclcpp_action::Client<curobo_msgs::action::SendTrajectory>::SharedPtr action_ptr_;
    rclcpp::Client<curobo_msgs::srv::TrajectoryGeneration>::SharedPtr trajectory_generation_client_;
    rclcpp_action::Client<curobo_msgs::action::SendTrajectory>::GoalHandle::SharedPtr goal_handle_;
    float time_dilation_factor_, voxel_size_;
    std::shared_ptr<ArrowInteraction> arrow_interaction_;
    bool user_editing_pose_; // Flag to prevent auto-update while user is editing

    // Last displayed pose to avoid unnecessary updates
    double last_displayed_x_;
    double last_displayed_y_;
    double last_displayed_z_;
    double last_displayed_roll_;
    double last_displayed_pitch_;
    double last_displayed_yaw_;

    // Obstacle update members
    rclcpp::Client<curobo_msgs::srv::GetVoxelGrid>::SharedPtr get_voxel_grid_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr voxel_marker_pub_;
    QTimer* obstacle_update_timer_;
    double obstacle_update_frequency_;

    // Robot strategy members
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr set_robot_strategy_client_;
    std::string current_robot_strategy_;

  };
} // curobo_rviz