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
#include "curobo_msgs/srv/trajectory_generation.hpp"
#include "curobo_msgs/action/send_trajectory.hpp"

// RVIZ2
#include <rviz_common/panel.hpp>
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

  private Q_SLOTS:
    void updateMaxAttempts(int value);
    void updateTimeout(double value);
    void updateTimeDilationFactor(double value);
    void updateVoxelSize(double value);
    void updateCollisionActivationDistance(double value);
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
    void on_pushButtonApplyFrameId_clicked();
    void on_pushButtonResetMarker_clicked();
    void on_checkBoxMarkerVisible_stateChanged(int state);
    void updateMarkerPoseDisplay();
  private:
    std::unique_ptr<Ui::gui_parameters> ui_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::SyncParametersClient::SharedPtr param_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr motion_gen_config_client_;
    std::shared_ptr<std_srvs::srv::Trigger::Request> motion_gen_config_request_;
    rclcpp_action::Client<curobo_msgs::action::SendTrajectory>::SharedPtr action_ptr_;
    rclcpp::Client<curobo_msgs::srv::TrajectoryGeneration>::SharedPtr trajectory_generation_client_;
    rclcpp_action::Client<curobo_msgs::action::SendTrajectory>::GoalHandle::SharedPtr goal_handle_;
    int max_attempts_;
    float timeout_, time_dilation_factor_, voxel_size_, collision_activation_distance_;
    std::shared_ptr<ArrowInteraction> arrow_interaction_; 

  };
} // curobo_rviz