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
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Projet
#include "curobo_rviz/arrow_interaction.hpp"
#include "curobo_rviz/arrow_interaction_display.hpp"
#include "curobo_rviz/node_spinner.hpp"
#include "curobo_msgs/srv/trajectory_generation.hpp"
#include "curobo_msgs/action/send_trajectory.hpp"
#include "curobo_msgs/srv/get_voxel_grid.hpp"
#include "curobo_msgs/msg/sparse_voxel_grid.hpp"
#include "curobo_msgs/srv/set_planner.hpp"
#include "curobo_msgs/srv/set_robot_strategy.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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
#include <mutex>
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

    // Planner type slots
    void on_comboBoxTrajectoryType_currentIndexChanged(int index);

    // MPC tracking slots
    void publishMpcGoal();  // Timer callback for continuous goal publishing

    // Helper methods for quaternion <-> Euler conversion
    void quaternionToEuler(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);
    void eulerToQuaternion(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion& q);

  private:
    // Runs fn on the Qt GUI thread. ROS callbacks fire on the background spin thread
    // (see spinner_ below) and must never touch QWidgets directly. Uses the
    // context-object overload of invokeMethod so Qt drops the call if `this` is
    // destroyed before the event loop gets to it.
    void runOnGuiThread(std::function<void()> fn);

    // Single source of truth for "is the planner actually responding" (not just
    // discoverable -- see node_is_available poll in the constructor/pollPlannerReady).
    // Gates every planner-facing widget so a click can never queue a request against
    // a not-yet-responsive planner (e.g. during its ~90s GPU warmup, or after it
    // respawns following a robot reboot).
    void setPlannerReady(bool ready);
    void pollPlannerReady();

    // Combines planner_ready_ with "is a goal currently executing" to drive
    // generateTrajectory/sendTrajectory/generateAndSend/stopRobot enabled state.
    void updateActionButtons();

    // Shared by on_generateTrajectory_clicked and the classic-mode branch of
    // on_generateAndSend_clicked (which used to chain onto a blocking generate call
    // via a fragile QTimer::singleShot(500, ...) guess -- now chains on the real
    // async completion instead).
    void generateTrajectoryAsync(std::function<void(bool)> on_done);

    // voxel_grid_sparse topic subscription callback (fires on the background spin
    // thread, spinner_) -- just caches the latest message under a mutex. The
    // auto-refresh timer (updateObstaclesFromTimer) reads that cache on the GUI
    // thread at the user-configured cadence, decoupling arrival rate (capped by
    // whatever the planner publishes at) from render rate.
    void onSparseVoxelGrid(curobo_msgs::msg::SparseVoxelGrid::ConstSharedPtr msg);

    // Builds a CUBE_LIST marker from a sparse grid's occupied_indices (C-order
    // linear -> x,y,z) and publishes it, mirroring the dense-grid path in
    // on_pushButtonUpdateObstacles_clicked but without the O(size_x*size_y*size_z)
    // scan.
    void publishMarkerFromSparse(const curobo_msgs::msg::SparseVoxelGrid & msg);

    std::unique_ptr<Ui::gui_parameters> ui_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    bool planner_ready_;
    bool planner_poll_in_flight_;
    bool goal_active_;
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_marker_array_pub_;
    std::string voxel_frame_id_;
    QTimer* obstacle_update_timer_;
    double obstacle_update_frequency_;

    // Auto-refresh path: persistent subscription to the planner's voxel_grid_sparse
    // topic (server-paced, currently ~7Hz) plus the latest message it delivered.
    // updateObstaclesFromTimer() renders from this cache instead of round-tripping
    // GetVoxelGrid on every tick; the manual "Update Obstacles" button still uses
    // the service directly (see on_pushButtonUpdateObstacles_clicked), since that's
    // a one-shot pull where paying for a fresh dense grid is fine.
    rclcpp::Subscription<curobo_msgs::msg::SparseVoxelGrid>::SharedPtr voxel_grid_sparse_sub_;
    std::mutex latest_sparse_mutex_;
    curobo_msgs::msg::SparseVoxelGrid::ConstSharedPtr latest_sparse_msg_;

    // Control strategy members (emulator / joint_speed / joint_pose — string key)
    rclcpp::Client<curobo_msgs::srv::SetRobotStrategy>::SharedPtr set_robot_strategy_client_;
    std::string current_robot_strategy_;

    // Planner type members
    rclcpp::Client<curobo_msgs::srv::SetPlanner>::SharedPtr set_planner_client_;
    uint8_t current_planner_type_;

    // MPC tracking members
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mpc_goal_pub_;
    QTimer* mpc_goal_publisher_timer_;
    bool is_mpc_tracking_active_;

    // Declared LAST so it is destroyed FIRST (members are torn down in reverse
    // declaration order): stops and joins the background spin thread before any
    // client/publisher/node it might still be delivering callbacks against is
    // destroyed.
    std::unique_ptr<NodeSpinner> spinner_;
  };
} // curobo_rviz