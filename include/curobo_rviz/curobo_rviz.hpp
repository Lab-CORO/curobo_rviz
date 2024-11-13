#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
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

  private:
    std::unique_ptr<Ui::gui> ui_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::SyncParametersClient::SharedPtr param_client_;
    int max_attempts_;
    float timeout_, time_dilation_factor_, voxel_size_, collision_activation_distance_;
  };
} // curobo_rviz