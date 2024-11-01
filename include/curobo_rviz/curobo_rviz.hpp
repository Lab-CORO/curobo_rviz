#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
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

  private:
    std::unique_ptr<Ui::gui> ui_;
    rclcpp::Node::SharedPtr node_;
    int max_attempts_;
    float timeout_, time_dilation_factor_;

  protected:
    std_msgs::msg::Bool msg_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr max_attempts_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timeout_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_dilation_factor_pub_;
  };
} // curobo_rviz