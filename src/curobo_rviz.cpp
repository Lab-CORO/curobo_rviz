#include "curobo_rviz/curobo_rviz.hpp"

namespace curobo_rviz
{
  RvizArgsPanel::RvizArgsPanel(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui>())
    , node_{nullptr}
    , max_attempts_{0}
    , timeout_{0.0}
    , time_dilation_factor_{0.0}
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Init rclcpp node
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_push_button_node", "--"});
    node_ = std::make_shared<rclcpp::Node>("_", options);

    // Prepare msg
    msg_.data = true;

    // Connect SpinBox and DoubleSpinBox to slots
    connect(ui_->spinBox_1, SIGNAL(valueChanged(int)), this, SLOT(updateMaxAttempts(int)));
    connect(ui_->doubleSpinBox_1, SIGNAL(valueChanged(double)), this, SLOT(updateTimeout(double)));
    connect(ui_->doubleSpinBox_2, SIGNAL(valueChanged(double)), this, SLOT(updateTimeDilationFactor(double)));
  }

  RvizArgsPanel::~RvizArgsPanel()
  {
  }
    void RvizArgsPanel::load(const rviz_common::Config &config)
    {
      Panel::load(config);
      int max_attempts;
      float timeout, time_dilation;

      if (max_attempts = config.mapGetInt("max_attempts", &max_attempts))
      {
        ui_->spinBox_1->setValue(max_attempts);
      }

      if (timeout = config.mapGetFloat("timeout", &timeout))
      {
        ui_->doubleSpinBox_1->setValue(timeout);
      }

      if (time_dilation = config.mapGetFloat("time_dilation_factor", &time_dilation))
      {
          ui_->doubleSpinBox_2->setValue(time_dilation);
      }
    }

    void RvizArgsPanel::save(rviz_common::Config config) const
    {
      Panel::save(config);
      config.mapSetValue("max_attempts", max_attempts_);
      config.mapSetValue("timeout", timeout_);
      config.mapSetValue("time_dilation_factor", time_dilation_factor_);
    }

    void RvizArgsPanel::updateMaxAttempts(int value)
    {
        max_attempts_ = value;
        RCLCPP_INFO(node_->get_logger(), "Max attempts set to %d", max_attempts_);
    }

    void RvizArgsPanel::updateTimeout(double value)
    {
        timeout_ = value;
        RCLCPP_INFO(node_->get_logger(), "Timeout set to %.2f", timeout_);
    }

    void RvizArgsPanel::updateTimeDilationFactor(double value)
    {
        time_dilation_factor_ = value;
        RCLCPP_INFO(node_->get_logger(), "Time dilation factor set to %.2f", time_dilation_factor_);
    }

} // curobo_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_rviz::RvizArgsPanel, rviz_common::Panel)