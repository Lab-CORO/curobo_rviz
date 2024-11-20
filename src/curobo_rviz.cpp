#include "curobo_rviz/curobo_rviz.hpp"

namespace curobo_rviz
{
  RvizArgsPanel::RvizArgsPanel(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui>())
    , node_{nullptr}
    , param_client_{nullptr}
    , max_attempts_{0}
    , timeout_{0.0}
    , time_dilation_factor_{0.0}
    , voxel_size_{0.0}
    , collision_activation_distance_{0.0}
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Init rclcpp node
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_push_button_node", "--"});
    node_ = std::make_shared<rclcpp::Node>("_", options);
    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "curobo_gen_traj");
    while (!param_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }

    // Connect SpinBox and DoubleSpinBox to slots
    connect(ui_->spinBoxMaxAttempts, SIGNAL(valueChanged(int)), this, SLOT(updateMaxAttempts(int)));
    connect(ui_->doubleSpinBoxTimeout, SIGNAL(valueChanged(double)), this, SLOT(updateTimeout(double)));
    connect(ui_->doubleSpinBoxTimeDilationFactor, SIGNAL(valueChanged(double)), this, SLOT(updateTimeDilationFactor(double)));
    connect(ui_->doubleSpinBoxVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(updateVoxelSize(double)));
    connect(ui_->doubleSpinBoxCollisionActivationDistance, SIGNAL(valueChanged(double)), this, SLOT(updateCollisionActivationDistance(double)));
  }

  RvizArgsPanel::~RvizArgsPanel()
  {
  }
    void RvizArgsPanel::load(const rviz_common::Config &config)
    {
      Panel::load(config);
      auto parameters = param_client_->get_parameters({"max_attempts", "timeout", "time_dilation_factor", "voxel_size", "collision_activation_distance"});
      RCLCPP_INFO(node_->get_logger(), "Parameters received: %ld", parameters.size());

      // set initial values in UI to the values from the parameter server
      ui_->spinBoxMaxAttempts->setValue(parameters[0].as_int());
      ui_->doubleSpinBoxTimeout->setValue(parameters[1].as_double());
      ui_->doubleSpinBoxTimeDilationFactor->setValue(parameters[2].as_double());
      ui_->doubleSpinBoxVoxelSize->setValue(parameters[3].as_double());
      ui_->doubleSpinBoxCollisionActivationDistance->setValue(parameters[4].as_double());
    }

    void RvizArgsPanel::save(rviz_common::Config config) const
    {
      Panel::save(config);
      config.mapSetValue("max_attempts", max_attempts_);
      config.mapSetValue("timeout", timeout_);
      config.mapSetValue("time_dilation_factor", time_dilation_factor_);
      config.mapSetValue("voxel_size", voxel_size_);
      config.mapSetValue("collision_activation_distance", collision_activation_distance_);
    }

    void RvizArgsPanel::updateMaxAttempts(int value)
    {
        max_attempts_ = value;
        while (!param_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }
        
        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("max_attempts", max_attempts_)});
        RCLCPP_INFO(node_->get_logger(), "Max attempts set to %d", max_attempts_);
    }

    void RvizArgsPanel::updateTimeout(double value)
    {
        timeout_ = value;
        while (!param_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }
        
        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("timeout", timeout_)});
        RCLCPP_INFO(node_->get_logger(), "Timeout set to %.2f", timeout_);
    }

    void RvizArgsPanel::updateTimeDilationFactor(double value)
    {
        time_dilation_factor_ = value;
        while (!param_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }
        
        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("time_dilation_factor", time_dilation_factor_)});
        RCLCPP_INFO(node_->get_logger(), "Time dilation factor set to %.2f", time_dilation_factor_);
    }

    void RvizArgsPanel::updateVoxelSize(double value)
    {
        voxel_size_ = value;
        RCLCPP_INFO(node_->get_logger(), "Voxel size changed to %.2f", voxel_size_);
    }

    void RvizArgsPanel::updateCollisionActivationDistance(double value)
    {
        collision_activation_distance_ = value;
        RCLCPP_INFO(node_->get_logger(), "Collision activation distance changed to %.2f", collision_activation_distance_);
    }

    void RvizArgsPanel::on_confirmPushButton_clicked()
    {
        RCLCPP_INFO(node_->get_logger(), "Confirm button clicked.");
        while (!param_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }
        
        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("voxel_size", voxel_size_),
                                       rclcpp::Parameter("collision_activation_distance", collision_activation_distance_)});
        RCLCPP_INFO(node_->get_logger(), "Parameters set:\voxel_size: %.2f, collision_activation_distance: %.2f", voxel_size_, collision_activation_distance_);
        
        // call Trigger service update_motion_gen_config
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = node_->create_client<std_srvs::srv::Trigger>("/curobo_gen_traj/update_motion_gen_config");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        RCLCPP_INFO(node_->get_logger(), "Service call sent.");
        
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Service call successful.");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
        }
    }

} // curobo_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_rviz::RvizArgsPanel, rviz_common::Panel)