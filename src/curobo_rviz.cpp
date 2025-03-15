#include "curobo_rviz/curobo_rviz.hpp"

namespace curobo_rviz
{
  RvizArgsPanel::RvizArgsPanel(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui_parameters>())
    , node_{nullptr}
    , param_client_{nullptr}
    , motion_gen_config_client_{nullptr}
    , motion_gen_config_request_{nullptr}
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
        {"--ros-args", "--remap", "__node:=rviz_updata_parameters_node", "--"});
    node_ = std::make_shared<rclcpp::Node>("_", options);
    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "curobo_gen_traj");

    motion_gen_config_client_ = node_->create_client<std_srvs::srv::Trigger>("/curobo_gen_traj/update_motion_gen_config");
    motion_gen_config_request_ = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Connect SpinBox and DoubleSpinBox to slots
    connect(ui_->spinBoxMaxAttempts, SIGNAL(valueChanged(int)), this, SLOT(updateMaxAttempts(int)));
    connect(ui_->doubleSpinBoxTimeout, SIGNAL(valueChanged(double)), this, SLOT(updateTimeout(double)));
    connect(ui_->doubleSpinBoxTimeDilationFactor, SIGNAL(valueChanged(double)), this, SLOT(updateTimeDilationFactor(double)));
    connect(ui_->doubleSpinBoxVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(updateVoxelSize(double)));
    connect(ui_->doubleSpinBoxCollisionActivationDistance, SIGNAL(valueChanged(double)), this, SLOT(updateCollisionActivationDistance(double)));
    
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        // Replace this condition with your actual check
        if(param_client_->wait_for_service(std::chrono::milliseconds(10))){
            auto conditionMet = param_client_->get_parameters({"node_is_available"});
            // Enable or disable the widget based on the condition
            ui_->spinBoxMaxAttempts->setEnabled(conditionMet[0].as_bool());
            ui_->doubleSpinBoxTimeout->setEnabled(conditionMet[0].as_bool());
            ui_->doubleSpinBoxTimeDilationFactor->setEnabled(conditionMet[0].as_bool());
            ui_->doubleSpinBoxVoxelSize->setEnabled(conditionMet[0].as_bool());
            ui_->doubleSpinBoxCollisionActivationDistance->setEnabled(conditionMet[0].as_bool());
            ui_->confirmPushButton->setEnabled(conditionMet[0].as_bool());
        }else{
            ui_->spinBoxMaxAttempts->setEnabled(false);
            ui_->doubleSpinBoxTimeout->setEnabled(false);
            ui_->doubleSpinBoxTimeDilationFactor->setEnabled(false);
            ui_->doubleSpinBoxVoxelSize->setEnabled(false);
            ui_->doubleSpinBoxCollisionActivationDistance->setEnabled(false);
            ui_->confirmPushButton->setEnabled(false);
        }
    });
    timer->start(100); // checks every 1000 milliseconds (1 second)
  }

  RvizArgsPanel::~RvizArgsPanel()
  {
  }
    void RvizArgsPanel::load(const rviz_common::Config &config)
    {
      Panel::load(config);
      // set initial values in UI to the values from the parameter server
      
      ui_->spinBoxMaxAttempts->setValue(this->node_->get_parameter("max_attempts").as_int());
      ui_->doubleSpinBoxTimeout->setValue(this->node_->get_parameter("timeout").as_double());
      ui_->doubleSpinBoxTimeDilationFactor->setValue(this->node_->get_parameter("time_dilation_factor").as_double());
      ui_->doubleSpinBoxVoxelSize->setValue(this->node_->get_parameter("voxel_size").as_double());
      ui_->doubleSpinBoxCollisionActivationDistance->setValue(this->node_->get_parameter("collision_activation_distance").as_double());
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
        
        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("max_attempts", max_attempts_)});
        RCLCPP_INFO(node_->get_logger(), "Max attempts set to %d", max_attempts_);
    }

    void RvizArgsPanel::updateTimeout(double value)
    {
        timeout_ = value;
        
        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("timeout", timeout_)});
        RCLCPP_INFO(node_->get_logger(), "Timeout set to %.2f", timeout_);
    }

    void RvizArgsPanel::updateTimeDilationFactor(double value)
    {
        time_dilation_factor_ = value;
       
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
        if (!ui_->confirmPushButton->isEnabled()) {
            return;
          }
        // Set ui to disable
        ui_->spinBoxMaxAttempts->setEnabled(false);
        ui_->doubleSpinBoxTimeout->setEnabled(false);
        ui_->doubleSpinBoxTimeDilationFactor->setEnabled(false);
        ui_->doubleSpinBoxVoxelSize->setEnabled(false);
        ui_->doubleSpinBoxCollisionActivationDistance->setEnabled(false);
        ui_->confirmPushButton->setEnabled(false);
        RCLCPP_INFO(node_->get_logger(), "Confirm button clicked.");
        if (!param_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(node_->get_logger(), "service not available");
            return;
        }
        
        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("voxel_size", voxel_size_),
                                       rclcpp::Parameter("collision_activation_distance", collision_activation_distance_)});
        RCLCPP_INFO(node_->get_logger(), "Parameters set:\voxel_size: %.2f, collision_activation_distance: %.2f", voxel_size_, collision_activation_distance_);
        

        if(!motion_gen_config_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(node_->get_logger(), "service not available");
            return;
        }
            // Make the asynchronous (non-blocking) call.
        motion_gen_config_client_->async_send_request(motion_gen_config_request_,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response_future) {
          auto response = response_future.get();
          RCLCPP_INFO(node_->get_logger(), "Service call successful");
          ui_->spinBoxMaxAttempts->setEnabled(true);
          ui_->doubleSpinBoxTimeout->setEnabled(true);
          ui_->doubleSpinBoxTimeDilationFactor->setEnabled(true);
          ui_->doubleSpinBoxVoxelSize->setEnabled(true);
          ui_->doubleSpinBoxCollisionActivationDistance->setEnabled(true);
          ui_->confirmPushButton->setEnabled(true);
        });

    }


} // curobo_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_rviz::RvizArgsPanel, rviz_common::Panel)