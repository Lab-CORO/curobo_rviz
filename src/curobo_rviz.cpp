#include "curobo_rviz/curobo_rviz.hpp"
#include <cmath>

namespace curobo_rviz
{
  RvizArgsPanel::RvizArgsPanel(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui_parameters>())
    , node_{nullptr}
    , param_client_{nullptr}
    , motion_gen_config_client_{nullptr}
    , motion_gen_config_request_{nullptr}
    , time_dilation_factor_{0.0}
    , voxel_size_{0.0}
    , arrow_interaction_{nullptr}
    , user_editing_pose_{false}
    , last_displayed_x_{std::numeric_limits<double>::quiet_NaN()}
    , last_displayed_y_{std::numeric_limits<double>::quiet_NaN()}
    , last_displayed_z_{std::numeric_limits<double>::quiet_NaN()}
    , last_displayed_roll_{std::numeric_limits<double>::quiet_NaN()}
    , last_displayed_pitch_{std::numeric_limits<double>::quiet_NaN()}
    , last_displayed_yaw_{std::numeric_limits<double>::quiet_NaN()}
    , get_voxel_grid_client_{nullptr}
    , voxel_marker_pub_{nullptr}
    , obstacle_update_timer_{nullptr}
    , obstacle_update_frequency_{0.0}
  {
    // Extend the widget with all attributes and children from UI file
    ui_->setupUi(this);

    // Init rclcpp node
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_updata_parameters_node", "--"});
    node_ = std::make_shared<rclcpp::Node>("_", options);

    // Try to find ArrowInteractionDisplay, will be set by timer if not immediately available
    this->arrow_interaction_ = nullptr;

    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "curobo_gen_traj");

    motion_gen_config_client_ = node_->create_client<std_srvs::srv::Trigger>("/curobo_gen_traj/update_motion_gen_config");
    motion_gen_config_request_ = std::make_shared<std_srvs::srv::Trigger::Request>();

    // action client
    this->action_ptr_ = rclcpp_action::create_client<curobo_msgs::action::SendTrajectory>(
      node_,
      "/curobo_gen_traj/send_trajectrory");

    // create service client to generate traj
    this->trajectory_generation_client_ = node_->create_client<curobo_msgs::srv::TrajectoryGeneration>("/curobo_gen_traj/generate_trajectory");

    // Create service client for getting voxel grid
    this->get_voxel_grid_client_ = node_->create_client<curobo_msgs::srv::GetVoxelGrid>("/curobo_gen_traj/get_voxel_grid");

    // Create publisher for voxel grid visualization
    this->voxel_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualise_voxel_grid", 10);

    // Create timer for automatic obstacle updates
    obstacle_update_timer_ = new QTimer(this);
    connect(obstacle_update_timer_, &QTimer::timeout, this, &RvizArgsPanel::updateObstaclesFromTimer);

    // Create service client for setting robot strategy
    this->set_robot_strategy_client_ = node_->create_client<std_srvs::srv::Trigger>("/curobo_gen_traj/set_robot_strategy");
    current_robot_strategy_ = "";

    // Connect SpinBox and DoubleSpinBox to slots
    connect(ui_->doubleSpinBoxTimeDilationFactor, SIGNAL(valueChanged(double)), this, SLOT(updateTimeDilationFactor(double)));
    connect(ui_->doubleSpinBoxVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(updateVoxelSize(double)));
    
    ui_->stopRobot->setEnabled(false);

    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        // Replace this condition with your actual check
        if(param_client_->wait_for_service(std::chrono::milliseconds(10))){
            auto conditionMet = param_client_->get_parameters({"node_is_available"});
            // Enable or disable the widget based on the condition
            ui_->doubleSpinBoxTimeDilationFactor->setEnabled(conditionMet[0].as_bool());
            ui_->doubleSpinBoxVoxelSize->setEnabled(conditionMet[0].as_bool());
            ui_->confirmPushButton->setEnabled(conditionMet[0].as_bool());
        }else{
            ui_->doubleSpinBoxTimeDilationFactor->setEnabled(false);
            ui_->doubleSpinBoxVoxelSize->setEnabled(false);
            ui_->confirmPushButton->setEnabled(false);
        }
    });
    timer->start(100); // checks every 1000 milliseconds (1 second)

    // Connect pose spinboxes to apply changes in real-time
    // Use valueChanged to update immediately when user changes value (arrows, wheel, typing)
    connect(ui_->spinBoxPosX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
      applyPoseFromSpinboxes();
    });
    connect(ui_->spinBoxPosY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
      applyPoseFromSpinboxes();
    });
    connect(ui_->spinBoxPosZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
      applyPoseFromSpinboxes();
    });

    // Connect orientation spinboxes
    connect(ui_->spinBoxRoll, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
      applyPoseFromSpinboxes();
    });
    connect(ui_->spinBoxPitch, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
      applyPoseFromSpinboxes();
    });
    connect(ui_->spinBoxYaw, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
      applyPoseFromSpinboxes();
    });

    // Detect when spinbox gets focus (user starts editing) to pause auto-update
    connect(ui_->spinBoxPosX, &QDoubleSpinBox::editingFinished, this, [this]() { user_editing_pose_ = false; });
    connect(ui_->spinBoxPosY, &QDoubleSpinBox::editingFinished, this, [this]() { user_editing_pose_ = false; });
    connect(ui_->spinBoxPosZ, &QDoubleSpinBox::editingFinished, this, [this]() { user_editing_pose_ = false; });
    connect(ui_->spinBoxRoll, &QDoubleSpinBox::editingFinished, this, [this]() { user_editing_pose_ = false; });
    connect(ui_->spinBoxPitch, &QDoubleSpinBox::editingFinished, this, [this]() { user_editing_pose_ = false; });
    connect(ui_->spinBoxYaw, &QDoubleSpinBox::editingFinished, this, [this]() { user_editing_pose_ = false; });

    ui_->spinBoxPosX->installEventFilter(this);
    ui_->spinBoxPosY->installEventFilter(this);
    ui_->spinBoxPosZ->installEventFilter(this);
    ui_->spinBoxRoll->installEventFilter(this);
    ui_->spinBoxPitch->installEventFilter(this);
    ui_->spinBoxYaw->installEventFilter(this);

    // Timer to update marker pose display
    QTimer* poseUpdateTimer = new QTimer(this);
    connect(poseUpdateTimer, &QTimer::timeout, this, &RvizArgsPanel::updateMarkerPoseDisplay);
    poseUpdateTimer->start(100); // Update pose display every 100ms

    // Timer to find ArrowInteractionDisplay
    QTimer* findDisplayTimer = new QTimer(this);
    connect(findDisplayTimer, &QTimer::timeout, this, &RvizArgsPanel::findArrowInteractionDisplay);
    findDisplayTimer->start(500); // Check every 500ms until found

    // Connect obstacle update controls
    connect(ui_->pushButtonUpdateObstacles, &QPushButton::clicked, this, &RvizArgsPanel::on_pushButtonUpdateObstacles_clicked);
    connect(ui_->spinBoxUpdateFrequency, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &RvizArgsPanel::updateObstacleFrequency);

    // Connect robot strategy controls
    connect(ui_->comboBoxRobotStrategy, &QComboBox::currentTextChanged, this, &RvizArgsPanel::on_comboBoxRobotStrategy_currentTextChanged);
  }

  RvizArgsPanel::~RvizArgsPanel()
  {
  }
    void RvizArgsPanel::load(const rviz_common::Config &config)
    {
      Panel::load(config);

      // Load values from RViz config file
      float time_dilation_factor;
      if (config.mapGetFloat("time_dilation_factor", &time_dilation_factor)) {
        time_dilation_factor_ = time_dilation_factor;
        ui_->doubleSpinBoxTimeDilationFactor->setValue(time_dilation_factor);
      }

      float voxel_size;
      if (config.mapGetFloat("voxel_size", &voxel_size)) {
        voxel_size_ = voxel_size;
        ui_->doubleSpinBoxVoxelSize->setValue(voxel_size);
      }
    }

    void RvizArgsPanel::save(rviz_common::Config config) const
    {
      Panel::save(config);
      config.mapSetValue("time_dilation_factor", time_dilation_factor_);
      config.mapSetValue("voxel_size", voxel_size_);
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

    void RvizArgsPanel::on_confirmPushButton_clicked()
    {
        if (!ui_->confirmPushButton->isEnabled()) {
            return;
          }
        // Set ui to disable
        ui_->doubleSpinBoxTimeDilationFactor->setEnabled(false);
        ui_->doubleSpinBoxVoxelSize->setEnabled(false);
        ui_->confirmPushButton->setEnabled(false);
        RCLCPP_INFO(node_->get_logger(), "Confirm button clicked.");
        if (!param_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(node_->get_logger(), "service not available");
            return;
        }

        // set parameters on parameter server
        param_client_->set_parameters_atomically({rclcpp::Parameter("voxel_size", voxel_size_)});
        RCLCPP_INFO(node_->get_logger(), "Parameters set:\voxel_size: %.2f", voxel_size_);


        if(!motion_gen_config_client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_INFO(node_->get_logger(), "service not available");
            return;
        }
            // Make the asynchronous (non-blocking) call.
        motion_gen_config_client_->async_send_request(motion_gen_config_request_,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response_future) {
          auto response = response_future.get();
          RCLCPP_INFO(node_->get_logger(), "Service call successful");
          ui_->doubleSpinBoxTimeDilationFactor->setEnabled(true);
          ui_->doubleSpinBoxVoxelSize->setEnabled(true);
          ui_->confirmPushButton->setEnabled(true);
        });

    }

    void RvizArgsPanel::on_sendTrajectory_clicked(){

      auto goal_request = curobo_msgs::action::SendTrajectory::Goal();
      auto send_goal_options = rclcpp_action::Client<curobo_msgs::action::SendTrajectory>::SendGoalOptions();

      send_goal_options.goal_response_callback = std::bind(&RvizArgsPanel::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback = std::bind(&RvizArgsPanel::result_callback, this, std::placeholders::_1);

      auto future_cancel = action_ptr_->async_send_goal(goal_request, send_goal_options);
      ui_->sendTrajectory->setEnabled(false);
      ui_->generateTrajectory->setEnabled(false);
      ui_->stopRobot->setEnabled(true);
      

    }

    void RvizArgsPanel::result_callback(const rclcpp_action::ClientGoalHandle<curobo_msgs::action::SendTrajectory>::WrappedResult & result){
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_ERROR(node_->get_logger(), "Goal was succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
          break;
      }
      ui_->sendTrajectory->setEnabled(true);
      ui_->generateTrajectory->setEnabled(true);
      ui_->stopRobot->setEnabled(false);
  }

  void RvizArgsPanel::goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<curobo_msgs::action::SendTrajectory>> goal_handle){
    this->goal_handle_ = goal_handle;
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

    void RvizArgsPanel::on_generateTrajectory_clicked(){
      if (!arrow_interaction_) {
        RCLCPP_WARN(node_->get_logger(), "Arrow marker not available yet. Please add ArrowInteractionDisplay to RViz.");
        return;
      }

      auto goal_request = std::make_shared<curobo_msgs::srv::TrajectoryGeneration::Request>();

      //TODO This is temporary, next step add and arrow in interface.
      auto pose = this->arrow_interaction_->get_pose();
      goal_request->target_pose = pose;

      
      auto result = this->trajectory_generation_client_->async_send_request(goal_request);
      if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Succeeded to call service");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call ");
      }
        

    }

    void RvizArgsPanel::on_generateAndSend_clicked(){
    }
    void RvizArgsPanel::on_stopRobot_clicked(){
      auto future_cancel = action_ptr_->async_cancel_goal(this->goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, future_cancel) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Succeeded to cancel goal");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to cancel goal");
      }
    }

    void RvizArgsPanel::findArrowInteractionDisplay(){
      // If already found, stop searching
      if (arrow_interaction_ != nullptr) {
        return;
      }

      // getDisplayContext() is the public method to access context
      auto context = getDisplayContext();
      if (!context) {
        return;
      }

      // Get the root display group directly from context
      auto root_display = context->getRootDisplayGroup();
      if (!root_display) {
        return;
      }

      // Search for ArrowInteractionDisplay
      for (int i = 0; i < root_display->numDisplays(); ++i) {
        auto display = root_display->getDisplayAt(i);
        if (display) {
          // Try to cast to ArrowInteractionDisplay
          auto arrow_display = dynamic_cast<ArrowInteractionDisplay*>(display);
          if (arrow_display) {
            // Found it!
            arrow_interaction_ = arrow_display->getArrowInteraction();
            if (arrow_interaction_) {
              RCLCPP_INFO(node_->get_logger(), "Found ArrowInteractionDisplay, using its marker");
            }
            return;
          }
        }
      }
    }

    void RvizArgsPanel::quaternionToEuler(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw) {
      // Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
      // Roll (x-axis rotation)
      double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
      double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
      roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

      // Pitch (y-axis rotation)
      double sinp = 2.0 * (q.w * q.y - q.z * q.x);
      if (std::abs(sinp) >= 1)
        pitch = std::copysign(90.0, sinp); // use 90 degrees if out of range
      else
        pitch = std::asin(sinp) * 180.0 / M_PI;

      // Yaw (z-axis rotation)
      double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    }

    void RvizArgsPanel::eulerToQuaternion(double roll, double pitch, double yaw, geometry_msgs::msg::Quaternion& q) {
      // Convert Euler angles (in degrees) to quaternion
      double roll_rad = roll * M_PI / 180.0;
      double pitch_rad = pitch * M_PI / 180.0;
      double yaw_rad = yaw * M_PI / 180.0;

      double cy = std::cos(yaw_rad * 0.5);
      double sy = std::sin(yaw_rad * 0.5);
      double cp = std::cos(pitch_rad * 0.5);
      double sp = std::sin(pitch_rad * 0.5);
      double cr = std::cos(roll_rad * 0.5);
      double sr = std::sin(roll_rad * 0.5);

      q.w = cr * cp * cy + sr * sp * sy;
      q.x = sr * cp * cy - cr * sp * sy;
      q.y = cr * sp * cy + sr * cp * sy;
      q.z = cr * cp * sy - sr * sp * cy;
    }

    void RvizArgsPanel::applyPoseFromSpinboxes(){
      if (!arrow_interaction_) {
        RCLCPP_WARN(node_->get_logger(), "ArrowInteraction not available yet");
        return;
      }

      // Get values from spinboxes
      geometry_msgs::msg::Pose pose;
      pose.position.x = ui_->spinBoxPosX->value();
      pose.position.y = ui_->spinBoxPosY->value();
      pose.position.z = ui_->spinBoxPosZ->value();

      // Get orientation from spinboxes and convert to quaternion
      double roll = ui_->spinBoxRoll->value();
      double pitch = ui_->spinBoxPitch->value();
      double yaw = ui_->spinBoxYaw->value();
      eulerToQuaternion(roll, pitch, yaw, pose.orientation);

      // Apply the new pose to the marker
      arrow_interaction_->setPoseWithOrientation(pose);

      // Update last displayed values to match what we just set
      last_displayed_x_ = pose.position.x;
      last_displayed_y_ = pose.position.y;
      last_displayed_z_ = pose.position.z;
      last_displayed_roll_ = roll;
      last_displayed_pitch_ = pitch;
      last_displayed_yaw_ = yaw;

      RCLCPP_INFO(node_->get_logger(), "Applied pose: X=%.3f, Y=%.3f, Z=%.3f, Roll=%.2f, Pitch=%.2f, Yaw=%.2f",
                  pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);
    }

    bool RvizArgsPanel::eventFilter(QObject *obj, QEvent *event) {
      // Check if the event is a FocusIn event on one of the pose spinboxes
      if (event->type() == QEvent::FocusIn) {
        if (obj == ui_->spinBoxPosX || obj == ui_->spinBoxPosY || obj == ui_->spinBoxPosZ ||
            obj == ui_->spinBoxRoll || obj == ui_->spinBoxPitch || obj == ui_->spinBoxYaw) {
          user_editing_pose_ = true;
        }
      }
      // Pass the event to the base class
      return QObject::eventFilter(obj, event);
    }

    void RvizArgsPanel::updateMarkerPoseDisplay(){
      // Don't update if marker not found yet or if user is editing
      if (!arrow_interaction_ || user_editing_pose_) {
        return;
      }

      auto pose = arrow_interaction_->get_pose();

      // Convert quaternion to Euler angles
      double roll, pitch, yaw;
      quaternionToEuler(pose.orientation, roll, pitch, yaw);

      // Compare with last displayed values - only update if changed
      constexpr double epsilon_pos = 1e-6; // Small threshold for position
      constexpr double epsilon_rot = 0.01; // Small threshold for rotation (degrees)

      bool x_changed = std::isnan(last_displayed_x_) || std::fabs(pose.position.x - last_displayed_x_) > epsilon_pos;
      bool y_changed = std::isnan(last_displayed_y_) || std::fabs(pose.position.y - last_displayed_y_) > epsilon_pos;
      bool z_changed = std::isnan(last_displayed_z_) || std::fabs(pose.position.z - last_displayed_z_) > epsilon_pos;
      bool roll_changed = std::isnan(last_displayed_roll_) || std::fabs(roll - last_displayed_roll_) > epsilon_rot;
      bool pitch_changed = std::isnan(last_displayed_pitch_) || std::fabs(pitch - last_displayed_pitch_) > epsilon_rot;
      bool yaw_changed = std::isnan(last_displayed_yaw_) || std::fabs(yaw - last_displayed_yaw_) > epsilon_rot;

      // Only update if at least one value has changed
      if (!x_changed && !y_changed && !z_changed && !roll_changed && !pitch_changed && !yaw_changed) {
        return;
      }

      // Update the spinboxes with current pose
      // Block signals to avoid triggering updates while we're setting values
      ui_->spinBoxPosX->blockSignals(true);
      ui_->spinBoxPosY->blockSignals(true);
      ui_->spinBoxPosZ->blockSignals(true);
      ui_->spinBoxRoll->blockSignals(true);
      ui_->spinBoxPitch->blockSignals(true);
      ui_->spinBoxYaw->blockSignals(true);

      if (x_changed) {
        ui_->spinBoxPosX->setValue(pose.position.x);
        last_displayed_x_ = pose.position.x;
      }
      if (y_changed) {
        ui_->spinBoxPosY->setValue(pose.position.y);
        last_displayed_y_ = pose.position.y;
      }
      if (z_changed) {
        ui_->spinBoxPosZ->setValue(pose.position.z);
        last_displayed_z_ = pose.position.z;
      }
      if (roll_changed) {
        ui_->spinBoxRoll->setValue(roll);
        last_displayed_roll_ = roll;
      }
      if (pitch_changed) {
        ui_->spinBoxPitch->setValue(pitch);
        last_displayed_pitch_ = pitch;
      }
      if (yaw_changed) {
        ui_->spinBoxYaw->setValue(yaw);
        last_displayed_yaw_ = yaw;
      }

      ui_->spinBoxPosX->blockSignals(false);
      ui_->spinBoxPosY->blockSignals(false);
      ui_->spinBoxPosZ->blockSignals(false);
      ui_->spinBoxRoll->blockSignals(false);
      ui_->spinBoxPitch->blockSignals(false);
      ui_->spinBoxYaw->blockSignals(false);
    }

    void RvizArgsPanel::on_pushButtonUpdateObstacles_clicked() {
      RCLCPP_INFO(node_->get_logger(), "Update Obstacles button clicked");

      // Check if service is available
      if (!get_voxel_grid_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "GetVoxelGrid service not available");
        return;
      }

      // Create request
      auto request = std::make_shared<curobo_msgs::srv::GetVoxelGrid::Request>();

      // Call service asynchronously
      auto future = get_voxel_grid_client_->async_send_request(request,
        [this](rclcpp::Client<curobo_msgs::srv::GetVoxelGrid>::SharedFuture future) {
          try {
            auto response = future.get();

            // Create marker from voxel grid
            auto marker = std::make_shared<visualization_msgs::msg::Marker>();
            // Get base_link parameter from node, default to "base_link"
            std::string base_link = "base_link";
            if (node_->has_parameter("base_link")) {
              base_link = node_->get_parameter("base_link").as_string();
            }
            marker->header.frame_id = base_link;
            marker->header.stamp = node_->get_clock()->now();
            marker->ns = "voxel_grid";
            marker->id = 0;
            marker->type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker->action = visualization_msgs::msg::Marker::ADD;

            auto& voxel_grid = response->voxel_grid;
            marker->scale.x = voxel_grid.resolutions.x;
            marker->scale.y = voxel_grid.resolutions.y;
            marker->scale.z = voxel_grid.resolutions.z;

            // Set marker color (green)
            marker->color.r = 0.0;
            marker->color.g = 1.0;
            marker->color.b = 0.0;
            marker->color.a = 1.0;

            // Convert voxel grid data into cubes
            size_t index = 0;
            for (size_t x = 0; x < voxel_grid.size_z; x++) {
              for (size_t y = 0; y < voxel_grid.size_y; y++) {
                for (size_t z = 0; z < voxel_grid.size_x; z++) {
                  if (voxel_grid.data[index] > 0) {
                    geometry_msgs::msg::Point point;
                    point.x = voxel_grid.origin.x + x * voxel_grid.resolutions.x;
                    point.y = voxel_grid.origin.y + y * voxel_grid.resolutions.y;
                    point.z = voxel_grid.origin.z + z * voxel_grid.resolutions.z;
                    marker->points.push_back(point);
                  }
                  index++;
                }
              }
            }

            // Publish marker
            voxel_marker_pub_->publish(*marker);
            RCLCPP_INFO(node_->get_logger(), "Published voxel grid visualization with %zu occupied voxels", marker->points.size());

          } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get voxel grid: %s", e.what());
          }
        });
    }

    void RvizArgsPanel::updateObstacleFrequency(double value) {
      obstacle_update_frequency_ = value;

      // Stop timer if frequency is 0
      if (obstacle_update_frequency_ <= 0.0) {
        obstacle_update_timer_->stop();
        ui_->labelUpdateStatus->setText("Off");
        RCLCPP_INFO(node_->get_logger(), "Obstacle auto-update disabled");
      } else {
        // Convert Hz to milliseconds
        int interval_ms = static_cast<int>(1000.0 / obstacle_update_frequency_);
        obstacle_update_timer_->start(interval_ms);
        ui_->labelUpdateStatus->setText(QString("%.1f Hz").arg(obstacle_update_frequency_));
        RCLCPP_INFO(node_->get_logger(), "Obstacle auto-update set to %.1f Hz (every %d ms)",
                    obstacle_update_frequency_, interval_ms);
      }
    }

    void RvizArgsPanel::updateObstaclesFromTimer() {
      // Call the same method as the button
      on_pushButtonUpdateObstacles_clicked();
    }

    void RvizArgsPanel::on_comboBoxRobotStrategy_currentTextChanged(const QString &text) {
      RCLCPP_INFO(node_->get_logger(), "Robot strategy changed to: %s", text.toStdString().c_str());

      std::string new_strategy = text.toStdString();

      // Update status label
      ui_->labelStrategyStatus->setText("Switching...");
      ui_->labelStrategyStatus->setStyleSheet("QLabel { color : orange; }");

      // Set the robot_type parameter
      if (!param_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "Parameter service not available");
        ui_->labelStrategyStatus->setText("Service unavailable");
        ui_->labelStrategyStatus->setStyleSheet("QLabel { color : red; }");
        return;
      }

      try {
        // Set the robot_type parameter
        param_client_->set_parameters({rclcpp::Parameter("robot_type", new_strategy)});
        RCLCPP_INFO(node_->get_logger(), "Set robot_type parameter to: %s", new_strategy.c_str());

        // Call the set_robot_strategy service
        if (!set_robot_strategy_client_->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_WARN(node_->get_logger(), "SetRobotStrategy service not available");
          ui_->labelStrategyStatus->setText("Service unavailable");
          ui_->labelStrategyStatus->setStyleSheet("QLabel { color : red; }");
          return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        auto future = set_robot_strategy_client_->async_send_request(request,
          [this, new_strategy](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            try {
              auto response = future.get();

              if (response->success) {
                current_robot_strategy_ = new_strategy;
                ui_->labelStrategyStatus->setText(QString::fromStdString(new_strategy));
                ui_->labelStrategyStatus->setStyleSheet("QLabel { color : green; }");
                RCLCPP_INFO(node_->get_logger(), "Successfully switched to strategy: %s", new_strategy.c_str());
                RCLCPP_INFO(node_->get_logger(), "Service response: %s", response->message.c_str());
              } else {
                ui_->labelStrategyStatus->setText("Failed");
                ui_->labelStrategyStatus->setStyleSheet("QLabel { color : red; }");
                RCLCPP_ERROR(node_->get_logger(), "Failed to switch strategy: %s", response->message.c_str());
              }

            } catch (const std::exception& e) {
              ui_->labelStrategyStatus->setText("Error");
              ui_->labelStrategyStatus->setStyleSheet("QLabel { color : red; }");
              RCLCPP_ERROR(node_->get_logger(), "Exception calling set_robot_strategy service: %s", e.what());
            }
          });

      } catch (const std::exception& e) {
        ui_->labelStrategyStatus->setText("Error");
        ui_->labelStrategyStatus->setStyleSheet("QLabel { color : red; }");
        RCLCPP_ERROR(node_->get_logger(), "Exception setting parameter: %s", e.what());
      }
    }


} // curobo_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_rviz::RvizArgsPanel, rviz_common::Panel)