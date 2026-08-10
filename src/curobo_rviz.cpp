#include "curobo_rviz/curobo_rviz.hpp"
#include <cmath>

namespace curobo_rviz
{
  RvizArgsPanel::RvizArgsPanel(QWidget *parent)
    : Panel{parent}
    , ui_(std::make_unique<Ui::gui_parameters>())
    , node_{nullptr}
    , param_client_{nullptr}
    , planner_ready_{false}
    , planner_poll_in_flight_{false}
    , goal_active_{false}
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

    // Declare base_link parameter with default value
    node_->declare_parameter<std::string>("base_link", "base_0");

    // Target planner node name is configurable (was hard-coded to "unified_planner").
    // On the leeloo system the planner node is "curobo_trajectory_planner"; override
    // with the "planner_node_name" parameter on this panel's node
    // (/rviz_updata_parameters_node) if it differs.
    node_->declare_parameter<std::string>("planner_node_name", "curobo_trajectory_planner");
    const std::string planner_node = node_->get_parameter("planner_node_name").as_string();
    const std::string planner_ns = "/" + planner_node + "/";

    // Try to find ArrowInteractionDisplay, will be set by timer if not immediately available
    this->arrow_interaction_ = nullptr;

    // AsyncParametersClient -- never SyncParametersClient, which builds its own
    // temporary executor per call and would fight the background NodeSpinner (see
    // spinner_ below) for ownership of node_.
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, planner_node);

    motion_gen_config_client_ = node_->create_client<std_srvs::srv::Trigger>(planner_ns + "update_motion_gen_config");
    motion_gen_config_request_ = std::make_shared<std_srvs::srv::Trigger::Request>();

    // action client
    this->action_ptr_ = rclcpp_action::create_client<curobo_msgs::action::SendTrajectory>(
      node_,
      planner_ns + "execute_trajectory");

    // create service client to generate traj
    this->trajectory_generation_client_ = node_->create_client<curobo_msgs::srv::TrajectoryGeneration>(planner_ns + "generate_trajectory");

    // Create service client for getting voxel grid
    this->get_voxel_grid_client_ = node_->create_client<curobo_msgs::srv::GetVoxelGrid>(planner_ns + "get_voxel_grid");

    // Create publisher for voxel grid visualization
    this->voxel_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualise_voxel_grid", 10);

    // Frame the voxel grid markers are expressed in. The planner publishes voxel grids
    // in dsr01/world (as seen in /curobo_trajectory_planner/voxel_grid_sparse), so use
    // that as the default. Can be overridden via the "voxel_frame_id" ROS parameter.
    node_->declare_parameter<std::string>("voxel_frame_id", "dsr01/world");
    voxel_frame_id_ = node_->get_parameter("voxel_frame_id").as_string();

    // MarkerArray publisher for the voxel grid, latched (transient_local) so an
    // RViz MarkerArray display shows the last grid even if it connects afterwards.
    this->voxel_marker_array_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualise_voxel_grid_array", rclcpp::QoS(1).transient_local());

    // Subscribe to the planner's sparse voxel grid stream for the auto-refresh path
    // (see updateObstaclesFromTimer). Namespaced under planner_ns like the service
    // clients above, so it follows the same planner_node_name override.
    this->voxel_grid_sparse_sub_ = node_->create_subscription<curobo_msgs::msg::SparseVoxelGrid>(
        planner_ns + "voxel_grid_sparse", rclcpp::QoS(10),
        [this](curobo_msgs::msg::SparseVoxelGrid::ConstSharedPtr msg) { onSparseVoxelGrid(msg); });

    // Create timer for automatic obstacle updates
    obstacle_update_timer_ = new QTimer(this);
    connect(obstacle_update_timer_, &QTimer::timeout, this, &RvizArgsPanel::updateObstaclesFromTimer);

    // Create service client for setting robot strategy
    this->set_robot_strategy_client_ = node_->create_client<curobo_msgs::srv::SetRobotStrategy>(planner_ns + "set_robot_strategy");
    current_robot_strategy_ = "";

    // Create service client for setting planner type
    this->set_planner_client_ = node_->create_client<curobo_msgs::srv::SetPlanner>(planner_ns + "set_planner");
    current_planner_type_ = 0; // Default to CLASSIC

    // Connect SpinBox and DoubleSpinBox to slots
    connect(ui_->doubleSpinBoxTimeDilationFactor, SIGNAL(valueChanged(double)), this, SLOT(updateTimeDilationFactor(double)));
    connect(ui_->doubleSpinBoxVoxelSize, SIGNAL(valueChanged(double)), this, SLOT(updateVoxelSize(double)));
    
    // Every planner-facing widget starts disabled and stays that way until
    // pollPlannerReady() confirms the planner is actually responding -- not just
    // discoverable (see setPlannerReady()/pollPlannerReady() for why that distinction
    // matters during the planner's ~90s GPU warmup).
    setPlannerReady(false);

    QTimer* readinessTimer = new QTimer(this);
    connect(readinessTimer, &QTimer::timeout, this, &RvizArgsPanel::pollPlannerReady);
    readinessTimer->start(250);

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

    // setupUi() applied the .ui file's default frequency before the connect above
    // existed, so valueChanged never fired for it and the timer would sit stopped
    // until the user nudged the spinbox. Apply the current value explicitly to arm
    // it. Safe to start here: QTimer fires on the GUI event loop, which is not
    // running yet during this constructor.
    updateObstacleFrequency(ui_->spinBoxUpdateFrequency->value());

    // Connect robot strategy controls
    connect(ui_->comboBoxRobotStrategy, &QComboBox::currentTextChanged, this, &RvizArgsPanel::on_comboBoxRobotStrategy_currentTextChanged);

    // Connect planner type controls
    connect(ui_->comboBoxTrajectoryType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &RvizArgsPanel::on_comboBoxTrajectoryType_currentIndexChanged);

    // Connect stop robot button
    connect(ui_->stopRobot, &QPushButton::clicked, this, &RvizArgsPanel::on_stopRobot_clicked);

    // Create MPC goal publisher for real-time tracking
    this->mpc_goal_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/unified_planner/mpc_goal", 10);

    // Create timer for MPC goal publishing (10Hz), but don't start it yet
    mpc_goal_publisher_timer_ = new QTimer(this);
    connect(mpc_goal_publisher_timer_, &QTimer::timeout, this, &RvizArgsPanel::publishMpcGoal);
    is_mpc_tracking_active_ = false;

    // Spin node_ on a background thread for the rest of this panel's lifetime, so
    // every async_send_request/AsyncParametersClient callback below actually gets
    // delivered. Constructed last: by now every client/publisher/subscription this
    // node will ever need already exists.
    spinner_ = std::make_unique<NodeSpinner>(node_);
  }

  RvizArgsPanel::~RvizArgsPanel()
  {
    // spinner_ is declared last in the header, so it is destroyed FIRST here --
    // cancel()+join() completes (see NodeSpinner) before any client/publisher/node
    // it might still be invoking callbacks against is torn down.
  }

  void RvizArgsPanel::runOnGuiThread(std::function<void()> fn)
  {
    QMetaObject::invokeMethod(this, std::move(fn), Qt::QueuedConnection);
  }

  void RvizArgsPanel::setPlannerReady(bool ready)
  {
    if (planner_ready_ == ready) {
      return;
    }
    planner_ready_ = ready;

    ui_->doubleSpinBoxTimeDilationFactor->setEnabled(ready);
    ui_->doubleSpinBoxVoxelSize->setEnabled(ready);
    ui_->confirmPushButton->setEnabled(ready);
    ui_->comboBoxRobotStrategy->setEnabled(ready);
    ui_->comboBoxTrajectoryType->setEnabled(ready);
    // pushButtonUpdateObstacles calls the GetVoxelGrid *service*, so it stays gated.
    // spinBoxUpdateFrequency drives updateObstaclesFromTimer(), which only re-renders
    // the cached voxel_grid_sparse topic and makes no request of the planner -- it
    // stays live so obstacles remain visible during the planner's GPU warmup, when
    // node_is_available is still false.
    ui_->pushButtonUpdateObstacles->setEnabled(ready);
    updateActionButtons();

    RCLCPP_INFO(node_->get_logger(), "Planner is %s", ready ? "ready" : "not ready");
  }

  void RvizArgsPanel::updateActionButtons()
  {
    const bool can_start = planner_ready_ && !goal_active_;
    ui_->generateTrajectory->setEnabled(can_start);
    ui_->sendTrajectory->setEnabled(can_start);
    ui_->generateAndSend->setEnabled(can_start);
    ui_->stopRobot->setEnabled(goal_active_);
  }

  void RvizArgsPanel::pollPlannerReady()
  {
    if (planner_poll_in_flight_) {
      return;
    }
    if (!param_client_->service_is_ready()) {
      setPlannerReady(false);
      return;
    }

    planner_poll_in_flight_ = true;
    param_client_->get_parameters({"node_is_available"},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        bool ready = false;
        try {
          auto params = future.get();
          ready = !params.empty() && params[0].as_bool();
        } catch (const std::exception & e) {
          RCLCPP_WARN(node_->get_logger(), "node_is_available check failed: %s", e.what());
        }
        runOnGuiThread([this, ready]() {
          planner_poll_in_flight_ = false;
          setPlannerReady(ready);
        });
      });
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

        // This slot fires during RViz config load (spinbox setValue -> valueChanged),
        // so it must never block: set_parameters_atomically is called with a callback
        // (AsyncParametersClient), never the SyncParametersClient/blocking form.
        if (!param_client_->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(),
                "Planner parameter service not available; skipping time_dilation_factor update");
            return;
        }
        param_client_->set_parameters_atomically(
          {rclcpp::Parameter("time_dilation_factor", time_dilation_factor_)},
          [this, value](std::shared_future<rcl_interfaces::msg::SetParametersResult> future) {
            try {
              auto result = future.get();
              if (result.successful) {
                RCLCPP_INFO(node_->get_logger(), "Time dilation factor set to %.2f", value);
              } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set time_dilation_factor: %s",
                    result.reason.c_str());
              }
            } catch (const std::exception & e) {
              RCLCPP_ERROR(node_->get_logger(), "Exception setting time_dilation_factor: %s", e.what());
            }
          });
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
        if (!param_client_->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Planner parameter service not available");
            setPlannerReady(false); // already on the GUI thread (button click slot)
            return;
        }

        param_client_->set_parameters_atomically(
          {rclcpp::Parameter("voxel_size", voxel_size_)},
          [this](std::shared_future<rcl_interfaces::msg::SetParametersResult> future) {
            try {
              auto result = future.get();
              if (result.successful) {
                RCLCPP_INFO(node_->get_logger(), "Parameters set: voxel_size: %.2f", voxel_size_);
              } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set voxel_size: %s", result.reason.c_str());
              }
            } catch (const std::exception & e) {
              RCLCPP_ERROR(node_->get_logger(), "Exception setting voxel_size: %s", e.what());
            }

            if (!motion_gen_config_client_->service_is_ready()) {
              RCLCPP_WARN(node_->get_logger(), "update_motion_gen_config service not available");
              runOnGuiThread([this]() {
                // Re-sync widget state with whatever planner_ready_ actually is
                // rather than blindly re-enabling.
                ui_->doubleSpinBoxTimeDilationFactor->setEnabled(planner_ready_);
                ui_->doubleSpinBoxVoxelSize->setEnabled(planner_ready_);
                ui_->confirmPushButton->setEnabled(planner_ready_);
              });
              return;
            }

            motion_gen_config_client_->async_send_request(motion_gen_config_request_,
              [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response_future) {
                try {
                  auto response = response_future.get();
                  RCLCPP_INFO(node_->get_logger(), "Service call successful: %s", response->message.c_str());
                } catch (const std::exception & e) {
                  RCLCPP_ERROR(node_->get_logger(), "update_motion_gen_config call failed: %s", e.what());
                }
                runOnGuiThread([this]() {
                  ui_->doubleSpinBoxTimeDilationFactor->setEnabled(planner_ready_);
                  ui_->doubleSpinBoxVoxelSize->setEnabled(planner_ready_);
                  ui_->confirmPushButton->setEnabled(planner_ready_);
                });
              });
          });
    }

    void RvizArgsPanel::on_sendTrajectory_clicked(){
      auto goal_request = curobo_msgs::action::SendTrajectory::Goal();

      // Publish marker pose ONCE to MPC goal topic
      if (arrow_interaction_) {
        auto marker_pose = arrow_interaction_->get_pose();
        mpc_goal_pub_->publish(marker_pose);
        RCLCPP_INFO(node_->get_logger(), "Published goal pose once for execution");

        // REQUIRED for the open-loop path, and it used to be missing: the goal
        // went out default-constructed, i.e. target_pose = (0,0,0) with identity
        // orientation -- the dsr01/world origin, inside the robot's own base.
        //
        // The panel relied on the server reusing the plan the "generate" button
        // had just cached (allow_cached defaults to true). But that reuse is
        // gated by _pending_plan_matches(), whose signature INCLUDES target_pose
        // and compares positions to 1 mm (unified_planner_node.py
        // _target_signature/_poses_match). Cached signature = the real marker
        // pose, incoming goal = the origin -> guaranteed mismatch, so the server
        // silently re-planned toward the robot's own base, burned its 10
        // max_attempts, failed, and aborted the goal without logging anything.
        // Net effect: "generate" returned a trajectory, "execute" did nothing.
        //
        // MPC never showed this because the reactive path gets its target from
        // mpc_goal_pub_ (republished at 10 Hz by mpc_goal_publisher_timer_),
        // which overrides the empty target_pose. The open-loop path has no such
        // second source -- the cache was its only route, and it was unreachable.
        goal_request.target_pose = marker_pose;
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "Arrow marker not available - goal sent WITHOUT a target pose "
                    "(open-loop planning will fail; add ArrowInteractionDisplay to RViz)");
      }

      auto send_goal_options = rclcpp_action::Client<curobo_msgs::action::SendTrajectory>::SendGoalOptions();

      send_goal_options.goal_response_callback = std::bind(&RvizArgsPanel::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback = std::bind(&RvizArgsPanel::result_callback, this, std::placeholders::_1);

      action_ptr_->async_send_goal(goal_request, send_goal_options);
      goal_active_ = true;
      updateActionButtons();
    }

    // goal_response_callback/result_callback are rclcpp_action client callbacks --
    // they now fire on the background spin thread (spinner_), so every UI/member
    // touch inside them is marshaled back via runOnGuiThread. Only pure logging
    // (thread-safe) stays outside.
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

      runOnGuiThread([this]() {
        if (is_mpc_tracking_active_) {
          mpc_goal_publisher_timer_->stop();
          is_mpc_tracking_active_ = false;
          RCLCPP_INFO(node_->get_logger(), "Stopped MPC tracking mode");
        }
        goal_active_ = false;
        updateActionButtons();
      });
  }

  void RvizArgsPanel::goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<curobo_msgs::action::SendTrajectory>> goal_handle){
    const bool accepted = static_cast<bool>(goal_handle);
    if (!accepted) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
    runOnGuiThread([this, goal_handle]() {
      this->goal_handle_ = goal_handle;
    });
  }

    void RvizArgsPanel::on_generateTrajectory_clicked(){
      generateTrajectoryAsync(nullptr);
    }

    // Shared by on_generateTrajectory_clicked (fire-and-forget) and the classic-mode
    // branch of on_generateAndSend_clicked (chains into on_sendTrajectory_clicked).
    // Pure async_send_request + callback -- no spin_until_future_complete, so this
    // can never block the GUI thread waiting on the planner.
    void RvizArgsPanel::generateTrajectoryAsync(std::function<void(bool)> on_done){
      // Always called from the GUI thread (button slots), so early-return failure
      // paths can invoke on_done() directly; only the async completion below
      // (which fires on the background spin thread) needs runOnGuiThread.
      if (!arrow_interaction_) {
        RCLCPP_WARN(node_->get_logger(), "Arrow marker not available yet. Please add ArrowInteractionDisplay to RViz.");
        if (on_done) { on_done(false); }
        return;
      }
      if (!trajectory_generation_client_->service_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "generate_trajectory service not available");
        if (on_done) { on_done(false); }
        return;
      }

      auto goal_request = std::make_shared<curobo_msgs::srv::TrajectoryGeneration::Request>();
      goal_request->target_pose = this->arrow_interaction_->get_pose();

      trajectory_generation_client_->async_send_request(goal_request,
        [this, on_done](rclcpp::Client<curobo_msgs::srv::TrajectoryGeneration>::SharedFuture future) {
          bool success = false;
          std::string message;
          try {
            auto response = future.get();
            success = response->success;
            message = response->message;
          } catch (const std::exception & e) {
            message = e.what();
          }
          if (success) {
            RCLCPP_INFO(node_->get_logger(), "generate_trajectory succeeded");
          } else {
            RCLCPP_ERROR(node_->get_logger(), "generate_trajectory failed: %s", message.c_str());
          }
          if (on_done) {
            runOnGuiThread([on_done, success]() { on_done(success); });
          }
        });
    }

    void RvizArgsPanel::on_generateAndSend_clicked(){
      if (!arrow_interaction_) {
        RCLCPP_WARN(node_->get_logger(), "Arrow marker not available");
        return;
      }

      // Check if MPC planner is selected (planner_type == 1)
      if (current_planner_type_ == 1) {
        // MPC Mode: Start continuous tracking
        RCLCPP_INFO(node_->get_logger(), "Starting MPC tracking mode (10Hz goal updates)");

        if (!trajectory_generation_client_->service_is_ready()) {
          RCLCPP_ERROR(node_->get_logger(), "generate_trajectory service not available");
          return;
        }

        auto gen_request = std::make_shared<curobo_msgs::srv::TrajectoryGeneration::Request>();
        gen_request->target_pose = arrow_interaction_->get_pose();

        trajectory_generation_client_->async_send_request(gen_request,
          [this](rclcpp::Client<curobo_msgs::srv::TrajectoryGeneration>::SharedFuture future) {
            bool success = false;
            std::string message;
            try {
              auto response = future.get();
              success = response->success;
              message = response->message;
            } catch (const std::exception & e) {
              message = e.what();
            }

            // Everything below touches ui_/QTimer/goal_active_ -- hop back to the
            // GUI thread as one block rather than cherry-picking individual lines.
            runOnGuiThread([this, success, message]() {
              if (success) {
                RCLCPP_INFO(node_->get_logger(), "MPC initialized successfully");

                is_mpc_tracking_active_ = true;
                mpc_goal_publisher_timer_->start(100); // 100ms = 10Hz
                publishMpcGoal(); // Publish first goal immediately

                auto goal_request = curobo_msgs::action::SendTrajectory::Goal();
                auto send_goal_options = rclcpp_action::Client<curobo_msgs::action::SendTrajectory>::SendGoalOptions();
                send_goal_options.goal_response_callback = std::bind(&RvizArgsPanel::goal_response_callback, this, std::placeholders::_1);
                send_goal_options.result_callback = std::bind(&RvizArgsPanel::result_callback, this, std::placeholders::_1);
                action_ptr_->async_send_goal(goal_request, send_goal_options);

                goal_active_ = true;
                updateActionButtons();
              } else {
                RCLCPP_ERROR(node_->get_logger(), "MPC initialization failed: %s", message.c_str());
              }
            });
          });

      } else {
        // Classic Mode: generate, then chain into send once generation actually
        // succeeds (previously a QTimer::singleShot(500, ...) guess resting on
        // on_generateTrajectory_clicked's old blocking behavior -- now a real
        // completion callback, since that call is fully async).
        RCLCPP_INFO(node_->get_logger(), "Classic mode: generate and execute once");
        generateTrajectoryAsync([this](bool success) {
          if (success) {
            on_sendTrajectory_clicked();
          } else {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory generation failed; not sending");
          }
        });
      }
    }
    void RvizArgsPanel::on_stopRobot_clicked(){
      // Stop MPC tracking timer if active
      if (is_mpc_tracking_active_) {
        mpc_goal_publisher_timer_->stop();
        is_mpc_tracking_active_ = false;
        RCLCPP_INFO(node_->get_logger(), "Stopped MPC tracking mode");
      }

      // Cancel the action goal if it exists and is still active
      if (goal_handle_) {
        try {
          // Use async_cancel_goal without blocking spin
          // The result_callback will handle the cleanup
          action_ptr_->async_cancel_goal(goal_handle_);
          RCLCPP_INFO(node_->get_logger(), "Cancel request sent");
        } catch (const std::exception& e) {
          RCLCPP_WARN(node_->get_logger(), "Exception during cancel: %s", e.what());
        }
      }

      // Re-enable buttons immediately (don't wait for result)
      goal_active_ = false;
      updateActionButtons();
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

      // Check if service is available (pure graph-discovery check, no spin needed
      // and no blocking wait -- see pollPlannerReady()/NodeSpinner for why this
      // matters).
      if (!get_voxel_grid_client_->service_is_ready()) {
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
            auto& voxel_grid = response->voxel_grid;

            // Build a MarkerArray with a single CUBE_LIST holding one cube per
            // occupied voxel (efficient for the thousands a dense grid returns).
            // A fixed ns+id means each new publish REPLACES the previous grid in
            // place, so no DELETEALL is needed (and adding one would collide on the
            // same (ns, id) and trip RViz's duplicate-marker check).
            visualization_msgs::msg::MarkerArray marker_array;
            const auto stamp = node_->get_clock()->now();
            // Prefer the frame the service provides; fall back to the configured one.
            std::string frame_id = voxel_grid.header.frame_id.empty()
                                       ? voxel_frame_id_
                                       : voxel_grid.header.frame_id;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = stamp;
            marker.ns = "voxel_grid";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = voxel_grid.resolutions.x;
            marker.scale.y = voxel_grid.resolutions.y;
            marker.scale.z = voxel_grid.resolutions.z;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // Dense C-order grid (linear = i*size_y*size_z + j*size_z + k).
            size_t index = 0;
            for (size_t i = 0; i < voxel_grid.size_x; i++) {
              for (size_t j = 0; j < voxel_grid.size_y; j++) {
                for (size_t k = 0; k < voxel_grid.size_z; k++) {
                  if (index < voxel_grid.data.size() && voxel_grid.data[index] > 0) {
                    geometry_msgs::msg::Point point;
                    // Cube centre = origin + (idx + 0.5) * resolution.
                    point.x = voxel_grid.origin.x + (i + 0.5) * voxel_grid.resolutions.x;
                    point.y = voxel_grid.origin.y + (j + 0.5) * voxel_grid.resolutions.y;
                    point.z = voxel_grid.origin.z + (k + 0.5) * voxel_grid.resolutions.z;
                    marker.points.push_back(point);
                  }
                  index++;
                }
              }
            }
            marker_array.markers.push_back(marker);

            // Publish MarkerArray (latched).
            voxel_marker_array_pub_->publish(marker_array);
            RCLCPP_INFO(node_->get_logger(),
                "Published voxel grid MarkerArray in frame '%s' with %zu occupied voxels",
                frame_id.c_str(), marker.points.size());

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
        ui_->labelUpdateStatus->setText(QString("%1 Hz").arg(obstacle_update_frequency_, 0, 'f', 1));
        RCLCPP_INFO(node_->get_logger(), "Obstacle auto-update set to %.1f Hz (every %d ms)",
                    obstacle_update_frequency_, interval_ms);
      }
    }

    void RvizArgsPanel::updateObstaclesFromTimer() {
      // Deliberately NOT gated on planner_ready_. This path makes no request of
      // the planner: it renders from whatever the sparse-topic subscription has
      // cached (see voxel_grid_sparse_sub_) rather than round-tripping the
      // GetVoxelGrid service on every tick. The planner publishes voxel_grid_sparse
      // throughout its ~90s GPU warmup, while node_is_available is still false --
      // which is exactly when seeing the obstacle grid is most useful. Only the
      // service-calling paths (pushButtonUpdateObstacles) stay gated.
      //
      // The topic is server-paced by the planner (~7Hz); if the user's requested
      // frequency exceeds that, we simply re-publish the same cached message more
      // often than new data actually arrives.
      curobo_msgs::msg::SparseVoxelGrid::ConstSharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(latest_sparse_mutex_);
        msg = latest_sparse_msg_;
      }
      if (!msg) {
        RCLCPP_WARN_ONCE(node_->get_logger(),
            "Auto-refresh timer fired but no voxel_grid_sparse message received yet");
        return;
      }
      publishMarkerFromSparse(*msg);
    }

    void RvizArgsPanel::onSparseVoxelGrid(curobo_msgs::msg::SparseVoxelGrid::ConstSharedPtr msg) {
      // Called on the background spin thread (spinner_). Just cache -- the GUI
      // thread (updateObstaclesFromTimer) decides when to actually render.
      std::lock_guard<std::mutex> lock(latest_sparse_mutex_);
      latest_sparse_msg_ = msg;
    }

    void RvizArgsPanel::publishMarkerFromSparse(const curobo_msgs::msg::SparseVoxelGrid & msg) {
      const uint32_t sy = msg.size_y;
      const uint32_t sz = msg.size_z;
      const uint32_t syz = sy * sz;
      const float vs = msg.resolution;
      const float half = vs / 2.0f;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg.header.frame_id.empty() ? voxel_frame_id_ : msg.header.frame_id;
      marker.header.stamp = node_->get_clock()->now();
      marker.ns = "voxel_grid";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = vs;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.points.reserve(msg.occupied_indices.size());

      for (int32_t linear : msg.occupied_indices) {
        const uint32_t u = static_cast<uint32_t>(linear);
        const uint32_t gx = u / syz;
        const uint32_t rem = u % syz;
        const uint32_t gy = rem / sz;
        const uint32_t gz = rem % sz;

        geometry_msgs::msg::Point point;
        point.x = msg.origin.x + gx * vs + half;
        point.y = msg.origin.y + gy * vs + half;
        point.z = msg.origin.z + gz * vs + half;
        marker.points.push_back(point);
      }

      visualization_msgs::msg::MarkerArray marker_array;
      marker_array.markers.push_back(marker);
      voxel_marker_array_pub_->publish(marker_array);
    }

    void RvizArgsPanel::on_comboBoxRobotStrategy_currentTextChanged(const QString &text) {
      RCLCPP_INFO(node_->get_logger(), "Control strategy changed to: %s", text.toStdString().c_str());

      std::string new_strategy = text.toStdString();

      try {
        // Call the set_robot_strategy service with the control-strategy KEY
        // (emulator / joint_speed / joint_pose). The service also updates the
        // node's `control_strategy` parameter, so no separate parameter set is
        // needed here (the old `robot_type` parameter no longer exists).
        if (!set_robot_strategy_client_->service_is_ready()) {
          RCLCPP_WARN(node_->get_logger(), "SetRobotStrategy service not available");
          return;
        }

        auto request = std::make_shared<curobo_msgs::srv::SetRobotStrategy::Request>();
        request->robot_strategy = new_strategy;

        set_robot_strategy_client_->async_send_request(request,
          [this, new_strategy](rclcpp::Client<curobo_msgs::srv::SetRobotStrategy>::SharedFuture future) {
            try {
              auto response = future.get();

              if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "Successfully switched to strategy: %s", new_strategy.c_str());
                RCLCPP_INFO(node_->get_logger(), "Service response: %s", response->message.c_str());
                runOnGuiThread([this, new_strategy]() { current_robot_strategy_ = new_strategy; });
              } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to switch strategy: %s", response->message.c_str());
              }

            } catch (const std::exception& e) {
              RCLCPP_ERROR(node_->get_logger(), "Exception calling set_robot_strategy service: %s", e.what());
            }
          });

      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception calling set_robot_strategy: %s", e.what());
      }
    }

    void RvizArgsPanel::on_comboBoxTrajectoryType_currentIndexChanged(int index) {
      RCLCPP_INFO(node_->get_logger(), "Planner type changed to index: %d", index);

      // Map index to planner type constant
      uint8_t planner_type = static_cast<uint8_t>(index);

      // Verify planner type is valid
      if (planner_type > 3) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid planner type: %d", planner_type);
        return;
      }

      // Map planner type to name for logging
      std::string planner_name;
      switch (planner_type) {
        case 0:
          planner_name = "CLASSIC";
          break;
        case 1:
          planner_name = "MPC";
          break;
        case 2:
          planner_name = "BATCH";
          break;
        case 3:
          planner_name = "CONSTRAINED";
          break;
      }

      RCLCPP_INFO(node_->get_logger(), "Switching to planner: %s", planner_name.c_str());

      // Call the set_planner service
      if (!set_planner_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "SetPlanner service not available");
        return;
      }

      auto request = std::make_shared<curobo_msgs::srv::SetPlanner::Request>();
      request->planner_type = planner_type;

      set_planner_client_->async_send_request(request,
        [this, planner_type, planner_name](rclcpp::Client<curobo_msgs::srv::SetPlanner>::SharedFuture future) {
          try {
            auto response = future.get();

            if (response->success) {
              RCLCPP_INFO(node_->get_logger(), "Successfully switched to planner: %s", planner_name.c_str());
              RCLCPP_INFO(node_->get_logger(), "Service response: %s", response->message.c_str());
              RCLCPP_INFO(node_->get_logger(), "Previous planner: %s, Current planner: %s",
                          response->previous_planner.c_str(), response->current_planner.c_str());
              runOnGuiThread([this, planner_type]() { current_planner_type_ = planner_type; });
            } else {
              RCLCPP_ERROR(node_->get_logger(), "Failed to switch planner: %s", response->message.c_str());
            }

          } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception calling set_planner service: %s", e.what());
          }
        });
    }

    void RvizArgsPanel::publishMpcGoal() {
      if (!is_mpc_tracking_active_ || !arrow_interaction_) {
        return;
      }

      // Get current marker pose and publish to MPC goal topic
      auto marker_pose = arrow_interaction_->get_pose();
      mpc_goal_pub_->publish(marker_pose);

      // Debug log (can be verbose, use sparingly)
      // RCLCPP_DEBUG(node_->get_logger(), "Published MPC goal at 10Hz");
    }


} // curobo_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_rviz::RvizArgsPanel, rviz_common::Panel)