#include <curobo_rviz/add_objects_panel.hpp>
#include <string>

namespace add_objects_panel
{
    AddObjectsPanel::AddObjectsPanel(QWidget *parent)
        : Panel{parent}
        , ui_{std::make_unique<Ui::gui_objects>()}
        , node_{nullptr}
        , add_object_client_{nullptr}
        , add_object_request_ {nullptr}
        , remove_object_client_{nullptr}
        , remove_object_request_{nullptr}
        , add_object_publisher_{nullptr}
        , remove_object_publisher_{nullptr}
        , timerMessage_{nullptr}
        , param_client_{nullptr}
        , planner_ready_{false}
        , planner_poll_in_flight_{false}
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_add_objects_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        // Target planner node name is configurable (was hard-coded to "unified_planner").
        // Defaults to "curobo_trajectory_planner" (the leeloo planner node).
        node_->declare_parameter<std::string>("planner_node_name", "curobo_trajectory_planner");
        const std::string planner_node = node_->get_parameter("planner_node_name").as_string();
        const std::string planner_ns = "/" + planner_node + "/";

        // create add_objects & remove_objects service
        add_object_client_ = node_->create_client<curobo_msgs::srv::AddObject>(planner_ns + "add_object");
        add_object_request_ = std::make_shared<curobo_msgs::srv::AddObject_Request>();
        remove_object_client_ = node_->create_client<curobo_msgs::srv::RemoveObject>(planner_ns + "remove_object");
        remove_object_request_ = std::make_shared<curobo_msgs::srv::RemoveObject_Request>();

        // create publisher so Display can retrieve the parameters to add objects and remove them
        add_object_publisher_ = node_->create_publisher<curobo_msgs::msg::ObjectParameters>("add_objects_topic", 10);
        remove_object_publisher_ = node_->create_publisher<std_msgs::msg::String>("remove_objects_topic", 10);

        // associate types to the service constants
        ui_->comboBoxObjects->addItem("Cube", QVariant(curobo_msgs::srv::AddObject_Request::CUBOID));
        ui_->comboBoxObjects->addItem("Sphere", QVariant(curobo_msgs::srv::AddObject_Request::SPHERE));
        ui_->comboBoxObjects->addItem("Capsule", QVariant(curobo_msgs::srv::AddObject_Request::CAPSULE));
        ui_->comboBoxObjects->addItem("Cylindre", QVariant(curobo_msgs::srv::AddObject_Request::CYLINDER));
        ui_->comboBoxObjects->addItem("Mesh", QVariant(curobo_msgs::srv::AddObject_Request::MESH));

        // put placeholders
        ui_->lineEditName->setPlaceholderText("object_name");
        ui_->lineEditMeshPath->setPlaceholderText("path/to/mesh");

        // create a timer to show labelMessage for 5 seconds
        timerMessage_ = new QTimer(this);
        connect(timerMessage_, SIGNAL(timeout()), ui_->labelMessage, SLOT(clear()));

        // Readiness poll: same "node_is_available" parameter curobo_rviz::RvizArgsPanel
        // polls on the planner node. Gray out Add/Remove until the planner is
        // actually responding (not just discoverable -- see NodeSpinner/that
        // class's pollPlannerReady for why the distinction matters during the
        // planner's ~90s GPU warmup, and why this must be AsyncParametersClient,
        // never SyncParametersClient).
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, planner_node);
        setPlannerReady(false);
        QTimer* readinessTimer = new QTimer(this);
        connect(readinessTimer, &QTimer::timeout, this, &AddObjectsPanel::pollPlannerReady);
        readinessTimer->start(250);

        RCLCPP_INFO(node_->get_logger(), "Initialized objects panel");

        // Spin node_ on a background thread for the rest of this panel's lifetime.
        // Constructed last, once every client/publisher above already exists.
        spinner_ = std::make_unique<curobo_rviz::NodeSpinner>(node_);
    }

    AddObjectsPanel::~AddObjectsPanel()
    {
        // spinner_ is declared last in the header, so it is destroyed FIRST here.
    }

    void AddObjectsPanel::runOnGuiThread(std::function<void()> fn)
    {
        QMetaObject::invokeMethod(this, std::move(fn), Qt::QueuedConnection);
    }

    void AddObjectsPanel::setPlannerReady(bool ready)
    {
        if (planner_ready_ == ready) {
            return;
        }
        planner_ready_ = ready;
        ui_->pushButtonAdd->setEnabled(ready);
        ui_->pushButtonRemove->setEnabled(ready);
        RCLCPP_INFO(node_->get_logger(), "Planner is %s", ready ? "ready" : "not ready");
    }

    void AddObjectsPanel::pollPlannerReady()
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

    void AddObjectsPanel::on_pushButtonAdd_clicked()
    {
        // retrieve values on the UI for some checkups
        int type = ui_->comboBoxObjects->currentData().toInt();
        std::string name = ui_->lineEditName->displayText().toStdString();
        std::string mesh_file_path = ui_->lineEditMeshPath->displayText().toStdString();

        // if name is not unique -> checked by the service
        if (name.empty()) {
            displayMessage("The object must have a name. Can't make it empty");
            RCLCPP_WARN(node_->get_logger(), "The object must have a name. Can't make it empty");
            return;
        }
        if (type == curobo_msgs::srv::AddObject_Request::MESH && mesh_file_path.empty()) {
            displayMessage("The mesh path must be specified. Can't make it empty");
            RCLCPP_WARN(node_->get_logger(), "The mesh path must be specified. Can't make it empty");
            return;
        }

        // setup request for service
        add_object_request_->type = type;
        add_object_request_->name = name;
        add_object_request_->mesh_file_path = mesh_file_path;
        add_object_request_->pose.position.x = ui_->doubleSpinBoxPositionX->value();
        add_object_request_->pose.position.y = ui_->doubleSpinBoxPositionY->value();
        add_object_request_->pose.position.z = ui_->doubleSpinBoxPositionZ->value();
        add_object_request_->pose.orientation.x = ui_->doubleSpinBoxOrientationX->value();
        add_object_request_->pose.orientation.y = ui_->doubleSpinBoxOrientationY->value();
        add_object_request_->pose.orientation.z = ui_->doubleSpinBoxOrientationZ->value();
        add_object_request_->pose.orientation.w = ui_->doubleSpinBoxOrientationW->value();
        add_object_request_->dimensions.x = ui_->doubleSpinBoxDimensionX->value();
        add_object_request_->dimensions.y = ui_->doubleSpinBoxDimensionY->value();
        add_object_request_->dimensions.z = ui_->doubleSpinBoxDimensionZ->value();
        add_object_request_->color.r = ui_->doubleSpinBoxColorA->value();
        add_object_request_->color.g = ui_->doubleSpinBoxColorG->value();
        add_object_request_->color.b = ui_->doubleSpinBoxColorB->value();
        add_object_request_->color.a = ui_->doubleSpinBoxColorA->value();

        RCLCPP_INFO(node_->get_logger(), "Sending following message to service:\n"
                                            "\ttype: %d\tname: %s\n"
                                            "\tmesh_file_path: %s\n"
                                            "\tpose: {position: %f, %f, %f}{orientation: %f, %f, %f, %f}\n"
                                            "\tdimensions: %f, %f, %f\n"
                                            "\tcolor: %f, %f, %f, %f",
                                            type, name.c_str(),
                                            mesh_file_path.c_str(),
                                            add_object_request_->pose.position.x,
                                            add_object_request_->pose.position.y,
                                            add_object_request_->pose.position.z,
                                            add_object_request_->pose.orientation.x,
                                            add_object_request_->pose.orientation.y,
                                            add_object_request_->pose.orientation.z,
                                            add_object_request_->pose.orientation.w,
                                            add_object_request_->dimensions.x,
                                            add_object_request_->dimensions.y,
                                            add_object_request_->dimensions.z,
                                            add_object_request_->color.r,
                                            add_object_request_->color.g, 
                                            add_object_request_->color.b,
                                            add_object_request_->color.a);

        if (!add_object_client_->service_is_ready()) {
            displayMessage("add_object service not available");
            RCLCPP_ERROR(node_->get_logger(), "add_object service not available");
            return;
        }

        // Built now, while add_object_request_'s fields are fresh -- captured into
        // the callback instead of re-read from the (reused) request member later.
        QString objectDisplayText = QString("%1 {pos: %2, %3, %4}{ori: %5, %6, %7, %8}")
                                            .arg(name.c_str())
                                            .arg(add_object_request_->pose.position.x)
                                            .arg(add_object_request_->pose.position.y)
                                            .arg(add_object_request_->pose.position.z)
                                            .arg(add_object_request_->pose.orientation.x)
                                            .arg(add_object_request_->pose.orientation.y)
                                            .arg(add_object_request_->pose.orientation.z)
                                            .arg(add_object_request_->pose.orientation.w);

        // Disabled until the response arrives: no spin_until_future_complete here
        // (that used to block the GUI thread for up to 5s, or forever if the
        // planner never replied), and this also keeps a second click from
        // mutating add_object_request_ while this one is still in flight.
        ui_->pushButtonAdd->setEnabled(false);
        add_object_client_->async_send_request(add_object_request_,
          [this, name, objectDisplayText](rclcpp::Client<curobo_msgs::srv::AddObject>::SharedFuture future) {
            bool success = false;
            std::string message;
            try {
              auto result = future.get();
              success = result->success;
              message = result->message;
            } catch (const std::exception & e) {
              message = e.what();
            }

            runOnGuiThread([this, success, message, name, objectDisplayText]() {
              if (success) {
                RCLCPP_INFO(node_->get_logger(), "Service call successful. %s", message.c_str());

                // call Display service to add the object on the screen
                sendObjectParameters();

                QListWidgetItem* objectItem = new QListWidgetItem(objectDisplayText);
                // store name as data for the remove service so it's easier to handle
                objectItem->setData(Qt::UserRole, QVariant(QString::fromStdString(name)));
                ui_->listWidgetObjects->addItem(objectItem);
              } else {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed. %s", message.c_str());
              }
              displayMessage(message);
              ui_->pushButtonAdd->setEnabled(planner_ready_);
            });
          });
    }

    void AddObjectsPanel::on_pushButtonRemove_clicked()
    {
        // only way to check if an object is selected is with selectedItems
        QList<QListWidgetItem *> selectedItems = ui_->listWidgetObjects->selectedItems();
        if (selectedItems.isEmpty()) {
            return;
        }

        if (!remove_object_client_->service_is_ready()) {
            displayMessage("remove_object service not available");
            RCLCPP_ERROR(node_->get_logger(), "remove_object service not available");
            return;
        }

        for (int i = 0; i < selectedItems.size(); i++) {
            // find the selected object
            QListWidgetItem* item = selectedItems.at(i);
            std::string name = item->data(Qt::UserRole).toString().toStdString();
            remove_object_request_->name = name;

            // send the request to the service to remove the object
            remove_object_client_->async_send_request(remove_object_request_,
              [this, item, name](rclcpp::Client<curobo_msgs::srv::RemoveObject>::SharedFuture future) {
                bool success = false;
                std::string message;
                try {
                  auto result = future.get();
                  success = result->success;
                  message = result->message;
                } catch (const std::exception & e) {
                  message = e.what();
                }

                runOnGuiThread([this, item, name, success, message]() {
                  if (success) {
                    // call Display service to remove the object from screen
                    auto msg = std_msgs::msg::String();
                    msg.data = name;
                    remove_object_publisher_->publish(msg);

                    // remove item from QListWidget
                    ui_->listWidgetObjects->removeItemWidget(item);
                    delete item;

                    RCLCPP_INFO(node_->get_logger(), "Service call successful. %s", message.c_str());
                  } else {
                    RCLCPP_ERROR(node_->get_logger(), "Service call failed. %s", message.c_str());
                  }
                  displayMessage(message);
                });
              });
        }
    }

    void AddObjectsPanel::sendObjectParameters() {
        // setup request
        auto msg = curobo_msgs::msg::ObjectParameters();

        msg.type = ui_->comboBoxObjects->currentData().toInt();
        msg.name = ui_->lineEditName->displayText().toStdString();
        msg.mesh_file_path = ui_->lineEditMeshPath->displayText().toStdString();
        msg.pose.position.x = ui_->doubleSpinBoxPositionX->value();
        msg.pose.position.y = ui_->doubleSpinBoxPositionY->value();
        msg.pose.position.z = ui_->doubleSpinBoxPositionZ->value();
        msg.pose.orientation.x = ui_->doubleSpinBoxOrientationX->value();
        msg.pose.orientation.y = ui_->doubleSpinBoxOrientationY->value();
        msg.pose.orientation.z = ui_->doubleSpinBoxOrientationZ->value();
        msg.pose.orientation.w = ui_->doubleSpinBoxOrientationW->value();
        msg.dimensions.x = ui_->doubleSpinBoxDimensionX->value();
        msg.dimensions.y = ui_->doubleSpinBoxDimensionY->value();
        msg.dimensions.z = ui_->doubleSpinBoxDimensionZ->value();
        msg.color.r = ui_->doubleSpinBoxColorA->value();
        msg.color.g = ui_->doubleSpinBoxColorG->value();
        msg.color.b = ui_->doubleSpinBoxColorB->value();
        msg.color.a = ui_->doubleSpinBoxColorA->value();

        // send request
        add_object_publisher_->publish(msg);
    }

    void AddObjectsPanel::displayMessage(std::string msg) {
        // show message in UI
        QString Qmsg = msg.c_str();
        ui_->labelMessage->setText(Qmsg);
        timerMessage_->start(5000); // 5 seconds
    }
} // add_objects_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_panel::AddObjectsPanel, rviz_common::Panel)