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
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);
        
        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_add_objects_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        // create add_objects & remove_objects service
        add_object_client_ = node_->create_client<curobo_msgs::srv::AddObject>("/unified_planner/add_object");
        add_object_request_ = std::make_shared<curobo_msgs::srv::AddObject_Request>();
        remove_object_client_ = node_->create_client<curobo_msgs::srv::RemoveObject>("/unified_planner/remove_object");
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

        RCLCPP_INFO(node_->get_logger(), "Initialized objects panel");
    }

    AddObjectsPanel::~AddObjectsPanel()
    {
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

        // call Add Objects with parameters
        auto future = add_object_client_->async_send_request(add_object_request_);
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get(); // can only call future.get() once https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html

            if (result->success) {
                RCLCPP_INFO(node_->get_logger(), "Service call successful. %s", result->message.c_str());

                // call Display service to add the object on the screen
                sendObjectParameters();

                // Add item to QListWidget:
                    // setup display text in the QListWidget
                QString objectDisplayText = QString("%1 {pos: %2, %3, %4}{ori: %5, %6, %7, %8}")
                                                    .arg(name.c_str())
                                                    .arg(add_object_request_->pose.position.x)
                                                    .arg(add_object_request_->pose.position.y)
                                                    .arg(add_object_request_->pose.position.z)
                                                    .arg(add_object_request_->pose.orientation.x)
                                                    .arg(add_object_request_->pose.orientation.y)
                                                    .arg(add_object_request_->pose.orientation.z)
                                                    .arg(add_object_request_->pose.orientation.w);
                QListWidgetItem* objectItem = new QListWidgetItem(objectDisplayText);
                    // store name as data for the remove service so it's easier to handle
                objectItem->setData(Qt::UserRole, QVariant(QString::fromStdString(name)));
                    // add the item
                ui_->listWidgetObjects->addItem(objectItem);

            } else {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed. %s", result->message.c_str());
            }

            displayMessage(result->message);

        } else {
            RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
        }
    }

    void AddObjectsPanel::on_pushButtonRemove_clicked()
    {
        // only way to check if an object is selected is with selectedItems
        QList<QListWidgetItem *> selectedItems = ui_->listWidgetObjects->selectedItems();

        for (int i = 0; i < selectedItems.size(); i++) {
            
            // find the selected object
            std::string name = selectedItems.at(i)->data(Qt::UserRole).toString().toStdString();
            remove_object_request_->name = name;

            // send the request to the service to remove the object
            auto future = remove_object_client_->async_send_request(remove_object_request_);

            if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
                auto result = future.get(); // can only call future.get() once https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html
                
                if (result->success) {
                    // call Display service to remove the object from screen
                    auto msg = std_msgs::msg::String();
                    msg.data = name;
                    remove_object_publisher_->publish(msg);

                    // remove item from QListWidget
                    ui_->listWidgetObjects->removeItemWidget(selectedItems.at(i));
                    delete selectedItems.at(i);
                    
                    RCLCPP_INFO(node_->get_logger(), "Service call successful. %s", result->message.c_str());

                } else {

                    RCLCPP_ERROR(node_->get_logger(), "Service call failed. %s", result->message.c_str());
                }

                displayMessage(result->message);

            } else {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
            }
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