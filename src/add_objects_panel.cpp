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
        add_object_client_ = node_->create_client<curobo_msgs::srv::AddObject>("/curobo_gen_traj/add_object");
        add_object_request_ = std::make_shared<curobo_msgs::srv::AddObject_Request>();
        remove_object_client_ = node_->create_client<curobo_msgs::srv::RemoveObject>("/curobo_gen_traj/remove_object");
        remove_object_request_ = std::make_shared<curobo_msgs::srv::RemoveObject_Request>();

        // create publisher so Display can retrieve the parameters to add objects and remove them
        add_object_publisher_ = node_->create_publisher<curobo_msgs::srv::AddObject_Request>("add_objects_topic", 10);
        remove_object_publisher_ = node_->create_publisher<curobo_msgs::srv::RemoveObject_Request>("remove_objects_topic", 10);

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
        // retrieve values on the UI
        double posX = ui_->doubleSpinBoxPositionX->value();
        double posY = ui_->doubleSpinBoxPositionY->value();
        double posZ = ui_->doubleSpinBoxPositionZ->value();
        double orientationX = ui_->doubleSpinBoxOrientationX->value();
        double orientationY = ui_->doubleSpinBoxOrientationY->value();
        double orientationZ = ui_->doubleSpinBoxOrientationZ->value();
        double orientationW = ui_->doubleSpinBoxOrientationW->value();
        double dimX = ui_->doubleSpinBoxDimensionX->value();
        double dimY = ui_->doubleSpinBoxDimensionY->value();
        double dimZ = ui_->doubleSpinBoxDimensionZ->value();
        double colorR = ui_->doubleSpinBoxColorA->value();
        double colorG = ui_->doubleSpinBoxColorG->value();
        double colorB = ui_->doubleSpinBoxColorB->value();
        double colorA = ui_->doubleSpinBoxColorA->value();
        int type = ui_->comboBoxObjects->currentData().toInt();
        std::string name = ui_->lineEditName->displayText().toStdString();
        std::string mesh_file_path = "";

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

        RCLCPP_INFO(node_->get_logger(), "Sending following message to service:\n"
                                            "\ttype: %d\tname: %s\n"
                                            "\tmesh_file_path: %s\n"
                                            "\tpose: {position: %f, %f, %f}{orientation: %f, %f, %f, %f}\n"
                                            "\tdimensions: %f, %f, %f\n"
                                            "\tcolor: %f, %f, %f, %f",
                                            type, name.c_str(),
                                            mesh_file_path.c_str(),
                                            posX, posY, posZ, orientationX, orientationY, orientationZ, orientationW,
                                            dimX, dimY, dimZ,
                                            colorR, colorG, colorB, colorA);

        // setup request
        // auto add_object_request_ = curobo_msgs::srv::AddObject_Request();
        add_object_request_->type = type;
        add_object_request_->name = name;
        add_object_request_->pose.position.x = posX;
        add_object_request_->pose.position.y = posY;
        add_object_request_->pose.position.z = posZ;
        add_object_request_->pose.orientation.x = orientationX;
        add_object_request_->pose.orientation.y = orientationY;
        add_object_request_->pose.orientation.z = orientationZ;
        add_object_request_->pose.orientation.w = orientationW;
        add_object_request_->dimensions.x = dimX;
        add_object_request_->dimensions.y = dimY;
        add_object_request_->dimensions.z = dimZ;
        add_object_request_->color.r = colorR;
        add_object_request_->color.g = colorG;
        add_object_request_->color.b = colorB;
        add_object_request_->color.a = colorA;

        // call Add Objects with parameters
        auto future = add_object_client_->async_send_request(add_object_request_);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get(); // can only call future.get() once https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html
            if (result->success) {
                RCLCPP_INFO(node_->get_logger(), "Service call successful. %s", result->message.c_str());

                // call Display service to add the object on the screen
                add_object_publisher_->publish(*add_object_request_);

                // add item to QListWidget
                QString objectDisplayText = QString("%1 {pos: %2, %3, %4}{ori: %5, %6, %7, %8}")
                                                    .arg(name.c_str()).arg(posX).arg(posY).arg(posZ)
                                                    .arg(orientationX).arg(orientationY).arg(orientationZ).arg(orientationW);
                QListWidgetItem* objectItem = new QListWidgetItem(objectDisplayText);
                objectItem->setData(Qt::UserRole, QVariant(QString::fromStdString(name))); // store name as data for the remove service
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
        QList<QListWidgetItem *> selectedItems = ui_->listWidgetObjects->selectedItems();

        for (int i = 0; i < selectedItems.size(); i++) {

            std::string name = selectedItems.at(i)->data(Qt::UserRole).toString().toStdString();
            remove_object_request_->name = name;
            auto future = remove_object_client_->async_send_request(remove_object_request_);

            if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
                auto result = future.get(); // can only call future.get() once https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html
                
                if (result->success) {
                    // call Display service to remove the object from screen
                    remove_object_publisher_->publish(*remove_object_request_);
                    // remove item from QListWidget
                    ui_->listWidgetObjects->removeItemWidget(selectedItems.at(i));           
                    
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

    void AddObjectsPanel::displayMessage(std::string msg) {
        // show message in UI
        QString Qmsg = msg.c_str();
        ui_->labelMessage->setText(Qmsg);
        timerMessage_->start(5000); // 5 seconds
    }
} // add_objects_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_panel::AddObjectsPanel, rviz_common::Panel)