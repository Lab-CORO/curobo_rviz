#include <curobo_rviz/add_objects_panel.hpp>
#include <string>

namespace add_objects_panel
{
    AddObjectsPanel::AddObjectsPanel(QWidget *parent)
        : Panel{parent}
        , ui_{std::make_unique<Ui::gui>()}
        , node_{nullptr}
        , add_object_client_ {nullptr}
        , add_object_publisher_ {nullptr}
    {
        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_add_objects_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        // create Add_objects service
        add_object_client_ = node_->create_client<curobo_msgs::srv::AddObject>("/curobo_gen_traj/add_object");

        // create publisher so Display can retrieve the parameters to add objects
        add_object_publisher_ = node_->create_publisher<curobo_msgs::srv::AddObject_Request>("add_objects_topic", 10);
        remove_object_publisher_ = node_->create_publisher<curobo_msgs::srv::RemoveObject_Request>("remove_objects_topic", 10);

        ui_->comboBoxObjects->addItem("Cube", QVariant(curobo_msgs::srv::AddObject_Request::CUBOID));
        ui_->comboBoxObjects->addItem("Sphere", QVariant(curobo_msgs::srv::AddObject_Request::SPHERE));
        ui_->comboBoxObjects->addItem("Capsule", QVariant(curobo_msgs::srv::AddObject_Request::CAPSULE));
        ui_->comboBoxObjects->addItem("Cylindre", QVariant(curobo_msgs::srv::AddObject_Request::CYLINDER));
        ui_->comboBoxObjects->addItem("Mesh", QVariant(curobo_msgs::srv::AddObject_Request::MESH)); // Par défaut si MESH non défini

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
        int type = ui_->comboBoxObjects->currentData().toInt();     // retrieve type #
        // TODO: Check if name is unique. else : error message
        std::string name = "temp_name";
        std::string mesh_file_path = "";

        RCLCPP_INFO(node_->get_logger(), "Sending following message to service:\n"
                                            "\ttype: %d\tname: %s\n"
                                            "\tmesh_file_path: %s\n"
                                            "\tpose: {position: %f, %f, %f}{orientation: %f, %f, %f}\n"
                                            "\tdimensions: %f, %f, %f\n"
                                            "\tcolor: %f, %f, %f, %f\n",
                                            type, name.c_str(),
                                            mesh_file_path.c_str(),
                                            posX, posY, posZ, orientationX, orientationY, orientationZ, orientationW,
                                            dimX, dimY, dimZ,
                                            colorR, colorG, colorB, colorA);

        // setup request
        auto add_object_request_ = curobo_msgs::srv::AddObject_Request();
        add_object_request_.type = type;
        add_object_request_.name = name;
        add_object_request_.pose.position.x = posX;
        add_object_request_.pose.position.y = posY;
        add_object_request_.pose.position.z = posZ;
        add_object_request_.pose.orientation.x = orientationX;
        add_object_request_.pose.orientation.y = orientationY;
        add_object_request_.pose.orientation.z = orientationZ;
        add_object_request_.pose.orientation.w = orientationW;
        add_object_request_.dimensions.x = dimX;
        add_object_request_.dimensions.y = dimY;
        add_object_request_.dimensions.z = dimZ;
        add_object_request_.color.r = colorR;
        add_object_request_.color.g = colorG;
        add_object_request_.color.b = colorB;
        add_object_request_.color.a = colorA;

        // call Add Objects with parameters
        // TODO: if the call works, call the display. else: error message


        // call Display service to add the object on the screen
        add_object_publisher_->publish(add_object_request_);

        // add item to QListWidget (store the name to easily retrieve the name for suppression later on)
        // TODO : show name +  pose (= position + orientation)
    }

    void AddObjectsPanel::on_pushButtonRemove_clicked()
    {
        

    }
} // add_objects_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_panel::AddObjectsPanel, rviz_common::Panel)