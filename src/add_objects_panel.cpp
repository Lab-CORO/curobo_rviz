#include <curobo_rviz/add_objects_panel.hpp>
#include <string>

namespace add_objects
{
    AddObjectsPanel::AddObjectsPanel(QWidget *parent)
        : Panel{parent}
        , ui_{std::make_unique<Ui::gui>()}
        , node_{nullptr}
        , add_objects_client_ {nullptr}
    {
        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_add_objects_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        // create Add_objects service
        add_objects_client_ = node_->create_client<curobo_msgs::srv::AddObject>("/curobo_gen_traj/add_object");

        ui_->comboBoxObjects->addItem("Cube", QVariant(curobo_msgs::srv::AddObject::CUBOID));
        ui_->comboBoxObjects->addItem("Sphere", QVariant(curobo_msgs::srv::AddObject::SPHERE));
        ui_->comboBoxObjects->addItem("Capsule", QVariant(curobo_msgs::srv::AddObject::CAPSULE));
        ui_->comboBoxObjects->addItem("Cylindre", QVariant(curobo_msgs::srv::AddObject::CYLINDER));
        ui_->comboBoxObjects->addItem("Mesh", QVariant(curobo_msgs::srv::AddObject::MESH)); // Par défaut si MESH non défini

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
        int type;
        string name;

        // retrieve type # from type string
        type = ui_->comboBoxObjects->currentData().toInt();

        // name the object

        // call Add Objects with parameters
        // add_objects_request_ = std::make_shared<>();

        // call Display service to add the object on the screen

        // add item to QListWidget (store the name to easily retrieve the name for suppression later on)
    }

    void AddObjectsPanel::on_pushButtonRemove_clicked()
    {
        

    }
} // add_objects