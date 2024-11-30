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
        double orientationX = ui_->doubleSpinBoxxOrientationX->value();
        double orientationY = ui_->doubleSpinBoxOrientationY->value();
        double orientationZ = ui_->doubleSpinBoxOrientationZ->value();
        double orientationW = ui_->doubleSpinBoxOrientationW->value();
        double dimX = ui_->doubleSpinBoxDimensionsX->value();
        double dimY = ui_->doubleSpinBoxDimensionsY->value();
        double dimZ = ui_->doubleSpinBoxDimensionsZ->value();
        double colorR = ui_->doubleSpinBoxColorA->value();
        double colorG = ui_->doubleSpinBoxColorG->value();
        double colorB = ui_->doubleSpinBoxColorB->value();
        double colorA = ui_->doubleSpinBoxColorA->value();
        int type;
        string name;


        // retrieve type # from type string
        QString textType = ui_->comboBoxObjects->currentData();
        switch (textType) {
            case 'Cube':
                type = add_objects_client_->CUBOID;
                break;
            case 'Sphere':
                type = add_objects_client_->SPHERE;
                break;
            case 'Capsule':
                type = add_objects_client_->CAPSULE;
                break;
            case 'Cylindre':
                type = add_objects_client_->CYLINDER;
                break;
            case 'Mesh':
                type = add_objects_client_->MESH;
                break;
            default:
                RCLCPP_WARN(node_->get_logger(), "Couldn't find the type.");
                break;
        }

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