#include <curobo_rviz/add_objects_panel.hpp>

namespace add_objects
{
    AddObjectsPanel::AddObjectsPanel(QWidget *parent)
        : Panel{parent}
        , ui_{std::make_unique<Ui::gui>()}
        , node_{nullptr}
    {
        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_add_objects_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        // create Add_objects service
        add_objects_client_ = node_->create_client<curobo_msgs::srv::Add_objects>("/curobo_gen_traj/add_object");
    }

    AddObjectsPanel::~AddObjectsPanel()
    {
    }

    void AddObjectsPanel::on_pushButtonAdd_clicked()
    {
        // retrieve values on the UI
        double posX = doubleSpinBoxPositionX, posY = doubleSpinBoxPositionY, posZ = doubleSpinBoxPositionZ;
        double orientationX = doubleSpinBoxxOrientationX, orientationY = doubleSpinBoxOrientationY, orientationZ = doubleSpinBoxOrientationZ, orientationW = doubleSpinBoxOrientationW;
        double dimX = doubleSpinBoxDimensionsX, dimY = doubleSpinBoxDimensionsY, dimZ = doubleSpinBoxDimensionsZ;
        double colorR = doubleSpinBoxColorA, colorG = doubleSpinBoxColorG, colorB = doubleSpinBoxColorB, colorA = doubleSpinBoxColorA;

        // retrieve type # from type string

        // name the object

        // call Add Objects with parameters
        // add_objects_request_ = std::make_shared<>();

        // call Display service to add the object on the screen
    }

    void AddObjectsPanel::on_pushButtonRemove_clicked()
    {
        

    }
} // add_objects