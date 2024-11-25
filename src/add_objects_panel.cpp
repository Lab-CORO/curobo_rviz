#include <curobo_rviz/add_objects_panel.hpp>

namespace add_objects
{
    AddObjectsPanel::AddObjectsPanel(QWidget *parent)
        : Panel{parent}
        , ui_{std::make_unique<Ui::gui>()}
        , node_{nullptr}
    {
        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_push_button_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);
    }

    AddObjectsPanel::~AddObjectsPanel()
    {
    }
} // add_objects