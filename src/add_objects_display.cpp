#include <rviz_common/logging.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <curobo_rviz/add_objects_display.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace add_objects_display
{
    using rviz_common::properties::StatusProperty;
    
    AddObjectsDisplay::AddObjectsDisplay()
        : Display{}
        , shape_{nullptr}
        , color_property_{nullptr}
        , node_{nullptr}
        , add_object_subscriber_{nullptr}
        , remove_object_subscriber_{nullptr}
    {
    }

    AddObjectsDisplay::~AddObjectsDisplay()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::~AddObjectsDisplay()");
    }

    void AddObjectsDisplay::onInitialize()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::onInitialize()");

        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_add_objects_display_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        add_object_subscriber_ = node_->create_subscription<curobo_msgs::srv::AddObject_Request>("add_objects_topic", 10, std::bind(&AddObjectsDisplay::onAddUpdate, this, std::placeholders::_1));
        remove_object_subscriber_ = node_->create_subscription<curobo_msgs::srv::RemoveObject_Request>("remove_objects_topic", 10, std::bind(&AddObjectsDisplay::onRemoveUpdate, this, std::placeholders::_1));
        
        shape_ = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, scene_node_);
        
        color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("Point Color", QColor(204, 51, 204), "Color to draw the object.", this, SLOT(updateStyle()));
        updateStyle();
        scene_manager_->remove_object(shape_); // testing how to remove the shape
    }

    void AddObjectsDisplay::onAddUpdate(const curobo_msgs::srv::AddObject_Request::ConstSharedPtr request)
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::processMessage()");
        
        if (!shape_) {
            RCLCPP_WARN(rclcpp::get_logger("AddObjectsDisplay"), "Shape is not initialized.");
            return;
        }

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        shape_->setOrientation(Ogre::Quaternion(
                                request->pose.orientation.x,
                                request->pose.orientation.y,
                                request->pose.orientation.z,
                                request->pose.orientation.w));
        shape_->setScale(Ogre::Vector3(
                                request->dimensions.x,
                                request->dimensions.y,
                                request->dimensions.z));
        shape_->setPosition(Ogre::Vector3(
                                request->pose.position.x,
                                request->pose.position.y,
                                request->pose.position.z));
        color_property_->setColor(QColor(request->color.r, request->color.g, request->color.b));
        updateStyle();
    }

    void AddObjectsDisplay::onRemoveUpdate(const curobo_msgs::srv::RemoveObject_Request::ConstSharedPtr request)
    {

    }

    void AddObjectsDisplay::updateStyle()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::updateStyle()");
        shape_->setColor(color_property_->getOgreColor());
    }
} // add_objects_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_display::AddObjectsDisplay, rviz_common::Display)