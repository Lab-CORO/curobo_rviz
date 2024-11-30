#include <rviz_common/logging.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <curobo_rviz/add_objects_display.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace add_objects_display
{
    using rviz_common::properties::StatusProperty;
    
    AddObjectsDisplay::AddObjectsDisplay()
        : RosTopicDisplay{}
        , shape_{nullptr}
        , color_property_{nullptr}
        , node_{nullptr}
    {
    }

    AddObjectsDisplay::~AddObjectsDisplay()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::~AddObjectsDisplay()");
    }

    void AddObjectsDisplay::onInitialize()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::onInitialize()");
        
        shape_ = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, scene_node_);
        
        color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("Point Color", QColor(204, 51, 204), "Color to draw the object.", this, SLOT(updateStyle()));
        updateStyle();
    }

    void AddObjectsDisplay::onEnable(){
        RosTopicDisplay::onEnable();
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::onEnable()");
    }

    void AddObjectsDisplay::onDisable(){
        RosTopicDisplay::onDisable();
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::onDisable()");
    }

    void AddObjectsDisplay::processMessage(const curobo_msgs::srv::AddObject_Request::ConstSharedPtr request)
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::processMessage()");
        
        if (!shape_) {
            RCLCPP_WARN(rclcpp::get_logger("AddObjectsDisplay"), "Shape is not initialized.");
            return;
        }
        
        // shape_->setScale(Ogre::Vector3(msg->dimensions.x, msg->dimensions.y, msg->dimensions.z));
        // shape_->setPosition(Ogre::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    }

    void AddObjectsDisplay::updateStyle()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::updateStyle()");
        shape_->setColor(color_property_->getOgreColor());
    }
} // add_objects_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_display::AddObjectsDisplay, rviz_common::RosTopicDisplay<curobo_msgs::srv::AddObject_Request>)