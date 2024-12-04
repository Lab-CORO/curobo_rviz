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
        , color_property_{nullptr}
        , node_{nullptr}
        , add_object_subscriber_{nullptr}
        , remove_object_subscriber_{nullptr}
        , shapeMap_{nullptr}
    {
    }

    AddObjectsDisplay::~AddObjectsDisplay()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::~AddObjectsDisplay()");
    }

    void AddObjectsDisplay::onInitialize()
    {
        auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "--remap", "__node:=rviz_add_objects_display_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        add_object_subscriber_ = node_->create_subscription<curobo_msgs::srv::AddObject_Request>("add_objects_topic", 10, std::bind(&AddObjectsDisplay::onAddUpdate, this, std::placeholders::_1));
        remove_object_subscriber_ = node_->create_subscription<curobo_msgs::srv::RemoveObject_Request>("remove_objects_topic", 10, std::bind(&AddObjectsDisplay::onRemoveUpdate, this, std::placeholders::_1));
        
        shapeMap_ = std::unordered_map<std::string, std::unique_ptr<rviz_rendering::Shape>>();
        // shape_ = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, scene_node_);
        
        color_property_ = std::make_unique<rviz_common::properties::ColorProperty>("Point Color", QColor(204, 51, 204), "Color to draw the object.", this, SLOT(updateStyle()));
        // updateStyle();
        
        //shape_.reset();
        RCLCPP_INFO(node_->get_logger(), "Initialized objects display");
    }

    void AddObjectsDisplay::onAddUpdate(const curobo_msgs::srv::AddObject_Request::ConstSharedPtr request)
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::processMessage()");
        
        std::unique_ptr<rviz_rendering::Shape> shape = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Type::Cube, scene_manager_, scene_node_);

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        shape->setOrientation(Ogre::Quaternion(
                                request->pose.orientation.x,
                                request->pose.orientation.y,
                                request->pose.orientation.z,
                                request->pose.orientation.w));
        shape->setScale(Ogre::Vector3(
                                request->dimensions.x,
                                request->dimensions.y,
                                request->dimensins.z));
        shape->setPosition(Ogre::Vector3(
                                request->pose.position.x,
                                request->pose.position.y,
                                request->pose.position.z));
        color_property_->setColor(QColor(request->color.r, request->color.g, request->color.b));
        updateStyle(shape);

        shapeMap_[request->name] = std::move(shape);
    }

    void AddObjectsDisplay::onRemoveUpdate(const curobo_msgs::srv::RemoveObject_Request::ConstSharedPtr request)
    {
        std::unique_ptr<rviz_rendering::Shape> shape = shapeMap_.find(request->name);
        if (shape == shapeMap_.end()) {
            RCLCPP_WARN(node_->get_logger(), "Couldn't remove the shape");
            return;
        })
        shapeMap_.erase(request->name);
        shape.reset();
        RCLCPP_INFO(node_->get_logger(), "Removed object named: %s", request->name.c_str());
    }

    void AddObjectsDisplay::updateStyle(std::unique_ptr<rviz_rendering::Shape> shape)
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::updateStyle()");
        shape->setColor(color_property_->getOgreColor());
    }
} // add_objects_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_display::AddObjectsDisplay, rviz_common::Display)