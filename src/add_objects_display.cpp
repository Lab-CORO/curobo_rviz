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
        , node_{nullptr}
        , add_object_subscriber_{nullptr}
        , remove_object_subscriber_{nullptr}
        , shapeMap_{}
    {
    }

    AddObjectsDisplay::~AddObjectsDisplay()
    {
    }

    void AddObjectsDisplay::onInitialize()
    {
        auto options = rclcpp::NodeOptions().arguments(
                            {"--ros-args", "--remap", "__node:=rviz_display_objects_node", "--"});
        node_ = std::make_shared<rclcpp::Node>("_", options);

        add_object_subscriber_ = node_->create_subscription<curobo_msgs::srv::AddObject_Request>("add_objects_topic", 10, std::bind(&AddObjectsDisplay::onAddUpdate, this, std::placeholders::_1));
        remove_object_subscriber_ = node_->create_subscription<curobo_msgs::srv::RemoveObject_Request>("remove_objects_topic", 10, std::bind(&AddObjectsDisplay::onRemoveUpdate, this, std::placeholders::_1));
        
        shapeMap_ = std::unordered_map<std::string, std::unique_ptr<rviz_rendering::Shape>>();

        RCLCPP_INFO(node_->get_logger(), "Initialized objects display");
    }

    void AddObjectsDisplay::onAddUpdate(const curobo_msgs::srv::AddObject_Request::ConstSharedPtr request)
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::processMessage()");
        
        try{
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;

            auto shapeType = getShapeType(request->type);
            Ogre::SceneNode* shapeSceneNode = scene_manager_->createSceneNode();

            std::unique_ptr<rviz_rendering::Shape> shape = std::make_unique<rviz_rendering::Shape>(shapeType, scene_manager_, shapeSceneNode);

            shapeSceneNode->setPosition(position);
            shapeSceneNode->setOrientation(orientation);

            shape->setPosition(Ogre::Vector3(
                                    request->pose.position.x,
                                    request->pose.position.y,
                                    request->pose.position.z));
            shape->setOrientation(Ogre::Quaternion(
                                    request->pose.orientation.x,
                                    request->pose.orientation.y,
                                    request->pose.orientation.z,
                                    request->pose.orientation.w));
            shape->setScale(Ogre::Vector3(
                                    request->dimensions.x,
                                    request->dimensions.y,
                                    request->dimensions.z));
            shape->setColor(request->color.r, request->color.g, request->color.b, request->color.a);

            shapeMap_[request->name] = std::move(shape);

        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to add shape: %s", e.what());
        }
    }

    void AddObjectsDisplay::onRemoveUpdate(const curobo_msgs::srv::RemoveObject_Request::ConstSharedPtr request)
    {
        auto it = shapeMap_.find(request->name);
        if (it == shapeMap_.end()) {
            RCLCPP_WARN(node_->get_logger(), "Couldn't remove the shape");
            return;
        }

        auto& shape = it->second;
        Ogre::SceneNode* shapeSceneNode = shape->getRootNode();
        if (shapeSceneNode) {
            shapeSceneNode->getCreator()->destroySceneNode(shapeSceneNode);
        }
        scene_manager_->destroySceneNode(shapeSceneNode);
        shapeMap_.erase(it);
        shape.reset();
        RCLCPP_INFO(node_->get_logger(), "Removed object named: %s", request->name.c_str());
    }

    rviz_rendering::Shape::Type AddObjectsDisplay::getShapeType(const int& type) {
        // Shape doesn't support Caspsule type, so it isn't listed here for now
        if (type == curobo_msgs::srv::AddObject_Request::CUBOID) return rviz_rendering::Shape::Type::Cube;
        if (type == curobo_msgs::srv::AddObject_Request::SPHERE) return rviz_rendering::Shape::Type::Sphere;
        if (type == curobo_msgs::srv::AddObject_Request::CYLINDER) return rviz_rendering::Shape::Type::Cylinder;
        if (type == curobo_msgs::srv::AddObject_Request::MESH) return rviz_rendering::Shape::Type::Mesh;
        throw std::invalid_argument("Unknown shape type: " + type);
    }
} // add_objects_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_display::AddObjectsDisplay, rviz_common::Display)