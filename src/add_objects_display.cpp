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

        scene_manager_ = context_->getSceneManager();

        // retrieve informations from add objects panel
        add_object_subscriber_ = node_->create_subscription<curobo_msgs::msg::ObjectParameters>("add_objects_topic", 10, std::bind(&AddObjectsDisplay::onAddUpdate, this, std::placeholders::_1));
        remove_object_subscriber_ = node_->create_subscription<std_msgs::msg::String>("remove_objects_topic", 10, std::bind(&AddObjectsDisplay::onRemoveUpdate, this, std::placeholders::_1));

        // add map of all the shapes. Allows to retrieve easily the shapes by their name for removal
        shapeMap_ = std::unordered_map<std::string, std::unique_ptr<rviz_rendering::Shape>>();

        RCLCPP_INFO(node_->get_logger(), "Initialized objects display");
    }

    void AddObjectsDisplay::onAddUpdate(const curobo_msgs::msg::ObjectParameters & request)
    {
        try{
            Ogre::Vector3 position;
            Ogre::Quaternion orientation;

            auto shapeType = getShapeType(request.type);
            // TODO: Validate that the child scene is created and visible (and its content too)
            Ogre::SceneNode* shapeSceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

            std::unique_ptr<rviz_rendering::Shape> shape = std::make_unique<rviz_rendering::Shape>(shapeType, scene_manager_, shapeSceneNode);

            shape->setPosition(Ogre::Vector3(
                                    request.pose.position.x,
                                    request.pose.position.y,
                                    request.pose.position.z));
            shape->setOrientation(Ogre::Quaternion(
                                    request.pose.orientation.x,
                                    request.pose.orientation.y,
                                    request.pose.orientation.z,
                                    request.pose.orientation.w)); // must be 1 for shape to show up on screen
            shape->setScale(Ogre::Vector3(
                                    request.dimensions.x,
                                    request.dimensions.y,
                                    request.dimensions.z));
            // TODO: Check why the color setting doesn't work
            shape->setColor(request.color.r, request.color.g, request.color.b, request.color.a);

            shapeSceneNode->setPosition(position);
            shapeSceneNode->setOrientation(orientation);

            shapeMap_[request.name] = std::move(shape);

            RCLCPP_INFO(node_->get_logger(), "Added object %s", request.name.c_str());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to add shape: %s", e.what());
        } 
    }

    void AddObjectsDisplay::onRemoveUpdate(const std_msgs::msg::String & request)
    {
        // find the name of the shape to remove in the map
        auto it = shapeMap_.find(request.data.c_str());
        if (it == shapeMap_.end()) {
            RCLCPP_WARN(node_->get_logger(), "Couldn't remove the shape");
            return;
        }

        // we need to remove the scene node, its content, the shape in the shape map and reset the shape pointer to delete the object
        auto& shape = it->second;
        Ogre::SceneNode* shapeSceneNode = shape->getRootNode();
        if (shapeSceneNode) {
            shapeSceneNode->getCreator()->destroySceneNode(shapeSceneNode);
        }
        
        scene_manager_->destroySceneNode(shapeSceneNode);
        shapeMap_.erase(it);
        shape.reset();
        RCLCPP_INFO(node_->get_logger(), "Removed object named: %s", request.data.c_str());
    }

    rviz_rendering::Shape::Type AddObjectsDisplay::getShapeType(const int& type) {
        // Shape doesn't support Caspsule type, so it isn't listed here for now
        // https://docs.ros.org/en/humble/p/rviz_rendering/generated/classrviz__rendering_1_1Shape.html#_CPPv4N14rviz_rendering5Shape4TypeE
        if (type == curobo_msgs::srv::AddObject_Request::CUBOID) return rviz_rendering::Shape::Type::Cube;
        if (type == curobo_msgs::srv::AddObject_Request::SPHERE) return rviz_rendering::Shape::Type::Sphere;
        if (type == curobo_msgs::srv::AddObject_Request::CYLINDER) return rviz_rendering::Shape::Type::Cylinder;
        if (type == curobo_msgs::srv::AddObject_Request::MESH) return rviz_rendering::Shape::Type::Mesh;
        throw std::invalid_argument("Unknown shape type: " + std::to_string(type));
    }
} // add_objects_display

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_objects_display::AddObjectsDisplay, rviz_common::Display)