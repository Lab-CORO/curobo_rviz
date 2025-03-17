#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

class SimpleMarker
{
public:
  /// Constructor accepts a shared pointer to an existing ROS2 node.
  explicit SimpleMarker(rclcpp::Node::SharedPtr node)
  : node_(node),
    double_click_threshold_(0.3)
  {
    // Create the interactive marker server using the provided node.
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(node_, "simple_marker");

    // Create a publisher to publish marker poses.
    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", 10);

    // Initialize the last click time.
    last_click_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

    // Create the 6-DOF marker with an initial position (0,0,0).
    geometry_msgs::msg::Point position;
    position.x = 0.0;
    position.y = 0.0;
    position.z = 0.0;
    make6DofMarker(position);
  }

  ~SimpleMarker()
  {
    if (server_) {
      server_->clear();
    }
  }

  /// Feedback callback to handle interactive marker events.
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback & feedback)
  {
    if (feedback.event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      // Get current time from the node's clock.
      auto current_time = node_->now();
      double time_since_last_click = (current_time - last_click_time_).seconds();

      if (time_since_last_click < double_click_threshold_) {
        RCLCPP_INFO(node_->get_logger(), "%s was double-clicked", feedback.marker_name.c_str());

        // Create and populate a PoseStamped message.
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = feedback.header.frame_id;
        pose_msg.header.stamp = current_time;
        pose_msg.pose = feedback.pose;

        // Publish the pose.
        pose_publisher_->publish(pose_msg);
        RCLCPP_INFO(node_->get_logger(),
          "Published pose: position [%.2f, %.2f, %.2f] orientation [%.2f, %.2f, %.2f, %.2f]",
          pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
          pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
          pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
      }
      last_click_time_ = current_time;
    }
  }

  /// Create a 6-DOF interactive marker.
  void make6DofMarker(const geometry_msgs::msg::Point & position)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_0";
    int_marker.pose.position = position;
    int_marker.scale = 1.0;
    int_marker.name = "simple_6dof";
    int_marker.description = "Simple 6-DOF Control";

    // Create a grey arrow marker.
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.scale.x = 0.1;  // Arrow length.
    arrow_marker.scale.y = 0.02; // Arrow shaft width.
    arrow_marker.scale.z = 0.02; // Arrow shaft height.
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 0.5;
    arrow_marker.color.b = 0.5;
    arrow_marker.color.a = 1.0;

    // Create a control that always shows the arrow.
    visualization_msgs::msg::InteractiveMarkerControl arrow_control;
    arrow_control.always_visible = true;
    arrow_control.markers.push_back(arrow_marker);
    int_marker.controls.push_back(arrow_control);

    // Add 6-DOF controls (rotate and move for X, Y, and Z axes).
    {
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.w = 1.0;
      control.orientation.x = 1.0;
      control.orientation.y = 0.0;
      control.orientation.z = 0.0;
      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
    {
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.w = 1.0;
      control.orientation.x = 1.0;
      control.orientation.y = 0.0;
      control.orientation.z = 0.0;
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }
    {
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.w = 1.0;
      control.orientation.x = 0.0;
      control.orientation.y = 1.0;
      control.orientation.z = 0.0;
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
    {
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.w = 1.0;
      control.orientation.x = 0.0;
      control.orientation.y = 1.0;
      control.orientation.z = 0.0;
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }
    {
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.w = 1.0;
      control.orientation.x = 0.0;
      control.orientation.y = 0.0;
      control.orientation.z = 1.0;
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
    }
    {
      visualization_msgs::msg::InteractiveMarkerControl control;
      control.orientation.w = 1.0;
      control.orientation.x = 0.0;
      control.orientation.y = 0.0;
      control.orientation.z = 1.0;
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }

    // Insert the marker into the server and bind the feedback callback.
    server_->insert(int_marker, std::bind(&SimpleMarker::processFeedback, this, std::placeholders::_1));
    server_->applyChanges();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Time last_click_time_;
  double double_click_threshold_;
};
