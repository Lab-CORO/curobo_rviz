#include "curobo_rviz/arrow_interaction.hpp"

ArrowInteraction::ArrowInteraction(std::shared_ptr<rclcpp::Node> node)
: node_(node),
  double_click_threshold_(0.3),
  frame_id_("base_0"),
  is_visible_(true)
{
  // Create the interactive marker server using the provided node.
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>( "simple_marker", node_.get());
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

ArrowInteraction::~ArrowInteraction()
{
  if (server_) {
    server_->clear();
  }
}

void ArrowInteraction::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    // Get the current pose from the feedback.
    marker_pose_ = feedback->pose;
  }

}

void ArrowInteraction::make6DofMarker(const geometry_msgs::msg::Point & position)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
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
  server_->insert(int_marker, std::bind(&ArrowInteraction::processFeedback, this, std::placeholders::_1));
  server_->applyChanges();
}

geometry_msgs::msg::Pose ArrowInteraction::get_pose(){
    return this->marker_pose_;
}

void ArrowInteraction::setFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;

  // Recreate the marker with the new frame_id
  // Preserve the current position
  geometry_msgs::msg::Point position;
  position.x = marker_pose_.position.x;
  position.y = marker_pose_.position.y;
  position.z = marker_pose_.position.z;

  // Clear existing marker
  server_->clear();

  // Recreate with new frame_id
  make6DofMarker(position);
}

std::string ArrowInteraction::getFrameId() const {
  return frame_id_;
}

void ArrowInteraction::resetPose() {
  // Reset to origin
  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;

  // Reset the stored pose
  marker_pose_.position = position;
  marker_pose_.orientation.x = 0.0;
  marker_pose_.orientation.y = 0.0;
  marker_pose_.orientation.z = 0.0;
  marker_pose_.orientation.w = 1.0;

  // Recreate the marker at origin
  server_->clear();
  make6DofMarker(position);
}

void ArrowInteraction::setVisible(bool visible) {
  is_visible_ = visible;

  if (!visible) {
    // Clear the marker to hide it
    server_->clear();
    server_->applyChanges();
  } else {
    // Recreate the marker to show it
    geometry_msgs::msg::Point position;
    position.x = marker_pose_.position.x;
    position.y = marker_pose_.position.y;
    position.z = marker_pose_.position.z;
    make6DofMarker(position);
  }
}