#ifndef SIMPLE_MARKER_HPP
#define SIMPLE_MARKER_HPP

#include <memory>
#include <string>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "visualization_msgs/msg/interactive_marker.h"
#include "visualization_msgs/msg/interactive_marker_control.h"
#include "visualization_msgs/msg/marker.h"
#include "visualization_msgs/msg/interactive_marker_feedback.h"

#include <interactive_markers/interactive_marker_server.hpp>
class ArrowInteraction
{
public:
  /// Constructor accepts a shared pointer to an existing ROS2 node.
//   explicit ArrowInteraction(rclcpp::Node::SharedPtr node);
  explicit ArrowInteraction(std::shared_ptr<rclcpp::Node> node);
  /// Destructor.
  ~ArrowInteraction();

  /// Feedback callback to handle interactive marker events.
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  /// Create a 6-DOF interactive marker.
  void make6DofMarker(const geometry_msgs::msg::Point & position);

  /// Get current marker pose
  geometry_msgs::msg::Pose get_pose();

  /// Set marker frame ID and recreate the marker
  void setFrameId(const std::string& frame_id);

  /// Get current frame ID
  std::string getFrameId() const;

  /// Reset marker to origin position
  void resetPose();

  /// Set marker visibility
  void setVisible(bool visible);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Time last_click_time_;
  double double_click_threshold_;
  geometry_msgs::msg::Pose marker_pose_;
  std::string frame_id_;
  bool is_visible_;
};

#endif // SIMPLE_MARKER_HPP
