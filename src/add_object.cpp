#include <add_object/add_object.hpp>
#include <rviz_common/logging.hpp>

namespace add_object
{
void PointDisplay::processMessage(const curobo_msgs::srv::AddObject::message msg)
{
  RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);
}
}  // namespace add_object

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(add_object::AddObjectDisplay, rviz_common::Display)