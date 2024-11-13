#include <rviz_common/message_filter_display.hpp>
#include <curobo_msgs/srv/AddObject.hpp>

namespace add_object
{
class AddObjectDisplay
  : public rviz_common::MessageFilterDisplay<curobo_msgs::srv::AddObject>
{
  Q_OBJECT

protected:
  void processMessage(const curobo_msgs::srv::AddObject::message msg) override;
};
}  // namespace add_object

#endif  // add_object